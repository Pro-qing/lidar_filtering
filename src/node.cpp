// 引入自定义的激光雷达滤波核心类头文件
#include "lidar_filtering/lidar_filter_core.hpp"
// YAML配置文件解析库
#include <yaml-cpp/yaml.h>
// ROS消息同步相关库，用于多传感器时间对齐
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// 激光雷达点云投影库（将2D扫描转换为3D点云）
#include <laser_geometry/laser_geometry.h> 
// PCL（Point Cloud Library）ROS转换及通用算法库
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
// Autoware相关消息类型定义
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/Waypoint.h>
#include <visualization_msgs/MarkerArray.h>

// C++标准库：内存管理、多线程、时间处理、算法、文件流、容器等
#include <new>
#include <future>
#include <thread>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <memory>
#include <iterator>

// Linux系统调用：文件监听（inotify）、轮询（poll）、文件描述符操作
#include <sys/inotify.h>
#include <poll.h>
#include <unistd.h>
// 多线程互斥锁
#include <mutex>

// 使用sensor_msgs和message_filters命名空间，简化代码
using namespace sensor_msgs;
using namespace message_filters;

// ==================== 全局变量定义 ====================

// 滤波核心类的智能指针，负责具体的点云处理逻辑
std::shared_ptr<LidarFilterCore> filter_core_ptr_;
// 激光投影对象，用于将LaserScan转换为PointCloud2
laser_geometry::LaserProjection projector_;

// ROS发布者定义
ros::Publisher pub_merged_filter_;       
ros::Publisher pub_points_raw;           
ros::Publisher pub_16_filter_, pub_mid_filter_, pub_left_filter_, pub_right_filter_;
ros::Publisher pub_16_calib_, pub_mid_calib_, pub_left_calib_, pub_right_calib_;
ros::Publisher pub_car_marker_, pub_debug_origins_;
ros::Publisher pub_charge_polygon_marker_;

// 父坐标系名称（通常是 "base_link" 或 "velodyne"）
std::string parent_frame_;
// 存储车辆尺寸的多边形顶点（用于自车滤波）
std::vector<geometry_msgs::Point> vehicleSize;
// 存储CAN总线信息（如车速等）
autoware_can_msgs::CANInfo can_info_;
// 调试模式开关，开启后会打印日志并发布调试Marker
bool debug_mode_ = false;

// 速度限制相关参数
double maxSpeed = 1.2, maxLimitDis_speed = 0.5;
double leftLimit_min = 0, leftLimit_max = 0, rightLimit_min = 0, rightLimit_max = 0;

// ==================== 结构体定义 ====================

/**
 * @brief 充电站多边形结构体
 * 用于存储检测到的充电站区域信息
 */
struct ChargingStationPolygon {
    bool detected = false;                       
    std::vector<geometry_msgs::Point> polygon_points; 
    double height = 1.5;                         
    double timestamp = 0.0;                      
    void reset() {                               
        detected = false;
        polygon_points.clear();
    }
};
ChargingStationPolygon charging_station_polygon_;

/**
 * @brief 雷达数据缓冲区结构体
 * 用于暂存每个雷达的标定后数据(calib)和滤波后数据
 */
struct LidarBuffers {
    pcl::PointCloud<pcl::PointXYZI>::Ptr calib; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr filt;  
    int raw_count = 0;                           
    LidarBuffers() {                             
        calib.reset(new pcl::PointCloud<pcl::PointXYZI>());
        filt.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
};
// 实例化四个雷达（主、中、左、右）的缓冲区
LidarBuffers buf_main, buf_mid, buf_left, buf_right;
pcl::PointCloud<pcl::PointXYZI>::Ptr g_merged_raw(new pcl::PointCloud<pcl::PointXYZI>());

/**
 * @brief 标定参数结构体
 */
struct CalibrationParams {
    double x = 0, y = 0, z = 0;       
    double yaw = 0, pitch = 0, roll = 0; 
    
    Eigen::Affine3f getMatrix() const {
        Eigen::Affine3f mat = Eigen::Affine3f::Identity();
        mat.translation() << x, y, z;
        mat.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
        mat.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
        mat.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
        return mat;
    }
};

/**
 * @brief 滤波参数结构体
 */
struct FilterParams { 
    int enable = 1;                 
    double min_angle = 0, max_angle = 360, max_dis = 100; 
    double a = 0, b = 360, c = 0, d = 360; 
};

// 全局互斥锁，用于保护标定参数的线程安全访问
std::mutex calib_mutex;
CalibrationParams p_main, p_mid, p_left, p_right;

// ==================== 场景化配置数据结构 ====================

/**
 * @brief 滤波配置快照（Profile）
 * 包含针对某一特定场景（Behavior）需要的所有参数集合
 */
struct FilterProfile {
    NativeFilterConfig core;
    FilterParams f_main, f_mid, f_left, f_right;
    bool enable_single_lidar = true; // 默认开启左右单线雷达
};

// 基础默认配置
FilterProfile base_profile_;
// 行为 ID 对应的配置字典快照
std::map<int, FilterProfile> behavior_profiles_;

// 当前正在使用的激活配置
FilterProfile current_profile_;
std::mutex profile_mutex_; // 保护 current_profile_ 的读写锁
int current_behavior_state_ = -1; // 当前激活的行为ID


// ==================== 配置加载函数 ====================

/**
 * @brief 从YAML文件加载车辆尺寸配置
 */
void reloadVehicleSizeYaml(const std::string& filepath) {
    try {
        YAML::Node yaml = YAML::LoadFile(filepath);
        if (yaml["vehicle_rect"]) {
            std::vector<geometry_msgs::Point> new_rect;
            for (const auto& pt : yaml["vehicle_rect"]) {
                geometry_msgs::Point p;
                p.x = pt["x"].as<double>(); p.y = pt["y"].as<double>(); p.z = 0;
                new_rect.push_back(p);
            }
            if(!new_rect.empty()) vehicleSize = new_rect;
        } else if (yaml["rect"]) { 
            std::vector<geometry_msgs::Point> new_rect;
            for (const auto& pt : yaml["rect"]) {
                geometry_msgs::Point p;
                p.x = pt["x"].as<double>(); p.y = pt["y"].as<double>(); p.z = 0;
                new_rect.push_back(p);
            }
            if(!new_rect.empty()) vehicleSize = new_rect;
        }
        
        // 更新车辆高度到当前激活的配置并下发
        std::lock_guard<std::mutex> lock(profile_mutex_);
        if(yaml["vehicle_height"]) {
            base_profile_.core.vehicle_height = yaml["vehicle_height"].as<double>(1.0);
            current_profile_.core.vehicle_height = base_profile_.core.vehicle_height;
        }
        if (filter_core_ptr_) filter_core_ptr_->updateNativeConfig(current_profile_.core);
        
        ROS_INFO("\033[1;32m[Config] Loaded Vehicle Size from %s!\033[0m", filepath.c_str());
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to parse Vehicle Size YAML: %s", e.what());
    }
}

/**
 * @brief 从YAML文件加载雷达标定参数
 */
void reloadCalibrationYaml(const std::string& filepath) {
    try {
        YAML::Node yaml = YAML::LoadFile(filepath);
        if (yaml["tf_calibration"]) {
            std::lock_guard<std::mutex> lock(calib_mutex);
            auto tf = yaml["tf_calibration"];
            auto parseCalib = [&](const std::string& prefix, CalibrationParams& cp) {
                if (tf[prefix + "_x"]) cp.x = tf[prefix + "_x"].as<double>(cp.x);
                if (tf[prefix + "_y"]) cp.y = tf[prefix + "_y"].as<double>(cp.y);
                if (tf[prefix + "_z"]) cp.z = tf[prefix + "_z"].as<double>(cp.z);
                if (tf[prefix + "_yaw"]) cp.yaw = tf[prefix + "_yaw"].as<double>(cp.yaw);
                if (tf[prefix + "_pitch"]) cp.pitch = tf[prefix + "_pitch"].as<double>(cp.pitch);
                if (tf[prefix + "_roll"]) cp.roll = tf[prefix + "_roll"].as<double>(cp.roll);
            };
            parseCalib("main", p_main);
            parseCalib("top", p_mid);
            parseCalib("left", p_left);
            parseCalib("right", p_right);
        }
        ROS_INFO("\033[1;32m[Config] Loaded Calibration from %s!\033[0m", filepath.c_str());
    } catch (const YAML::Exception& e) { 
        ROS_ERROR("Failed to parse Calibration YAML: %s", e.what()); 
    }
}

/**
 * @brief 从YAML文件加载基础滤波参数与行为覆写字典
 */
void reloadFilterParamsYaml(const std::string& filepath) {
    try {
        YAML::Node yaml = YAML::LoadFile(filepath);

        // --- Lambda: 解析雷达区域参数 ---
        auto parseRegions = [](const YAML::Node& node, FilterProfile& profile) {
            auto parseRegion = [&](const std::string& name, FilterParams& fp) {
                if(node[name]) {
                    auto n = node[name];
                    if(n["enable"]) fp.enable = n["enable"].as<int>(fp.enable);
                    if(n["min_angle"]) fp.min_angle = n["min_angle"].as<double>(fp.min_angle);
                    if(n["max_angle"]) fp.max_angle = n["max_angle"].as<double>(fp.max_angle);
                    if(n["max_dis"]) fp.max_dis = n["max_dis"].as<double>(fp.max_dis);
                    if(n["a"]) fp.a = n["a"].as<double>(fp.a); 
                    if(n["b"]) fp.b = n["b"].as<double>(fp.b);
                    if(n["c"]) fp.c = n["c"].as<double>(fp.c); 
                    if(n["d"]) fp.d = n["d"].as<double>(fp.d);
                }
            };
            parseRegion("main", profile.f_main); 
            parseRegion("top", profile.f_mid); 
            parseRegion("left", profile.f_left); 
            parseRegion("right", profile.f_right);
        };

        // --- Lambda: 解析核心Core参数 ---
        auto parseCore = [](const YAML::Node& cr, NativeFilterConfig& cfg) {
            if (cr["crop_radius"]) cfg.crop_radius = cr["crop_radius"].as<double>(cfg.crop_radius);
            if (cr["crop_radius_x"]) cfg.crop_radius_x = cr["crop_radius_x"].as<double>(cfg.crop_radius_x);
            if (cr["height_max"]) cfg.height_max = cr["height_max"].as<double>(cfg.height_max);
            if (cr["height_min"]) cfg.height_min = cr["height_min"].as<double>(cfg.height_min);
            if (cr["height_filt"]) cfg.height_filt = cr["height_filt"].as<double>(cfg.height_filt);
            if (cr["filter_floor"]) cfg.filter_floor = cr["filter_floor"].as<bool>(cfg.filter_floor);
            if (cr["voxel_filter"]) cfg.voxel_filter = cr["voxel_filter"].as<double>(cfg.voxel_filter);
            if (cr["voxel_filter_eleva"]) cfg.voxel_filter_eleva = cr["voxel_filter_eleva"].as<double>(cfg.voxel_filter_eleva);
            if (cr["filter_transient"]) cfg.filter_transient = cr["filter_transient"].as<bool>(cfg.filter_transient);
            if (cr["radius_enble"]) cfg.radius_enble = cr["radius_enble"].as<bool>(cfg.radius_enble);
            if (cr["radius_radius"]) cfg.radius_radius = cr["radius_radius"].as<double>(cfg.radius_radius);
            if (cr["radius_min_neighbors"]) cfg.radius_min_neighbors = cr["radius_min_neighbors"].as<int>(cfg.radius_min_neighbors);
            if (cr["neighboring_points"]) cfg.neighboring_points = cr["neighboring_points"].as<int>(cfg.neighboring_points);
            if (cr["stand_threshold"]) cfg.stand_threshold = cr["stand_threshold"].as<double>(cfg.stand_threshold);
            if (cr["time_consistency_filter"]) cfg.time_consistency_filter = cr["time_consistency_filter"].as<bool>(cfg.time_consistency_filter);
            if (cr["charge_enble"]) cfg.charge_enble = cr["charge_enble"].as<bool>(cfg.charge_enble);
            if (cr["consistency_enable"]) cfg.consistency_enable = cr["consistency_enable"].as<bool>(cfg.consistency_enable);
        };

        // 1. 读取全局速度限制参数
        if (yaml["speed_limits"]) {
            auto s = yaml["speed_limits"];
            if(s["maxSpeed"]) maxSpeed = s["maxSpeed"].as<double>(maxSpeed);
            if(s["maxLimitDis_speed"]) maxLimitDis_speed = s["maxLimitDis_speed"].as<double>(maxLimitDis_speed);
            if(s["leftLimit_min_angle"]) leftLimit_min = s["leftLimit_min_angle"].as<double>(leftLimit_min);
            if(s["leftLimit_max_angle"]) leftLimit_max = s["leftLimit_max_angle"].as<double>(leftLimit_max);
            if(s["rightLimit_min_angle"]) rightLimit_min = s["rightLimit_min_angle"].as<double>(rightLimit_min);
            if(s["rightLimit_max_angle"]) rightLimit_max = s["rightLimit_max_angle"].as<double>(rightLimit_max);
        }

        // 2. 解析基础配置 (Base Profile)
        FilterProfile temp_base = base_profile_; 
        if (yaml["regions"]) parseRegions(yaml["regions"], temp_base);
        if (yaml["core"]) parseCore(yaml["core"], temp_base.core);
        
        // 兼容原有的独立层级 consistency 和 charge
        if (yaml["consistency"]) {
            auto cs = yaml["consistency"];
            if(cs["enable"]) temp_base.core.consistency_enable = cs["enable"].as<bool>(temp_base.core.consistency_enable);
            if(cs["min_angle"]) temp_base.core.consistency_min_angle = cs["min_angle"].as<double>(temp_base.core.consistency_min_angle);
            if(cs["max_angle"]) temp_base.core.consistency_max_angle = cs["max_angle"].as<double>(temp_base.core.consistency_max_angle);
            if(cs["diff_dist"]) temp_base.core.consistency_diff_dist = cs["diff_dist"].as<double>(temp_base.core.consistency_diff_dist);
        }
        if (yaml["charge"]) {
            auto cg = yaml["charge"];
            if(cg["enble"]) temp_base.core.charge_enble = cg["enble"].as<bool>(temp_base.core.charge_enble);
            if(cg["length"]) temp_base.core.charge_length = cg["length"].as<double>(temp_base.core.charge_length);
            if(cg["wide"]) temp_base.core.charge_wide = cg["wide"].as<double>(temp_base.core.charge_wide);
            if(cg["high"]) temp_base.core.charge_high = cg["high"].as<double>(temp_base.core.charge_high);
            if(cg["error"]) temp_base.core.charge_error = cg["error"].as<double>(temp_base.core.charge_error);
        }

        // 3. 解析行为覆写字典 (Behaviors Overrides)
        std::map<int, FilterProfile> temp_profiles;
        if (yaml["behaviors"]) {
            for (YAML::const_iterator it = yaml["behaviors"].begin(); it != yaml["behaviors"].end(); ++it) {
                int behavior_id = it->first.as<int>();
                YAML::Node overrides = it->second;

                // 拷贝基础配置作为底板，避免丢失未复写参数
                FilterProfile profile = temp_base;

                // 覆盖解析
                if (overrides["enable_single_lidar"]) profile.enable_single_lidar = overrides["enable_single_lidar"].as<bool>();
                if (overrides["regions"]) parseRegions(overrides["regions"], profile);
                if (overrides["core"]) parseCore(overrides["core"], profile.core);

                temp_profiles[behavior_id] = profile;
            }
        }

        // 4. 加锁更新全局字典，并重置状态触发应用
        {
            std::lock_guard<std::mutex> lock(profile_mutex_);
            base_profile_ = temp_base;
            behavior_profiles_ = temp_profiles;
            
            // 如果是初始化或者尚未处于特殊状态，直接更新当前配置为基础配置
            if (current_behavior_state_ == -1) {
                current_profile_ = base_profile_;
            }
        }
        
        // 重置状态，迫使下一次 LQR 回调强制刷新核心配置
        current_behavior_state_ = -1; 
        
        // 初次加载时推送一次配置到底层核心
        if (filter_core_ptr_) {
            std::lock_guard<std::mutex> lock(profile_mutex_);
            filter_core_ptr_->updateNativeConfig(current_profile_.core);
        }

        ROS_INFO("\033[1;32m[Config] Loaded Filter Params with Behaviors from %s!\033[0m", filepath.c_str());
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to parse Filter Params YAML: %s", e.what());
    }
}

// ==================== 文件监听线程 ====================

/**
 * @brief 监听配置文件目录，实现配置热更新
 */
void watchMultipleConfigs(const std::string& dir_path, 
                          const std::string& f_vehicle, 
                          const std::string& f_calib, 
                          const std::string& f_filter) {
    if (dir_path.empty()) return;

    int fd = inotify_init1(IN_NONBLOCK);
    int wd = inotify_add_watch(fd, dir_path.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);
    if (wd < 0) { close(fd); return; }

    struct pollfd pfd; pfd.fd = fd; pfd.events = POLLIN; char buffer[2048];

    while (ros::ok()) {
        if (poll(&pfd, 1, 1000) > 0 && (pfd.revents & POLLIN)) {
            int len = read(fd, buffer, sizeof(buffer));
            bool reload_veh = false, reload_cal = false, reload_fil = false;

            for (int i = 0; i < len;) {
                struct inotify_event *event = (struct inotify_event *) &buffer[i];
                if (event->len) {
                    std::string changed_file = event->name;
                    if (changed_file == f_vehicle) reload_veh = true;
                    if (changed_file == f_calib) reload_cal = true;
                    if (changed_file == f_filter) reload_fil = true;
                }
                i += sizeof(struct inotify_event) + event->len;
            }

            if (reload_veh || reload_cal || reload_fil) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
            }

            if (reload_veh) reloadVehicleSizeYaml(dir_path + "/" + f_vehicle);
            if (reload_cal) reloadCalibrationYaml(dir_path + "/" + f_calib);
            if (reload_fil) reloadFilterParamsYaml(dir_path + "/" + f_filter);
        }
    }
    inotify_rm_watch(fd, wd); close(fd);
}

// ==================== 回调函数与核心逻辑 ====================

void canInfoCallback(const autoware_can_msgs::CANInfo::ConstPtr &msg) { can_info_ = *msg; }

void publishSensorMarkers(const std_msgs::Header& header) {
    if (pub_debug_origins_.getNumSubscribers() == 0) return;
    
    visualization_msgs::MarkerArray markers;
    auto add_marker = [&](const CalibrationParams& p, int id, float r, float g, float b, std::string text) {
        visualization_msgs::Marker m; m.header = header; m.header.frame_id = parent_frame_;
        m.ns = "sensor_origins"; m.id = id; m.type = visualization_msgs::Marker::SPHERE; m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = p.x; m.pose.position.y = p.y; m.pose.position.z = p.z;
        m.pose.orientation.w = 1.0; m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2;
        m.color.a = 0.8; m.color.r = r; m.color.g = g; m.color.b = b;
        markers.markers.push_back(m);
        
        visualization_msgs::Marker t; t.header = header; t.header.frame_id = parent_frame_;
        t.ns = "sensor_text"; t.id = id + 10; t.type = visualization_msgs::Marker::TEXT_VIEW_FACING; t.action = visualization_msgs::Marker::ADD;
        t.pose.position.x = p.x; t.pose.position.y = p.y; t.pose.position.z = p.z + 0.2; t.scale.z = 0.2;
        t.color.a = 1.0; t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.text = text;
        markers.markers.push_back(t);
    };

    CalibrationParams lm, lmid, lleft, lright;
    {
        std::lock_guard<std::mutex> lock(calib_mutex);
        lm = p_main; lmid = p_mid; lleft = p_left; lright = p_right;
    }

    add_marker(lm, 0, 1.0, 0.0, 0.0, "Main");  
    add_marker(lmid,  1, 0.0, 1.0, 0.0, "Mid");   
    add_marker(lleft, 2, 0.0, 0.0, 1.0, "Left");  
    add_marker(lright, 3, 1.0, 1.0, 0.0, "Right"); 
    
    pub_debug_origins_.publish(markers);
}

void publishChargingStationPolygon(const std_msgs::Header& header) {
    if (!charging_station_polygon_.detected || charging_station_polygon_.polygon_points.size() < 3) return;
    
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker polygon_marker;
    polygon_marker.header = header;
    polygon_marker.header.frame_id = parent_frame_;
    polygon_marker.ns = "charging_station_polygon";
    polygon_marker.id = 0;
    polygon_marker.type = visualization_msgs::Marker::LINE_STRIP;
    polygon_marker.action = visualization_msgs::Marker::ADD;
    polygon_marker.scale.x = 0.1; 
    polygon_marker.color.r = 0.0; polygon_marker.color.g = 1.0; polygon_marker.color.b = 0.0; polygon_marker.color.a = 1.0;
    polygon_marker.points = charging_station_polygon_.polygon_points;
    if (!charging_station_polygon_.polygon_points.empty()) polygon_marker.points.push_back(charging_station_polygon_.polygon_points[0]);
    for (auto& point : polygon_marker.points) point.z = 0;
    markers.markers.push_back(polygon_marker);
    
    visualization_msgs::Marker polygon_top_marker = polygon_marker;
    polygon_top_marker.id = 1;
    for (auto& point : polygon_top_marker.points) point.z = charging_station_polygon_.height;
    markers.markers.push_back(polygon_top_marker);
    
    visualization_msgs::Marker vertical_lines_marker = polygon_marker;
    vertical_lines_marker.id = 2;
    vertical_lines_marker.type = visualization_msgs::Marker::LINE_LIST;  
    vertical_lines_marker.points.clear();
    for (size_t i = 0; i < charging_station_polygon_.polygon_points.size(); ++i) {
        geometry_msgs::Point bottom_point = charging_station_polygon_.polygon_points[i];
        bottom_point.z = 0;
        geometry_msgs::Point top_point = charging_station_polygon_.polygon_points[i];
        top_point.z = charging_station_polygon_.height;
        vertical_lines_marker.points.push_back(bottom_point);
        vertical_lines_marker.points.push_back(top_point);
    }
    markers.markers.push_back(vertical_lines_marker);
    
    geometry_msgs::Point center_point;
    for (const auto& point : charging_station_polygon_.polygon_points) {
        center_point.x += point.x; center_point.y += point.y; center_point.z += point.z;
    }
    center_point.x /= charging_station_polygon_.polygon_points.size();
    center_point.y /= charging_station_polygon_.polygon_points.size();
    center_point.z = charging_station_polygon_.height / 2;
    
    visualization_msgs::Marker center_marker;
    center_marker.header = header;
    center_marker.header.frame_id = parent_frame_;
    center_marker.ns = "charging_station_center";
    center_marker.id = 3;
    center_marker.type = visualization_msgs::Marker::SPHERE;
    center_marker.action = visualization_msgs::Marker::ADD;
    center_marker.pose.position = center_point;
    center_marker.pose.orientation.w = 1.0;
    center_marker.scale.x = 0.3; center_marker.scale.y = 0.3; center_marker.scale.z = 0.3;
    center_marker.color.r = 1.0; center_marker.color.g = 0.0; center_marker.color.b = 0.0; center_marker.color.a = 0.7;
    markers.markers.push_back(center_marker);
    
    visualization_msgs::Marker text_marker;
    text_marker.header = header;
    text_marker.header.frame_id = parent_frame_;
    text_marker.ns = "charging_station_text";
    text_marker.id = 4;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position = center_point;
    text_marker.pose.position.z += charging_station_polygon_.height + 0.5;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.5;  
    text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0;
    text_marker.text = "Charging Station";
    markers.markers.push_back(text_marker);
    
    for (auto& marker : markers.markers) { marker.lifetime = ros::Duration(0.5); }
    pub_charge_polygon_marker_.publish(markers);
}

void projectScanToCloud(sensor_msgs::LaserScan& scan_in, 
                        const CalibrationParams& calib, 
                        const FilterParams& filter, 
                        pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud) 
{
    if (!filter.enable) return;

    filter_core_ptr_->filterScanMsg(scan_in, filter.a, filter.b, filter.max_dis, false, 0, 0);

    sensor_msgs::PointCloud2 cloud_msg;
    try {
        projector_.projectLaser(scan_in, cloud_msg);
    } catch (...) {
        return;
    }

    if (cloud_msg.width > 0) {
        pcl::PointCloud<pcl::PointXYZI> tmp_pcl;
        pcl::fromROSMsg(cloud_msg, tmp_pcl);
        pcl::transformPointCloud(tmp_pcl, tmp_pcl, calib.getMatrix());
        *output_cloud += tmp_pcl; 
    }
}

void processCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, CalibrationParams calib, FilterParams filter, LidarBuffers& buf, bool use_pcl = false) {
    buf.calib->clear(); buf.filt->clear(); buf.raw_count = cloud_msg->width * cloud_msg->height;
    pcl::fromROSMsg(*cloud_msg, *buf.calib);
    pcl::transformPointCloud(*buf.calib, *buf.calib, calib.getMatrix());
    
    if (filter.enable) {
        static pcl::PointCloud<pcl::PointXYZI>::Ptr env_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        env_cloud->clear();
        
        if (use_pcl) filter_core_ptr_->pointcloud_filter_pcl(buf.calib, env_cloud, false);
        else filter_core_ptr_->pointcloud_filter(buf.calib, env_cloud, false);
        
        filter_core_ptr_->filterVehicleBody(env_cloud, buf.filt, vehicleSize);
    }
}

struct SingleLineNoiseFilter {
    std::map<int, int> history_hits; 
    int FRAME_THRESHOLD = 3;          
    float DIST_THRESHOLD = 1.6f;     
    float INTENSITY_THRESHOLD = 5.0f; 

    pcl::PointCloud<pcl::PointXYZI>::Ptr near_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr far_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ror_cleaned;
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_near_cloud;
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror_filter;

    SingleLineNoiseFilter() {
        near_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        far_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        ror_cleaned.reset(new pcl::PointCloud<pcl::PointXYZI>());
        final_near_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        
        ror_filter.setRadiusSearch(0.15);
        ror_filter.setMinNeighborsInRadius(2); 
    }

    void process(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        if (cloud->empty()) return;

        near_cloud->clear();
        far_cloud->clear();
        ror_cleaned->clear();
        final_near_cloud->clear();

        for (const auto& pt : *cloud) {
            float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (dist < DIST_THRESHOLD) {
                if (pt.intensity < INTENSITY_THRESHOLD) continue; 
                near_cloud->push_back(pt);
            } else {
                far_cloud->push_back(pt);
            }
        }

        if (near_cloud->empty()) {
            history_hits.clear();
            cloud->swap(*far_cloud); 
            return;
        }

        ror_filter.setInputCloud(near_cloud);
        ror_filter.filter(*ror_cleaned);

        std::map<int, int> current_hits;
        for (const auto& pt : *ror_cleaned) {
            float angle = std::atan2(pt.y, pt.x) * 180.0 / M_PI;
            int angle_idx = static_cast<int>(angle * 0.025); 

            current_hits[angle_idx] = history_hits[angle_idx] + 1;

            if (current_hits[angle_idx] >= FRAME_THRESHOLD) {
                final_near_cloud->push_back(pt);
            }
        }
        history_hits = current_hits;

        cloud->clear();
        cloud->reserve(final_near_cloud->size() + far_cloud->size());
        *cloud += *final_near_cloud;
        *cloud += *far_cloud;
    }
};

/**
 * @brief 多传感器时间同步回调函数
 */
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg_16, 
              const sensor_msgs::PointCloud2::ConstPtr &msg_mid, 
              const sensor_msgs::LaserScan::ConstPtr &msg_left, 
              const sensor_msgs::LaserScan::ConstPtr &msg_right) 
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // 1. 获取当前线程安全的标定参数副本
    CalibrationParams local_p_main, local_p_mid, local_p_left, local_p_right;
    {
        std::lock_guard<std::mutex> lock(calib_mutex);
        local_p_main = p_main; local_p_mid = p_mid; local_p_left = p_left; local_p_right = p_right;
    }

    // 2. 获取当前激活动态配置快照（Profile）副本
    FilterProfile active_profile;
    {
        std::lock_guard<std::mutex> lock(profile_mutex_);
        active_profile = current_profile_;
    }

    // 3. 处理 3D 雷达 (使用 active_profile 中的过滤参数)
    processCloud(msg_16, local_p_main, active_profile.f_main, buf_main, false);
    processCloud(msg_mid, local_p_mid, active_profile.f_mid, buf_mid, true);

    // 4. 处理单线雷达 (先行合并)
    static pcl::PointCloud<pcl::PointXYZI>::Ptr combined_2d_raw(new pcl::PointCloud<pcl::PointXYZI>());
    combined_2d_raw->clear();

    sensor_msgs::LaserScan scan_left_copy = *msg_left;
    sensor_msgs::LaserScan scan_right_copy = *msg_right;

    projectScanToCloud(scan_left_copy, local_p_left, active_profile.f_left, combined_2d_raw);
    projectScanToCloud(scan_right_copy, local_p_right, active_profile.f_right, combined_2d_raw);

    // 5. 统一执行 2D 去噪
    static SingleLineNoiseFilter unified_2d_noise_filter;
    unified_2d_noise_filter.process(combined_2d_raw);

    // 6. 统一执行车体过滤
    buf_left.filt->clear();
    if (!combined_2d_raw->empty()) {
        filter_core_ptr_->filterVehicleBody(combined_2d_raw, buf_left.filt, vehicleSize);
    }

    size_t total_filt = buf_main.filt->size() + buf_mid.filt->size() + buf_left.filt->size();

    // 7. 合并最终输出
    static pcl::PointCloud<pcl::PointXYZI>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZI>());
    final_output->clear();

    if (pub_merged_filter_.getNumSubscribers() > 0) {
        *final_output += *buf_main.filt; 
        *final_output += *buf_mid.filt; 
        
        // 使用配置字典中的单线雷达开关
        if (active_profile.enable_single_lidar) {
            *final_output += *buf_left.filt; 
        }
        
        sensor_msgs::PointCloud2 msg; 
        pcl::toROSMsg(*final_output, msg);
        msg.header = msg_16->header; 
        msg.header.frame_id = parent_frame_;
        pub_merged_filter_.publish(msg);
    }

    // 8. Debug 统计输出
    if (debug_mode_) {
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms = end_time - start_time;
        
        ROS_INFO_THROTTLE(1.0, "\n[Debug] Time: %.2f ms | Filtered: %lu -> %lu", 
                          ms.count(), total_filt, final_output->size());
        
        publishSensorMarkers(msg_16->header);
    }

    // 9. 发布其他调试话题
    auto publish_cloud = [&](ros::Publisher& pub, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        if (pub.getNumSubscribers() > 0 && !cloud->empty()) { 
            sensor_msgs::PointCloud2 m; pcl::toROSMsg(*cloud, m);
            m.header = msg_16->header; m.header.frame_id = parent_frame_; pub.publish(m);
        }
    };

    publish_cloud(pub_16_filter_, buf_main.filt);
    publish_cloud(pub_mid_filter_, buf_mid.filt);
    publish_cloud(pub_left_filter_, buf_left.filt);

    if (pub_car_marker_.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray ma; 
        ma.markers.push_back(filter_core_ptr_->pubVehicleModel(vehicleSize));
        pub_car_marker_.publish(ma);
    }
}

/**
 * @brief LQR路径点回调：基于 YAML 配置的查表式参数切换
 */
void lqrWaypointCallback(const autoware_msgs::Waypoint::ConstPtr& msg) {
    if (!msg) return;

    // 1. 查找最高优先级的匹配状态
    // 定义优先级顺序（靠前的优先响应，比如电梯场景优先于人行横道）
    std::vector<int> priority_list = {4, 2, 5, 6}; 
    int target_state = 0; // 0代表正常基础模式

    for (int p : priority_list) {
        if (std::find(msg->wpsattr.routeBehavior.begin(), msg->wpsattr.routeBehavior.end(), p) != msg->wpsattr.routeBehavior.end()) {
            target_state = p;
            break;
        }
    }

    // 2. 防抖：状态未改变则跳过
    if (target_state == current_behavior_state_) return;

    // 3. 查表获取对应的 Profile
    FilterProfile target_profile;
    {
        std::lock_guard<std::mutex> lock(profile_mutex_);
        // 在行为字典中查找
        auto it = behavior_profiles_.find(target_state);
        if (it != behavior_profiles_.end()) {
            target_profile = it->second;
            ROS_WARN("[Perception] Switched to Behavior Profile: %d", target_state);
        } else {
            target_profile = base_profile_;
            ROS_INFO("[Perception] Switched to Base Profile (Normal)");
        }
        
        // 更新当前激活状态
        current_profile_ = target_profile;
        current_behavior_state_ = target_state;
    }

    // 4. 更新底层的核心滤波参数
    if (filter_core_ptr_) {
        filter_core_ptr_->updateNativeConfig(target_profile.core);
    }
}

// ==================== 主函数 ====================

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_points");
    ros::NodeHandle nh; 
    ros::NodeHandle p_nh("~"); 
    
    p_nh.param<std::string>("parent_frame", parent_frame_, "velodyne");
    p_nh.param("debug_mode", debug_mode_, false);

    // 初始化默认车辆尺寸矩形
    geometry_msgs::Point p_init; 
    p_init.x=1; p_init.y=1; p_init.z=0; vehicleSize.push_back(p_init); 
    p_init.x=1; p_init.y=-1; vehicleSize.push_back(p_init); 
    p_init.x=-1; p_init.y=-1; vehicleSize.push_back(p_init); 
    p_init.x=-1; p_init.y=1; vehicleSize.push_back(p_init); 

    std::string path_vehicle, path_calib, path_filter;
    p_nh.param<std::string>("vehicle_size_file", path_vehicle, "");
    p_nh.param<std::string>("calibration_file", path_calib, "");
    p_nh.param<std::string>("filter_params_file", path_filter, "");

    filter_core_ptr_ = std::make_shared<LidarFilterCore>(nh, p_nh);

    // 初次加载配置文件
    if (!path_vehicle.empty()) reloadVehicleSizeYaml(path_vehicle);
    if (!path_calib.empty()) reloadCalibrationYaml(path_calib);
    if (!path_filter.empty()) reloadFilterParamsYaml(path_filter);

    // 开启目录热更新监听
    if (!path_vehicle.empty()) {
        size_t last_slash = path_vehicle.find_last_of('/');
        if (last_slash != std::string::npos) {
            std::string watch_dir = path_vehicle.substr(0, last_slash);
            std::string name_veh = path_vehicle.substr(last_slash + 1);
            std::string name_cal = path_calib.substr(path_calib.find_last_of('/') + 1);
            std::string name_fil = path_filter.substr(path_filter.find_last_of('/') + 1);

            std::thread watcher_thread(watchMultipleConfigs, watch_dir, name_veh, name_cal, name_fil);
            watcher_thread.detach(); 
        }
    }

    pub_merged_filter_ = nh.advertise<PointCloud2>("points_filter", 5);
    pub_points_raw = nh.advertise<PointCloud2>("points_raw", 5);
    pub_16_filter_    = nh.advertise<PointCloud2>("points_16_filter", 5);
    pub_mid_filter_   = nh.advertise<PointCloud2>("points_mid_filter", 5);
    pub_left_filter_  = nh.advertise<PointCloud2>("scan_left_filter", 5);
    pub_right_filter_ = nh.advertise<PointCloud2>("scan_right_filter", 5);
    pub_16_calib_     = nh.advertise<PointCloud2>("points_16_calibration", 5);
    pub_mid_calib_    = nh.advertise<PointCloud2>("points_mid_calibration", 5);
    pub_left_calib_   = nh.advertise<PointCloud2>("points_left_calibration", 5);
    pub_right_calib_  = nh.advertise<PointCloud2>("points_right_calibration", 5);
    pub_car_marker_   = nh.advertise<visualization_msgs::MarkerArray>("car", 1, true);
    pub_debug_origins_ = nh.advertise<visualization_msgs::MarkerArray>("debug/sensor_origins", 1, true);
    pub_charge_polygon_marker_ = nh.advertise<visualization_msgs::MarkerArray>("charge", 10);

    ros::Subscriber sub_can = nh.subscribe("/can_info", 10, &canInfoCallback);
    
    message_filters::Subscriber<PointCloud2> sub_16(nh, "/points_16", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<PointCloud2> sub_mid(nh, "/points_mid", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<LaserScan> sub_left(nh, "/scan_left", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<LaserScan> sub_right(nh, "/scan_right", 1, ros::TransportHints().tcpNoDelay());
    
    // 订阅LQR路径点，触发参数配置热切换
    ros::Subscriber sub_lqr = nh.subscribe("/lqr_targetwayp", 1, lqrWaypointCallback);
    
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, LaserScan, LaserScan> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_16, sub_mid, sub_left, sub_right);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ROS_INFO("Lidar Filtering Node Started! Behavior-Driven Configuration Mode.");
    
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}