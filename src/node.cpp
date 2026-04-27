#include "lidar_filtering/lidar_filter_core.hpp"
#include <yaml-cpp/yaml.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/Waypoint.h>
#include <visualization_msgs/MarkerArray.h>

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

#include <sys/inotify.h>
#include <poll.h>
#include <unistd.h>
#include <mutex>

using namespace sensor_msgs;
using namespace message_filters;

std::shared_ptr<LidarFilterCore> filter_core_ptr_;
laser_geometry::LaserProjection projector_;

ros::Publisher pub_merged_filter_;
ros::Publisher pub_points_raw;
ros::Publisher pub_16_filter_, pub_mid_filter_, pub_left_filter_, pub_right_filter_;
ros::Publisher pub_16_calib_, pub_mid_calib_, pub_left_calib_, pub_right_calib_;
ros::Publisher pub_car_marker_, pub_debug_origins_;
ros::Publisher pub_charge_polygon_marker_;

std::string parent_frame_;
std::vector<geometry_msgs::Point> vehicleSize;
autoware_can_msgs::CANInfo can_info_;
bool debug_mode_ = false;
bool enable_single_lidar_fusion = true;

NativeFilterConfig g_config;

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

struct LidarBuffers {
    pcl::PointCloud<pcl::PointXYZI>::Ptr calib;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filt;
    int raw_count = 0;
    LidarBuffers() {
        calib.reset(new pcl::PointCloud<pcl::PointXYZI>());
        filt.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
};

LidarBuffers buf_main, buf_mid, buf_left, buf_right;
pcl::PointCloud<pcl::PointXYZI>::Ptr g_merged_raw(new pcl::PointCloud<pcl::PointXYZI>());

struct CalibrationParams {
    double x = 0, y = 0, z = 0, yaw = 0, pitch = 0, roll = 0;
    Eigen::Affine3f getMatrix() const {
        Eigen::Affine3f mat = Eigen::Affine3f::Identity();
        mat.translation() << x, y, z;
        mat.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
        mat.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
        mat.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
        return mat;
    }
};

struct FilterParams { 
    int enable = 1; 
    double min_angle = 0, max_angle = 360, max_dis = 100; 
    double a = 0, b = 360, c = 0, d = 360; 
};

std::mutex calib_mutex;
CalibrationParams p_main, p_mid, p_left, p_right;
FilterParams f_main, f_mid, f_left, f_right;
double maxSpeed = 1.2, maxLimitDis_speed = 0.5;
double leftLimit_min = 0, leftLimit_max = 0, rightLimit_min = 0, rightLimit_max = 0;

// === YAML 解析函数：车辆配置 ===
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
        } else if (yaml["rect"]) { // 兼容老写法
            std::vector<geometry_msgs::Point> new_rect;
            for (const auto& pt : yaml["rect"]) {
                geometry_msgs::Point p;
                p.x = pt["x"].as<double>(); p.y = pt["y"].as<double>(); p.z = 0;
                new_rect.push_back(p);
            }
            if(!new_rect.empty()) vehicleSize = new_rect;
        }
        if(yaml["vehicle_height"]) g_config.vehicle_height = yaml["vehicle_height"].as<double>(1.0);
        
        if (filter_core_ptr_) filter_core_ptr_->updateNativeConfig(g_config);
        ROS_INFO("\033[1;32m[Config] Loaded Vehicle Size from %s!\033[0m", filepath.c_str());
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to parse Vehicle Size YAML: %s", e.what());
    }
}

// === YAML 解析函数：雷达标定（兼容带有下划线前缀的格式） ===
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

// === YAML 解析函数：滤波与区域配置 ===
void reloadFilterParamsYaml(const std::string& filepath) {
    try {
        YAML::Node yaml = YAML::LoadFile(filepath);

        if (yaml["regions"]) {
            auto parseRegion = [&](const std::string& name, FilterParams& fp) {
                if(yaml["regions"][name]) {
                    auto n = yaml["regions"][name];
                    fp.enable = n["enable"].as<int>(fp.enable);
                    fp.min_angle = n["min_angle"].as<double>(fp.min_angle);
                    fp.max_angle = n["max_angle"].as<double>(fp.max_angle);
                    fp.max_dis = n["max_dis"].as<double>(fp.max_dis);
                    fp.a = n["a"].as<double>(fp.a); fp.b = n["b"].as<double>(fp.b);
                    fp.c = n["c"].as<double>(fp.c); fp.d = n["d"].as<double>(fp.d);
                }
            };
            parseRegion("main", f_main); parseRegion("top", f_mid); 
            parseRegion("left", f_left); parseRegion("right", f_right);
        }

        if (yaml["speed_limits"]) {
            auto s = yaml["speed_limits"];
            maxSpeed = s["maxSpeed"].as<double>(1.2);
            maxLimitDis_speed = s["maxLimitDis_speed"].as<double>(0.5);
            leftLimit_min = s["leftLimit_min_angle"].as<double>(190.0);
            leftLimit_max = s["leftLimit_max_angle"].as<double>(90.0);
            rightLimit_min = s["rightLimit_min_angle"].as<double>(250.0);
            rightLimit_max = s["rightLimit_max_angle"].as<double>(170.0);
        }

        if (yaml["core"]) {
            auto cr = yaml["core"];
            g_config.crop_radius = cr["crop_radius"].as<double>(g_config.crop_radius);
            g_config.crop_radius_x = cr["crop_radius_x"].as<double>(g_config.crop_radius_x);
            g_config.height_max = cr["height_max"].as<double>(g_config.height_max);
            g_config.height_min = cr["height_min"].as<double>(g_config.height_min);
            g_config.height_filt = cr["height_filt"].as<double>(g_config.height_filt);
            g_config.filter_floor = cr["filter_floor"].as<bool>(g_config.filter_floor);
            g_config.voxel_filter = cr["voxel_filter"].as<double>(g_config.voxel_filter);
            g_config.voxel_filter_eleva = cr["voxel_filter_eleva"].as<double>(g_config.voxel_filter_eleva);
            g_config.filter_transient = cr["filter_transient"].as<bool>(g_config.filter_transient);
            g_config.radius_enble = cr["radius_enble"].as<bool>(g_config.radius_enble);
            g_config.radius_radius = cr["radius_radius"].as<double>(g_config.radius_radius);
            g_config.radius_min_neighbors = cr["radius_min_neighbors"].as<int>(g_config.radius_min_neighbors);
        }

        if (yaml["consistency"]) {
            auto cs = yaml["consistency"];
            g_config.consistency_enable = cs["enable"].as<bool>(g_config.consistency_enable);
            g_config.consistency_min_angle = cs["min_angle"].as<double>(g_config.consistency_min_angle);
            g_config.consistency_max_angle = cs["max_angle"].as<double>(g_config.consistency_max_angle);
            g_config.consistency_diff_dist = cs["diff_dist"].as<double>(g_config.consistency_diff_dist);
        }

        if (yaml["charge"]) {
            auto cg = yaml["charge"];
            g_config.charge_enble = cg["enble"].as<bool>(g_config.charge_enble);
            g_config.charge_length = cg["length"].as<double>(g_config.charge_length);
            g_config.charge_wide = cg["wide"].as<double>(g_config.charge_wide);
            g_config.charge_high = cg["high"].as<double>(g_config.charge_high);
            g_config.charge_error = cg["error"].as<double>(g_config.charge_error);
        }

        if (filter_core_ptr_) filter_core_ptr_->updateNativeConfig(g_config);
        ROS_INFO("\033[1;32m[Config] Loaded Filter Params from %s!\033[0m", filepath.c_str());
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to parse Filter Params YAML: %s", e.what());
    }
}

// === 热监听逻辑：支持监听一个目录下的多个文件 ===
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

// ---------------- 以下为原工程完整保留的回调与核心流程计算函数 ----------------

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

void processScan(sensor_msgs::LaserScan scan_copy, CalibrationParams calib, FilterParams filter, bool is_limit_check, double limit_min, double limit_max, LidarBuffers& buf) {
    buf.calib->clear(); buf.filt->clear(); buf.raw_count = scan_copy.ranges.size();
    LidarFilterCore::filterScanMsg(scan_copy, 0, 0, 100.0, false, 0, 0);
    sensor_msgs::PointCloud2 cloud_msg;
    try { projector_.projectLaser(scan_copy, cloud_msg); } catch (...) {}
    if (cloud_msg.width > 0) {
        pcl::fromROSMsg(cloud_msg, *buf.calib);
        pcl::transformPointCloud(*buf.calib, *buf.calib, calib.getMatrix());
    }
    if (filter.enable) {
        sensor_msgs::LaserScan scan_strict = scan_copy;
        bool limit_mode = is_limit_check && (can_info_.speed < maxSpeed) && (maxLimitDis_speed < filter.max_dis);
        LidarFilterCore::filterScanMsgDualInterval(scan_strict, filter.a, filter.b, filter.c, filter.d, filter.max_dis, limit_mode, limit_min, limit_max, maxLimitDis_speed); 
        sensor_msgs::PointCloud2 strict_msg;
        
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_strict(new pcl::PointCloud<pcl::PointXYZI>);
        static pcl::PointCloud<pcl::PointXYZI>::Ptr env_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_strict->clear(); 
        env_cloud->clear();

        try { projector_.projectLaser(scan_strict, strict_msg); } catch(...){}
        if (strict_msg.width > 0) {
            pcl::fromROSMsg(strict_msg, *cloud_strict);
            pcl::transformPointCloud(*cloud_strict, *cloud_strict, calib.getMatrix());
            filter_core_ptr_->pointcloud_filter(cloud_strict, env_cloud, false); 
            filter_core_ptr_->filterVehicleBody(env_cloud, buf.filt, vehicleSize);
        }
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

// 单线雷达专用：综合噪点过滤器
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

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg_16, const sensor_msgs::PointCloud2::ConstPtr &msg_mid, const sensor_msgs::LaserScan::ConstPtr &msg_left, const sensor_msgs::LaserScan::ConstPtr &msg_right) {
    auto start_time = std::chrono::high_resolution_clock::now();
    bool check_speed = (can_info_.speed < maxSpeed);
    sensor_msgs::LaserScan scan_left_copy = *msg_left;
    sensor_msgs::LaserScan scan_right_copy = *msg_right;

    CalibrationParams local_p_main, local_p_mid, local_p_left, local_p_right;
    {
        std::lock_guard<std::mutex> lock(calib_mutex);
        local_p_main = p_main;
        local_p_mid = p_mid;
        local_p_left = p_left;
        local_p_right = p_right;
    }

    filter_core_ptr_->checkScanConsistency(scan_left_copy, scan_right_copy, local_p_left.yaw, local_p_right.yaw);

    processCloud(msg_16, local_p_main, f_main, buf_main, false);
    processCloud(msg_mid, local_p_mid, f_mid, buf_mid, true);
    processScan(scan_left_copy, local_p_left, f_left, check_speed, leftLimit_min, leftLimit_max, buf_left);
    processScan(scan_right_copy, local_p_right, f_right, check_speed, rightLimit_min, rightLimit_max, buf_right);

    if (filter_core_ptr_->charge_enble_ && filter_core_ptr_->fliter_charge_ != 0 && !buf_mid.filt->empty()) {
        filter_core_ptr_->filterChargingStation(buf_mid.filt);
    }

    if (pub_points_raw.getNumSubscribers() > 0) {
        g_merged_raw->clear();
        g_merged_raw->reserve(buf_main.calib->size() + buf_mid.calib->size() + buf_left.calib->size() + buf_right.calib->size());
        *g_merged_raw += *buf_main.calib; *g_merged_raw += *buf_mid.calib;
        *g_merged_raw += *buf_left.calib; *g_merged_raw += *buf_right.calib;
        sensor_msgs::PointCloud2 raw_msg;
        pcl::toROSMsg(*g_merged_raw, raw_msg);
        raw_msg.header = msg_16->header; raw_msg.header.frame_id = parent_frame_;
        pub_points_raw.publish(raw_msg);
    }

    static SingleLineNoiseFilter left_noise_filter;
    static SingleLineNoiseFilter right_noise_filter;

    left_noise_filter.process(buf_left.filt);
    right_noise_filter.process(buf_right.filt);

    static pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    static pcl::PointCloud<pcl::PointXYZI>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZI>());
    
    merged_cloud->clear();
    final_output->clear();

    size_t total_filt = buf_main.filt->size() + buf_mid.filt->size() + buf_left.filt->size() + buf_right.filt->size();

    if (pub_merged_filter_.getNumSubscribers() > 0) {
        merged_cloud->reserve(total_filt);
        *merged_cloud += *buf_main.filt; 
        *merged_cloud += *buf_mid.filt; 
        
        if (enable_single_lidar_fusion) {
            *merged_cloud += *buf_left.filt; 
            *merged_cloud += *buf_right.filt;
        }

        filter_core_ptr_->pointcloud_filter(merged_cloud, final_output, true);
        
        charging_station_polygon_.reset();  
        filter_core_ptr_->filterChargingStation(final_output);
        
        if (debug_mode_ && charging_station_polygon_.polygon_points.empty()) {
            geometry_msgs::Point p1, p2, p3, p4;
            p1.x = 5.0; p1.y = -1.0; p1.z = 0.0; p2.x = 5.0; p2.y = 1.0; p2.z = 0.0;
            p3.x = 7.0; p3.y = 1.0; p3.z = 0.0; p4.x = 7.0; p4.y = -1.0; p4.z = 0.0;
            charging_station_polygon_.polygon_points.push_back(p1);
            charging_station_polygon_.polygon_points.push_back(p2);
            charging_station_polygon_.polygon_points.push_back(p3);
            charging_station_polygon_.polygon_points.push_back(p4);
            charging_station_polygon_.detected = true;
            charging_station_polygon_.height = 1.5;
        }

        sensor_msgs::PointCloud2 msg; 
        pcl::toROSMsg(*final_output, msg);
        msg.header = msg_16->header; 
        msg.header.frame_id = parent_frame_;
        pub_merged_filter_.publish(msg);
    }

    if (pub_charge_polygon_marker_.getNumSubscribers() > 0) publishChargingStationPolygon(msg_16->header);

    if (debug_mode_) {
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms = end_time - start_time;
        ROS_INFO_THROTTLE(1.0, "\n[Debug] Time: %.2f ms | Filtered: %lu -> %lu", ms.count(), total_filt, final_output->size());
        publishSensorMarkers(msg_16->header);
    }

    auto publish = [&](ros::Publisher& pub, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        if (pub.getNumSubscribers() > 0 && cloud) { 
            sensor_msgs::PointCloud2 msg; 
            pcl::toROSMsg(*cloud, msg);
            msg.header = msg_16->header; msg.header.frame_id = parent_frame_; pub.publish(msg);
        }
    };

    publish(pub_16_filter_, buf_main.filt);
    publish(pub_mid_filter_, buf_mid.filt);
    publish(pub_left_filter_, buf_left.filt);
    publish(pub_right_filter_, buf_right.filt);
    publish(pub_mid_calib_, buf_mid.calib);
    publish(pub_left_calib_, buf_left.calib);
    publish(pub_right_calib_, buf_right.calib);

    if (pub_car_marker_.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray ma; 
        ma.markers.push_back(filter_core_ptr_->pubVehicleModel(vehicleSize));
        pub_car_marker_.publish(ma);
    }
}

void lqrWaypointCallback(const autoware_msgs::Waypoint::ConstPtr& msg) {
    if (!msg) return; 
    bool has_behavior_4 = (std::find(msg->wpsattr.routeBehavior.begin(), msg->wpsattr.routeBehavior.end(), 4) != msg->wpsattr.routeBehavior.end());
    if (has_behavior_4) {
        if (enable_single_lidar_fusion) { enable_single_lidar_fusion = false; ROS_WARN("Single Lidar Fusion DISABLED"); }
    } else {
        if (!enable_single_lidar_fusion) { enable_single_lidar_fusion = true; ROS_INFO("Single Lidar Fusion ENABLED."); }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_points");
    ros::NodeHandle nh; ros::NodeHandle p_nh("~");
    
    p_nh.param<std::string>("parent_frame", parent_frame_, "velodyne");
    p_nh.param("debug_mode", debug_mode_, false);

    // 回收之前默认防止空的情况
    geometry_msgs::Point p_init; p_init.x=1; p_init.y=1; p_init.z=0; vehicleSize.push_back(p_init); 
    p_init.x=1; p_init.y=-1; vehicleSize.push_back(p_init); p_init.x=-1; p_init.y=-1; vehicleSize.push_back(p_init); p_init.x=-1; p_init.y=1; vehicleSize.push_back(p_init); 

    std::string path_vehicle, path_calib, path_filter;
    p_nh.param<std::string>("vehicle_size_file", path_vehicle, "");
    p_nh.param<std::string>("calibration_file", path_calib, "");
    p_nh.param<std::string>("filter_params_file", path_filter, "");

    filter_core_ptr_ = std::make_shared<LidarFilterCore>(nh, p_nh);

    if (!path_vehicle.empty()) reloadVehicleSizeYaml(path_vehicle);
    if (!path_calib.empty()) reloadCalibrationYaml(path_calib);
    if (!path_filter.empty()) reloadFilterParamsYaml(path_filter);

    // 开始目录热监听
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
    ros::Subscriber sub_lqr = nh.subscribe("/lqr_targetwayp", 1, lqrWaypointCallback);
    
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, LaserScan, LaserScan> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_16, sub_mid, sub_left, sub_right);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ROS_INFO("Lidar Filtering Node Started! Pure YAML configuration mode.");
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}