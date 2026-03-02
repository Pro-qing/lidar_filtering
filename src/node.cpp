#include "lidar_filtering/lidar_filter_core.hpp"
#include <lidar_filtering/LidarFilteringConfig.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/Waypoint.h>
#include <visualization_msgs/MarkerArray.h>
#include <future>
#include <thread>
#include <chrono>

using namespace sensor_msgs;
using namespace message_filters;

std::shared_ptr<LidarFilterCore> filter_core_ptr_;
laser_geometry::LaserProjection projector_;

// 发布者
ros::Publisher pub_merged_filter_;
ros::Publisher pub_points_raw;
ros::Publisher pub_16_filter_, pub_mid_filter_, pub_left_filter_, pub_right_filter_;
ros::Publisher pub_16_calib_, pub_mid_calib_, pub_left_calib_, pub_right_calib_;
ros::Publisher pub_car_marker_, pub_car2_marker_, pub_debug_origins_;


// 新增：充电桩多边形发布者
ros::Publisher pub_charge_polygon_marker_;

std::string parent_frame_;
std::vector<geometry_msgs::Point> vehicleSize;
std::vector<geometry_msgs::Point> vehicleSize2;
autoware_can_msgs::CANInfo can_info_;
bool debug_mode_ = false;

// 全局变量：控制单线雷达是否参与融合
bool enable_single_lidar_fusion = true;

// 新增：存储充电桩多边形
struct ChargingStationPolygon {
    bool detected = false;
    std::vector<geometry_msgs::Point> polygon_points;  // 多边形顶点
    double height = 1.5;  // 充电桩高度（默认值）
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
    double x, y, z, yaw, pitch, roll;
    Eigen::Affine3f getMatrix() const {
        Eigen::Affine3f mat = Eigen::Affine3f::Identity();
        mat.translation() << x, y, z;
        mat.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
        mat.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
        mat.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
        return mat;
    }
};

struct FilterParams { int enable; double min_angle, max_angle, max_dis; };
CalibrationParams p_main, p_mid, p_left, p_right;
FilterParams f_main, f_mid, f_left, f_right;
double maxSpeed, maxLimitDis_speed;
double leftLimit_min, leftLimit_max, rightLimit_min, rightLimit_max;

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
    add_marker(p_main, 0, 1.0, 0.0, 0.0, "Main");  
    add_marker(p_mid,  1, 0.0, 1.0, 0.0, "Mid");   
    add_marker(p_left, 2, 0.0, 0.0, 1.0, "Left");  
    add_marker(p_right, 3, 1.0, 1.0, 0.0, "Right"); 
    pub_debug_origins_.publish(markers);
}

// 新增：发布充电桩多边形为Marker
void publishChargingStationPolygon(const std_msgs::Header& header) {
    if (!charging_station_polygon_.detected || 
        charging_station_polygon_.polygon_points.size() < 3) {
        return;
    }
    
    visualization_msgs::MarkerArray markers;
    
    // 1. 多边形底部（2D多边形）
    visualization_msgs::Marker polygon_marker;
    polygon_marker.header = header;
    polygon_marker.header.frame_id = parent_frame_;
    polygon_marker.ns = "charging_station_polygon";
    polygon_marker.id = 0;
    polygon_marker.type = visualization_msgs::Marker::LINE_STRIP;
    polygon_marker.action = visualization_msgs::Marker::ADD;
    polygon_marker.scale.x = 0.1;  // 线宽
    
    // 设置颜色（绿色表示充电桩）
    polygon_marker.color.r = 0.0;
    polygon_marker.color.g = 1.0;
    polygon_marker.color.b = 0.0;
    polygon_marker.color.a = 1.0;
    
    // 设置多边形的点
    polygon_marker.points = charging_station_polygon_.polygon_points;
    
    // 闭合多边形（添加第一个点到最后）
    if (!charging_station_polygon_.polygon_points.empty()) {
        polygon_marker.points.push_back(charging_station_polygon_.polygon_points[0]);
    }
    
    // 设置高度为0（地面）
    for (auto& point : polygon_marker.points) {
        point.z = 0;
    }
    
    markers.markers.push_back(polygon_marker);
    
    // 2. 多边形顶部（与底部相同但有一定高度）
    visualization_msgs::Marker polygon_top_marker = polygon_marker;
    polygon_top_marker.id = 1;
    for (auto& point : polygon_top_marker.points) {
        point.z = charging_station_polygon_.height;
    }
    markers.markers.push_back(polygon_top_marker);
    
    // 3. 连接底部和顶部的垂直线
    visualization_msgs::Marker vertical_lines_marker = polygon_marker;
    vertical_lines_marker.id = 2;
    vertical_lines_marker.type = visualization_msgs::Marker::LINE_LIST;  // 每条线需要两个点
    
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
    
    // 4. 多边形中心标记
    geometry_msgs::Point center_point;
    for (const auto& point : charging_station_polygon_.polygon_points) {
        center_point.x += point.x;
        center_point.y += point.y;
        center_point.z += point.z;
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
    center_marker.scale.x = 0.3;
    center_marker.scale.y = 0.3;
    center_marker.scale.z = 0.3;
    center_marker.color.r = 1.0;
    center_marker.color.g = 0.0;
    center_marker.color.b = 0.0;
    center_marker.color.a = 0.7;
    markers.markers.push_back(center_marker);
    
    // 5. 文字标记
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
    text_marker.scale.z = 0.5;  // 文字大小
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = "Charging Station";
    markers.markers.push_back(text_marker);
    
    // 设置所有marker的存活时间
    for (auto& marker : markers.markers) {
        marker.lifetime = ros::Duration(0.5);  // 0.5秒后自动消失
    }
    
    pub_charge_polygon_marker_.publish(markers);
    
    if (debug_mode_) {
        ROS_INFO_THROTTLE(1.0, "[Charging Station] Polygon published with %zu points", 
            charging_station_polygon_.polygon_points.size());
    }
}

void processScan(
    sensor_msgs::LaserScan scan_copy, 
    const CalibrationParams& calib, 
    const FilterParams& filter,
    bool is_limit_check, double limit_min, double limit_max,
    LidarBuffers& buf) 
{
    buf.calib->clear(); buf.filt->clear(); buf.raw_count = scan_copy.ranges.size();

    // 1. 标定数据生成 (使用已经校验过的 scan_copy)
    LidarFilterCore::filterScanMsg(scan_copy, 0, 0, 100.0, false, 0, 0);
    sensor_msgs::PointCloud2 cloud_msg;
    try { projector_.projectLaser(scan_copy, cloud_msg); } catch (...) {}
    if (cloud_msg.width > 0) {
        pcl::fromROSMsg(cloud_msg, *buf.calib);
        pcl::transformPointCloud(*buf.calib, *buf.calib, calib.getMatrix());
    }

    // 2. 过滤数据生成
    if (filter.enable) {
        sensor_msgs::LaserScan scan_strict = scan_copy;
        LidarFilterCore::filterScanMsg(scan_strict, filter.min_angle, filter.max_angle, filter.max_dis,
            is_limit_check && (can_info_.speed < maxSpeed) && (maxLimitDis_speed < filter.max_dis),
            limit_min, limit_max);
        
        sensor_msgs::PointCloud2 strict_msg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_strict(new pcl::PointCloud<pcl::PointXYZI>);
        try { projector_.projectLaser(scan_strict, strict_msg); } catch(...){}
        
        if (strict_msg.width > 0) {
            pcl::fromROSMsg(strict_msg, *cloud_strict);
            pcl::transformPointCloud(*cloud_strict, *cloud_strict, calib.getMatrix());
            pcl::PointCloud<pcl::PointXYZI>::Ptr env_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            filter_core_ptr_->pointcloud_filter(cloud_strict, env_cloud, false); 
            filter_core_ptr_->filterVehicleBody(env_cloud, buf.filt, vehicleSize);
        }
    }
}

void processCloud(
    const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
    const CalibrationParams& calib,
    const FilterParams& filter,
    LidarBuffers& buf,
    bool use_pcl = false) 
{
    buf.calib->clear(); buf.filt->clear(); buf.raw_count = cloud_msg->width * cloud_msg->height;
    pcl::fromROSMsg(*cloud_msg, *buf.calib);
    pcl::transformPointCloud(*buf.calib, *buf.calib, calib.getMatrix());

    if (filter.enable) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr env_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (use_pcl) filter_core_ptr_->pointcloud_filter_pcl(buf.calib, env_cloud, false);
        else filter_core_ptr_->pointcloud_filter(buf.calib, env_cloud, false);
        filter_core_ptr_->filterVehicleBody(env_cloud, buf.filt, vehicleSize);
    }
}

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg_16,
              const sensor_msgs::PointCloud2::ConstPtr &msg_mid,
              const sensor_msgs::LaserScan::ConstPtr &msg_left,
              const sensor_msgs::LaserScan::ConstPtr &msg_right) 
{
    auto start_time = std::chrono::high_resolution_clock::now();
    bool check_speed = (can_info_.speed < maxSpeed);

    // [关键] 1. 拷贝
    sensor_msgs::LaserScan scan_left_copy = *msg_left;
    sensor_msgs::LaserScan scan_right_copy = *msg_right;

    // [关键] 2. 一致性校验
    filter_core_ptr_->checkScanConsistency(scan_left_copy, scan_right_copy, p_left.yaw, p_right.yaw);

    // // [关键] 3. 并行处理
    // auto f1 = std::async(std::launch::async, processCloud, msg_16, p_main, f_main, std::ref(buf_main), false);
    // auto f2 = std::async(std::launch::async, processCloud, msg_mid, p_mid, f_mid, std::ref(buf_mid), true);
    // auto f3 = std::async(std::launch::async, processScan, scan_left_copy, p_left, f_left, check_speed, leftLimit_min, leftLimit_max, std::ref(buf_left));
    // auto f4 = std::async(std::launch::async, processScan, scan_right_copy, p_right, f_right, check_speed, rightLimit_min, rightLimit_max, std::ref(buf_right));

    // [关键] 3. 并行处理
    auto f1 = std::async(std::launch::async, processCloud, msg_16, p_main, f_main, std::ref(buf_main), false);
    auto f2 = std::async(std::launch::async, processCloud, msg_mid, p_mid, f_mid, std::ref(buf_mid), true); // Mid雷达使用PCL滤波
    auto f3 = std::async(std::launch::async, processScan, scan_left_copy, p_left, f_left, check_speed, leftLimit_min, leftLimit_max, std::ref(buf_left));
    auto f4 = std::async(std::launch::async, processScan, scan_right_copy, p_right, f_right, check_speed, rightLimit_min, rightLimit_max, std::ref(buf_right));

    f1.get(); f2.get(); f3.get(); f4.get();

    // [新增] 4. 专门针对 Mid 雷达进行充电桩处理
    // 注意：charge_enble_ 和 fliter_charge_ 是 LidarFilterCore 的成员变量
    if (filter_core_ptr_->charge_enble_ && filter_core_ptr_->fliter_charge_ != 0 && !buf_mid.filt->empty()) {
        // 4.1 执行充电桩过滤
        // 注意：这里直接修改 buf_mid.filt，将其中的充电桩点剔除
        filter_core_ptr_->filterChargingStation(buf_mid.filt);
    }



    // 1. Raw Merge
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

    // 2. Filter Merge
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZI>());
    
    size_t total_filt = buf_main.filt->size() + buf_mid.filt->size() + buf_left.filt->size() + buf_right.filt->size();
    
    if (pub_merged_filter_.getNumSubscribers() > 0) {
        
        merged_cloud->reserve(total_filt);
        *merged_cloud += *buf_main.filt; 
        *merged_cloud += *buf_mid.filt; // 此时 buf_mid.filt 已经剔除了充电桩点
        
        if (enable_single_lidar_fusion) {
            *merged_cloud += *buf_left.filt; 
            *merged_cloud += *buf_right.filt;
        }

        // 调用通用滤波 (降采样/历史更新)
        filter_core_ptr_->pointcloud_filter(merged_cloud, final_output, true);

        //  融合后进行充电桩过滤，并获取多边形
        charging_station_polygon_.reset();  // 重置多边形
        
        // 调用过滤函数，假设它能填充充电桩多边形
        // 这里需要根据您的实际实现来调用
        filter_core_ptr_->filterChargingStation(final_output);
        
        
        // 如果没有实际的充电桩多边形数据，这里提供一个多边形（矩形）
        if (debug_mode_ && charging_station_polygon_.polygon_points.empty()) {
            // 创建一个矩形多边形（充电桩）
            geometry_msgs::Point p1, p2, p3, p4;
            p1.x = 5.0; p1.y = -1.0; p1.z = 0.0;
            p2.x = 5.0; p2.y = 1.0; p2.z = 0.0;
            p3.x = 7.0; p3.y = 1.0; p3.z = 0.0;
            p4.x = 7.0; p4.y = -1.0; p4.z = 0.0;
            
            charging_station_polygon_.polygon_points.push_back(p1);
            charging_station_polygon_.polygon_points.push_back(p2);
            charging_station_polygon_.polygon_points.push_back(p3);
            charging_station_polygon_.polygon_points.push_back(p4);
            charging_station_polygon_.detected = true;
            charging_station_polygon_.height = 2.0;
        }

        sensor_msgs::PointCloud2 msg; 
        pcl::toROSMsg(*final_output, msg);
        
        msg.header = msg_16->header; 
        msg.header.frame_id = parent_frame_;
        pub_merged_filter_.publish(msg);
    }

    // 发布充电桩多边形Marker
    if (pub_charge_polygon_marker_.getNumSubscribers() > 0) {
        publishChargingStationPolygon(msg_16->header);
    }

    if (debug_mode_) {
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms = end_time - start_time;

        ROS_INFO_THROTTLE(1.0, "\n[Debug] Time: %.2f ms | Filtered: %lu -> %lu", ms.count(), total_filt, final_output->size());
        publishSensorMarkers(msg_16->header);
    }

    auto publish = [&](ros::Publisher& pub, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        if (pub.getNumSubscribers() > 0 && cloud) { 
            // 删除了 !cloud->empty() 的判断
            sensor_msgs::PointCloud2 msg; 
            pcl::toROSMsg(*cloud, msg);
            
            // 确保即使点云为空，Header 也是正确的
            msg.header = msg_16->header; 
            msg.header.frame_id = parent_frame_; 
            
            pub.publish(msg);
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
        visualization_msgs::MarkerArray ma; ma.markers.push_back(filter_core_ptr_->pubVehicleModel(vehicleSize));
        pub_car_marker_.publish(ma);
    }
    if (pub_car2_marker_.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray ma;
        ma.markers.push_back(filter_core_ptr_->pubVehicleModel(vehicleSize2));
        pub_car2_marker_.publish(ma);
    }
    if (pub_car2_marker_.getNumSubscribers() > 1) {
        visualization_msgs::MarkerArray ma;
        ma.markers.push_back(filter_core_ptr_->pubVehicleModel(vehicleSize2));
        pub_car2_marker_.publish(ma);
    }
}

void param_callback(lidar_filtering::LidarFilteringConfig &config, uint32_t level) {
    p_main = {config.main_x, config.main_y, config.main_z, config.main_yaw, config.main_pitch, config.main_roll};
    p_mid  = {config.top_x, config.top_y, config.top_z, config.top_yaw, config.top_pitch, config.top_roll};
    p_left = {config.left_x, config.left_y, config.left_z, config.left_yaw, config.left_pitch, config.left_roll};
    p_right= {config.right_x, config.right_y, config.right_z, config.right_yaw, config.right_pitch, config.right_roll};
    f_main = {config.main_filter_enable, config.main_min_angle, config.main_max_angle, 0};
    f_mid  = {config.top_filter_enable, config.top_min_angle, config.top_max_angle, 0};
    f_left = {config.left_filter_enable, config.left_min_angle, config.left_max_angle, config.left_max_dis};
    f_right= {config.right_filter_enable, config.right_min_angle, config.right_max_angle, config.right_max_dis};
    maxSpeed = config.maxSpeed; maxLimitDis_speed = config.maxLimitDis_speed;
    leftLimit_min = config.leftLimit_min_angle; leftLimit_max = config.leftLimit_max_angle;
    rightLimit_min = config.rightLimit_min_angle; rightLimit_max = config.rightLimit_max_angle;
}

void lqrWaypointCallback(const autoware_msgs::Waypoint::ConstPtr& msg) {
    if (msg) {
        if (msg->wpsattr.routeBehavior == 4) {
            if (enable_single_lidar_fusion) {
                enable_single_lidar_fusion = false;
                ROS_WARN("Single Lidar Fusion DISABLED (Direction: BACKWARD_LEFT).");
            }
        } else {
            if (!enable_single_lidar_fusion) {
                enable_single_lidar_fusion = true;
                ROS_INFO("Single Lidar Fusion ENABLED.");
            }
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_points");
    ros::NodeHandle nh; ros::NodeHandle p_nh("~");
    filter_core_ptr_ = std::make_shared<LidarFilterCore>(nh, p_nh);
    p_nh.param("debug_mode", debug_mode_, false);

    std::cout << "pcl_version:" << PCL_VERSION << std::endl;

    XmlRpc::XmlRpcValue carpoints, carpoints2;
    p_nh.getParam("rect", carpoints);
    if (carpoints.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for(int i=0; i<carpoints.size(); i++) { geometry_msgs::Point pt; pt.x=carpoints[i]["x"]; pt.y=carpoints[i]["y"]; vehicleSize.push_back(pt); }
    } else { geometry_msgs::Point p; p.x=1; p.y=1; vehicleSize.push_back(p); p.x=1; p.y=-1; vehicleSize.push_back(p); p.x=-1; p.y=-1; vehicleSize.push_back(p); p.x=-1; p.y=1; vehicleSize.push_back(p); }

    p_nh.param<std::string>("parent_frame", parent_frame_, "velodyne");
    dynamic_reconfigure::Server<lidar_filtering::LidarFilteringConfig> server;
    server.setCallback(boost::bind(&param_callback, _1, _2));

    pub_merged_filter_ = nh.advertise<PointCloud2>("points_filter", 10);
    pub_points_raw = nh.advertise<PointCloud2>("points_raw", 10);
    pub_16_filter_    = nh.advertise<PointCloud2>("points_16_filter", 10);
    pub_mid_filter_   = nh.advertise<PointCloud2>("points_mid_filter", 10);
    pub_left_filter_  = nh.advertise<PointCloud2>("scan_left_filter", 10);
    pub_right_filter_ = nh.advertise<PointCloud2>("scan_right_filter", 10);
    pub_16_calib_     = nh.advertise<PointCloud2>("points_16_calibration", 10);
    pub_mid_calib_    = nh.advertise<PointCloud2>("points_mid_calibration", 10);
    pub_left_calib_   = nh.advertise<PointCloud2>("points_left_calibration", 10);
    pub_right_calib_  = nh.advertise<PointCloud2>("points_right_calibration", 10);
    pub_car_marker_   = nh.advertise<visualization_msgs::MarkerArray>("car", 1, true);
    pub_car2_marker_  = nh.advertise<visualization_msgs::MarkerArray>("car2", 1, true);
    pub_debug_origins_ = nh.advertise<visualization_msgs::MarkerArray>("debug/sensor_origins", 1, true);
    
    // 充电桩多边形发布者
    pub_charge_polygon_marker_ = nh.advertise<visualization_msgs::MarkerArray>("charge", 10);

    ros::Subscriber sub_can = nh.subscribe("/can_info", 10, &canInfoCallback);
    message_filters::Subscriber<PointCloud2> sub_16(nh, "/points_16", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<PointCloud2> sub_mid(nh, "/points_mid", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<LaserScan> sub_left(nh, "/scan_left", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<LaserScan> sub_right(nh, "/scan_right", 1, ros::TransportHints().tcpNoDelay());
    
    ros::Subscriber sub_lqr = nh.subscribe("/lqr_targetwayp", 1, lqrWaypointCallback);
    
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, LaserScan, LaserScan> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(25), sub_16, sub_mid, sub_left, sub_right);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ROS_INFO("Lidar Filtering Node Started with Charging Station Polygon Publisher.");
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}
