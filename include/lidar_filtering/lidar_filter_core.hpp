#ifndef LIDAR_FILTER_CORE_HPP
#define LIDAR_FILTER_CORE_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <autoware_msgs/KeyPointArray.h>
#include <std_msgs/Int8.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/common/common.h> 
#include <pcl/filters/radius_outlier_removal.h>

#include <vector>
#include <mutex>
#include <atomic>
#include <memory>

// 原生 C++ 配置体
struct NativeFilterConfig {
    double crop_radius = 8.0;
    double crop_radius_x = 5.0;
    double height_max = 0.0;
    double height_min = -3.0;
    double height_filt = -0.0;
    bool filter_floor = true;
    
    double voxel_filter = 0.1;
    bool voxel_filter_auto = false;
    double voxel_filter_eleva = 0.1;
    
    bool filter_transient = true;
    int neighboring_points = 10;
    double stand_threshold = 0.8;
    bool time_consistency_filter = true;
    
    bool radius_enble = true;
    double radius_radius = 0.15;
    int radius_min_neighbors = 2;

    bool charge_enble = true;
    double charge_length = 1.2;
    double charge_wide = 1.2;
    double charge_high = 1.2;
    double charge_error = 1.2;

    double vehicle_height = 1.0;

    bool consistency_enable = false;
    double consistency_min_angle = -30.0;
    double consistency_max_angle = 30.0;
    double consistency_diff_dist = 1.2;
};

class LidarFilterCore {
public:
    LidarFilterCore(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~LidarFilterCore() = default;

    void updateNativeConfig(const NativeFilterConfig& config);

    void pointcloud_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                           bool update_history = true);

    void pointcloud_filter_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                               bool update_history = true);

    void filterChargingStation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    void filterVehicleBody(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                           const std::vector<geometry_msgs::Point>& vehicle_polygon);

    void checkScanConsistency(sensor_msgs::LaserScan& left, sensor_msgs::LaserScan& right, 
                              double left_yaw, double right_yaw);

    visualization_msgs::Marker pubVehicleModel(const std::vector<geometry_msgs::Point>& polyCorner);
    bool pointInPolygon(const geometry_msgs::Point& point, const std::vector<geometry_msgs::Point>& polyCorner);

    static void filterScanMsg(sensor_msgs::LaserScan& scan, double min_angle_deg, double max_angle_deg, double max_dis, bool is_limit_mode = false, double limit_min_deg = 0, double limit_max_deg = 0);
    static void filterScanMsgDualInterval(sensor_msgs::LaserScan& scan, double a, double b, double c, double d, double max_dis, bool is_limit_mode = false, double limit_min_deg = 0, double limit_max_deg = 0, double limit_dis = 0.0);

    bool charge_enble_ = true;
    std::atomic<int> fliter_charge_{0};

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber key_points_sub_;
    ros::Subscriber ctrol_sub_;
    ros::Publisher marker_pub_;
    ros::Timer charge_timer_;

    std::mutex core_param_mutex_;
    NativeFilterConfig config_;

    bool enbleElevator_ = false;

    pcl::PointCloud<pcl::PointXYZI> prev_non_ground_cloud_;
    std::mutex history_mutex_;
    std::vector<geometry_msgs::Pose> fliterpose_;
    geometry_msgs::Pose transPose_;

    std::vector<std::vector<pcl::PointXYZI>> omp_buffers_filter_;
    std::vector<std::vector<pcl::PointXYZI>> omp_buffers_vehicle_;

    void keyPointCallback(const autoware_msgs::KeyPointArrayConstPtr &msg);
    void ctrolCallback(const std_msgs::Int8ConstPtr &msg);
    void chargeTimerCallback(const ros::TimerEvent& event);
    void displayPolygonInRviz(ros::Publisher& publisher, const geometry_msgs::Pose& pose_, float scale_x, float scale_y, float scale_z);
    bool transformPose(const geometry_msgs::TransformStamped& transform, const geometry_msgs::Pose& input_pose, geometry_msgs::Pose& output_pose);

    struct ChargeCache {
        geometry_msgs::Pose pose;
        double cos_theta;
        double sin_theta;
        double half_length;
        double half_width;
        double z_min;
        double z_max;
        std::vector<geometry_msgs::Point> vertices;
        bool valid = false;
    } charge_cache_;
    
    std::mutex charge_cache_mutex_;

    void updateChargeCache(const geometry_msgs::Pose& pose);
    bool isPointInChargeArea(const pcl::PointXYZI& point);
    std::vector<geometry_msgs::Point> getRectangleVertices(const geometry_msgs::Pose& pose_, double length, double width);
};

#endif