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

// 包含动态配置的头文件
#include <lidar_filtering/LidarFilteringConfig.h>

#include <vector>
#include <mutex>
#include <atomic>
#include <memory>

class LidarFilterCore {
public:
    LidarFilterCore(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~LidarFilterCore() = default;

    // 接收 RQT 下发的动态参数更新
    void updateDynamicConfig(const lidar_filtering::LidarFilteringConfig& config);

    // 通用点云滤波器
    void pointcloud_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                           bool update_history = true);

    // PCL标准版滤波器
    void pointcloud_filter_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                               bool update_history = true);

    // 独立的充电桩过滤函数 
    void filterChargingStation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    // 车身去除
    void filterVehicleBody(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                           const std::vector<geometry_msgs::Point>& vehicle_polygon);

    // 双雷达一致性校验
    void checkScanConsistency(sensor_msgs::LaserScan& left, sensor_msgs::LaserScan& right, 
                              double left_yaw, double right_yaw);

    visualization_msgs::Marker pubVehicleModel(const std::vector<geometry_msgs::Point>& polyCorner);
    bool pointInPolygon(const geometry_msgs::Point& point, const std::vector<geometry_msgs::Point>& polyCorner);

    static void filterScanMsg(sensor_msgs::LaserScan& scan, double min_angle_deg, double max_angle_deg, double max_dis, bool is_limit_mode = false, double limit_min_deg = 0, double limit_max_deg = 0);
    static void filterScanMsgDualInterval(sensor_msgs::LaserScan& scan, double a, double b, double c, double d, double max_dis, bool is_limit_mode = false, double limit_min_deg = 0, double limit_max_deg = 0, double limit_dis = 0.0);

    bool charge_enble_;
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

    // 参数访问保护锁
    std::mutex core_param_mutex_;

    // Params
    double crop_radius_, crop_radius_x_;
    bool filter_floor_;
    double height_max_, height_min_, height_filt_;
    
    double voxel_filter_, voxel_filter_eleva_;
    bool voxel_filter_auto_;
    bool enbleElevator_ = false;

    bool filter_transient_;
    int neighboring_points_;
    double stand_threshold_;
    bool time_consistency_filter_;

    bool radius_enble_;
    double radius_radius_;
    int radius_min_neighbors_;

    double charge_length_, charge_wide_, charge_high_, charge_error_;
    double vehicle_height_; 

    bool consistency_enable_;
    double consistency_min_angle_;
    double consistency_max_angle_;
    double consistency_diff_dist_;

    pcl::PointCloud<pcl::PointXYZI> prev_non_ground_cloud_;
    std::mutex history_mutex_;
    std::vector<geometry_msgs::Pose> fliterpose_;
    geometry_msgs::Pose transPose_;

    // 【新增】用于消除 OpenMP 线程内部动态分配的缓冲池
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
