#ifndef LIDAR_FILTER_CORE_HPP
#define LIDAR_FILTER_CORE_HPP

// ==================== 系统与ROS库引入 ====================
#include <ros/ros.h>                         // ROS核心库
#include <pcl/point_cloud.h>                 // PCL点云基础类型
#include <pcl/point_types.h>                 // PCL点类型定义
#include <sensor_msgs/LaserScan.h>           // 激光扫描消息
#include <visualization_msgs/Marker.h>       // RViz可视化标记
#include <visualization_msgs/MarkerArray.h> // 标记数组
#include <geometry_msgs/Point.h>            // 几何点消息
#include <geometry_msgs/Pose.h>              // 位姿消息
#include <autoware_msgs/KeyPointArray.h>     // Autoware关键点数组
#include <std_msgs/Int8.h>                   // 标准整型消息
#include <tf2_ros/buffer.h>                 // TF2缓冲区
#include <tf2_ros/transform_listener.h>      // TF2坐标变换监听器
#include <pcl/common/common.h>              // PCL通用算法
#include <pcl/filters/radius_outlier_removal.h> // PCL半径离群点移除滤波器

// ==================== C++标准库引入 ====================
#include <vector>    // 动态数组
#include <mutex>     // 互斥锁，用于线程安全
#include <atomic>    // 原子操作，用于线程安全的计数
#include <memory>    // 智能指针

/**
 * @brief 原生 C++ 配置结构体
 * 
 * 该结构体集中管理点云滤波的所有参数，通过YAML文件加载后更新到此结构体。
 * 包含裁剪、高度过滤、体素滤波、离群点移除、充电站过滤等多种参数。
 */
struct NativeFilterConfig {
    // --- 空间裁剪参数 ---
    double crop_radius = 8.0;      // 圆形裁剪半径（米）
    double crop_radius_x = 5.0;    // X轴方向裁剪半径（矩形裁剪）
    
    // --- 高度过滤参数 ---
    double height_max = 0.0;       // 最大高度阈值（高于此点被过滤）
    double height_min = -3.0;     // 最小高度阈值（低于此点被过滤，通常用于去除地面）
    double height_filt = -0.0;     // 额外的高度过滤阈值
    bool filter_floor = true;     // 是否启用地面过滤

    // --- 体素降采样参数 ---
    double voxel_filter = 0.1;           // 体素滤波器的体素大小（米）
    bool voxel_filter_auto = false;      // 是否自动调整体素大小
    double voxel_filter_eleva = 0.1;     // 垂直方向的体素大小

    // --- 瞬态障碍物/一致性过滤参数 ---
    bool filter_transient = true;        // 是否过滤瞬态障碍物（如飞鸟、树叶）
    int neighboring_points = 10;         // 瞬态过滤的邻域点数阈值
    double stand_threshold = 0.8;       // 静态物体判定阈值
    bool time_consistency_filter = true; // 是否启用时间一致性过滤（基于历史帧）

    // --- 半径离群点移除参数 ---
    bool radius_enble = true;            // 是否启用半径滤波
    double radius_radius = 0.15;         // 搜索半径（米）
    int radius_min_neighbors = 2;        // 最小邻居数，少于则视为离群点

    // --- 充电站区域过滤参数 ---
    bool charge_enble = true;            // 是否启用充电站区域过滤
    double charge_length = 1.2;          // 充电站区域长度（米）
    double charge_wide = 1.2;            // 充电站区域宽度（米）
    double charge_high = 1.2;            // 充电站区域高度（米）
    double charge_error = 1.2;           // 充电站定位误差容限（米）

    // --- 车辆参数 ---
    double vehicle_height = 1.0;         // 车辆高度（米），用于车身过滤

    // --- 扫描一致性检查参数 ---
    bool consistency_enable = false;     // 是否启用左右雷达一致性检查
    double consistency_min_angle = -30.0;// 检查的最小角度
    double consistency_max_angle = 30.0; // 检查的最大角度
    double consistency_diff_dist = 1.2;  // 距离差异阈值
};

/**
 * @brief 激光雷达滤波核心类
 * 
 * 负责处理多路激光雷达数据的融合、滤波、坐标变换及特定区域（如车身、充电站）的过滤。
 * 提供了多种滤波算法的组合，包括体素滤波、半径滤波、高度滤波等。
 */
class LidarFilterCore {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄（公共命名空间）
     * @param private_nh ROS节点句柄（私有命名空间，用于读取参数）
     */
    LidarFilterCore(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    
    /**
     * @brief 析构函数（默认）
     */
    ~LidarFilterCore() = default;

    /**
     * @brief 更新滤波配置参数
     * @param config 新的配置结构体
     */
    void updateNativeConfig(const NativeFilterConfig& config);

    /**
     * @brief 核心点云处理函数（通用模式）
     * @param in_cloud_ptr 输入点云
     * @param filter_cloud_ptr 输出滤波后的点云
     * @param update_history 是否更新历史帧数据（用于一致性过滤）
     */
    void pointcloud_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                           bool update_history = true);

    /**
     * @brief 核心点云处理函数（PCL特定模式）
     * @param in_cloud_ptr 输入点云
     * @param filter_cloud_ptr 输出滤波后的点云
     * @param update_history 是否更新历史帧数据
     */
    void pointcloud_filter_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                               bool update_history = true);

    /**
     * @brief 过滤充电站区域内的点云
     * @param cloud 待处理的点云（会被修改）
     */
    void filterChargingStation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    /**
     * @brief 过滤自车车身轮廓内的点云
     * @param in_cloud_ptr 输入点云
     * @param out_cloud_ptr 输出点云（车身外的点）
     * @param vehicle_polygon 车辆轮廓多边形顶点
     */
    void filterVehicleBody(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                           const std::vector<geometry_msgs::Point>& vehicle_polygon);

    /**
     * @brief 检查左右激光扫描数据的一致性
     * @param left 左侧扫描数据
     * @param right 右侧扫描数据
     * @param left_yaw 左侧雷达偏航角
     * @param right_yaw 右侧雷达偏航角
     */
    void checkScanConsistency(sensor_msgs::LaserScan& left, sensor_msgs::LaserScan& right, 
                              double left_yaw, double right_yaw);

    /**
     * @brief 生成车辆模型的RViz可视化Marker
     * @param polyCorner 车辆轮廓顶点
     * @return 可视化Marker消息
     */
    visualization_msgs::Marker pubVehicleModel(const std::vector<geometry_msgs::Point>& polyCorner);

    /**
     * @brief 判断点是否在多边形内（射线法）
     * @param point 待检测的点
     * @param polyCorner 多边形顶点集合
     * @return true: 在内部, false: 在外部
     */
    bool pointInPolygon(const geometry_msgs::Point& point, const std::vector<geometry_msgs::Point>& polyCorner);

    /**
     * @brief 静态函数：过滤LaserScan消息（单区间角度裁剪）
     * @param scan 输入的LaserScan（会被修改）
     * @param min_angle_deg 最小角度（度）
     * @param max_angle_deg 最大角度（度）
     * @param max_dis 最大距离
     * @param is_limit_mode 是否启用限制模式
     * @param limit_min_deg 限制最小角度
     * @param limit_max_deg 限制最大角度
     */
    static void filterScanMsg(sensor_msgs::LaserScan& scan, double min_angle_deg, double max_angle_deg, double max_dis, bool is_limit_mode = false, double limit_min_deg = 0, double limit_max_deg = 0);

    /**
     * @brief 静态函数：过滤LaserScan消息（双区间角度裁剪）
     * @param scan 输入的LaserScan
     * @param a, b 第一区间角度范围
     * @param c, d 第二区间角度范围
     * @param max_dis 最大距离
     * @param is_limit_mode 是否启用限制模式
     * @param limit_min_deg, limit_max_deg 限制角度范围
     * @param limit_dis 限制距离
     */
    static void filterScanMsgDualInterval(sensor_msgs::LaserScan& scan, double a, double b, double c, double d, double max_dis, bool is_limit_mode = false, double limit_min_deg = 0, double limit_max_deg = 0, double limit_dis = 0.0);

    // --- 公共成员变量 ---
    bool charge_enble_ = true;         // 充电站过滤开关
    std::atomic<int> fliter_charge_{0}; // 充电站过滤状态原子计数器

private:
    // ==================== ROS相关成员 ====================
    ros::NodeHandle nh_;               // 公共节点句柄
    ros::NodeHandle private_nh_;       // 私有节点句柄
    
    tf2_ros::Buffer tf_buffer_;        // TF2缓冲区
    tf2_ros::TransformListener tf_listener_; // TF2监听器

    ros::Subscriber key_points_sub_;    // 关键点订阅者
    ros::Subscriber ctrol_sub_;        // 控制命令订阅者
    ros::Publisher marker_pub_;        // Marker发布者
    ros::Timer charge_timer_;          // 充电站检测定时器

    // ==================== 配置与状态 ====================
    std::mutex core_param_mutex_;      // 保护核心参数的互斥锁
    NativeFilterConfig config_;        // 当前滤波配置

    bool enbleElevator_ = false;      // 是否启用电梯模式（特殊场景）

    // ==================== 历史数据与缓存 ====================
    pcl::PointCloud<pcl::PointXYZI> prev_non_ground_cloud_; // 上一帧非地面点云（用于时间一致性）
    std::mutex history_mutex_;         // 保护历史数据的互斥锁
    std::vector<geometry_msgs::Pose> fliterpose_;           // 历史位姿列表
    geometry_msgs::Pose transPose_;   // 变换后的位姿

    // ==================== 并行计算缓冲区 ====================
    std::vector<std::vector<pcl::PointXYZI>> omp_buffers_filter_; // OpenMP滤波缓冲区
    std::vector<std::vector<pcl::PointXYZI>> omp_buffers_vehicle_; // OpenMP车身过滤缓冲区

    // ==================== 私有回调函数 ====================
    void keyPointCallback(const autoware_msgs::KeyPointArrayConstPtr &msg); // 关键点回调
    void ctrolCallback(const std_msgs::Int8ConstPtr &msg);                 // 控制回调
    void chargeTimerCallback(const ros::TimerEvent& event);                // 定时器回调
    
    // ==================== 辅助函数 ====================
    /**
     * @brief 在RViz中显示多边形
     */
    void displayPolygonInRviz(ros::Publisher& publisher, const geometry_msgs::Pose& pose_, float scale_x, float scale_y, float scale_z);
    
    /**
     * @brief 应用坐标变换到Pose
     */
    bool transformPose(const geometry_msgs::TransformStamped& transform, const geometry_msgs::Pose& input_pose, geometry_msgs::Pose& output_pose);

    // ==================== 充电站相关结构体与函数 ====================
    /**
     * @brief 充电站缓存结构体
     * 存储充电站的位置、尺寸及顶点信息，加速点云过滤判断
     */
    struct ChargeCache {
        geometry_msgs::Pose pose;          // 充电站中心位姿
        double cos_theta;                  // 方向余弦（优化计算）
        double sin_theta;                  // 方向正弦
        double half_length;                // 半长
        double half_width;                 // 半宽
        double z_min;                      // 最小Z值
        double z_max;                      // 最大Z值
        std::vector<geometry_msgs::Point> vertices; // 矩形顶点
        bool valid = false;               // 数据是否有效
    } charge_cache_;
    
    std::mutex charge_cache_mutex_;       // 保护充电站缓存的互斥锁

    void updateChargeCache(const geometry_msgs::Pose& pose); // 更新充电站缓存
    bool isPointInChargeArea(const pcl::PointXYZI& point);  // 判断点是否在充电站区域
    std::vector<geometry_msgs::Point> getRectangleVertices(const geometry_msgs::Pose& pose_, double length, double width); // 获取矩形顶点
};

#endif
