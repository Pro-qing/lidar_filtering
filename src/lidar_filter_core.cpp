#include "lidar_filtering/lidar_filter_core.hpp"

// 禁用PCL预编译头，以避免链接时可能出现的问题
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE 
#endif

// ==================== PCL滤波器库引入 ====================
#include <pcl/filters/voxel_grid.h>                 // 体素网格滤波
#include <pcl/filters/impl/voxel_grid.hpp>          // 体素网格实现
#include <pcl/filters/passthrough.h>               // 直通滤波（PassThrough）
#include <pcl/filters/impl/passthrough.hpp>        // 直通滤波实现
#include <pcl/filters/statistical_outlier_removal.h> // 统计离群点移除
#include <pcl/filters/radius_outlier_removal.h>     // 半径离群点移除
#include <pcl/filters/impl/radius_outlier_removal.hpp> // 半径滤波实现
#include <pcl/segmentation/sac_segmentation.h>      // 采样一致性分割（RANSAC）
#include <pcl/segmentation/impl/sac_segmentation.hpp> // 分割实现
#include <pcl/filters/extract_indices.h>           // 基于索引提取点云
#include <pcl/filters/impl/extract_indices.hpp>    // 索引提取实现
#include <pcl/sample_consensus/sac_model_plane.h>  // 平面模型
#include <pcl/sample_consensus/impl/sac_model_plane.hpp> // 平面模型实现

// ==================== 其他库引入 ====================
#include <pcl_conversions/pcl_conversions.h> // PCL与ROS消息转换
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // TF2几何消息转换
#include <tf2/utils.h>                       // TF2工具函数
#include <omp.h>                            // OpenMP多线程支持
#include <cmath>                            // 数学函数

/**
 * @brief 构造函数：初始化滤波核心类
 * @param nh ROS节点句柄
 * @param private_nh ROS私有节点句柄
 */
LidarFilterCore::LidarFilterCore(ros::NodeHandle &nh, ros::NodeHandle &private_nh) 
    : nh_(nh), private_nh_(private_nh), tf_listener_(tf_buffer_)
{
    // 获取最大线程数，并初始化OpenMP并行缓冲区
    int max_threads = omp_get_max_threads();
    omp_buffers_filter_.resize(max_threads);
    omp_buffers_vehicle_.resize(max_threads);

    // 初始化订阅者：关键点路径和控制指令
    key_points_sub_ = nh_.subscribe("/keypoint_path", 1, &LidarFilterCore::keyPointCallback, this);
    ctrol_sub_ = nh_.subscribe("/lqr_dire", 1, &LidarFilterCore::ctrolCallback, this);
    
    // 初始化发布者和定时器：用于可视化充电站区域
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("charge_marker", 1, true);
    charge_timer_ = nh_.createTimer(ros::Duration(0.2), &LidarFilterCore::chargeTimerCallback, this);
}

/**
 * @brief 更新滤波配置参数
 * @param config 新的配置结构体
 */
void LidarFilterCore::updateNativeConfig(const NativeFilterConfig& config) {
    std::lock_guard<std::mutex> lock(core_param_mutex_);
    config_ = config;
    charge_enble_ = config.charge_enble;
}

void LidarFilterCore::updateVehiclePolygon(const std::vector<geometry_msgs::Point>& polygon) {
    // 可以在这里做一些预处理，比如计算包围盒
}

/**
 * @brief 简化版 Scan 过滤器：只保留 [min_deg, max_deg] 范围内的点
 * @param scan 输入的激光扫描数据（会被修改）
 * @param min_deg 最小角度（度）
 * @param max_deg 最大角度（度）
 * @param max_dis 最大距离阈值
 * @param is_limit_mode 是否启用限制模式（未在此函数中详细使用）
 * @param limit_min_deg 限制模式最小角度
 * @param limit_max_deg 限制模式最大角度
 * 
 * 支持跨越 0 度的扇区（例如 350度 到 10度）。
 * 使用OpenMP并行处理以提高速度。
 */
void LidarFilterCore::filterScanMsg(sensor_msgs::LaserScan& scan, double min_deg, double max_deg, double max_dis, bool is_limit_mode, double limit_min_deg, double limit_max_deg) 
{
    int size = scan.ranges.size();
    double angle_min = scan.angle_min;
    double angle_inc = scan.angle_increment;

    // 并行遍历所有扫描点
    #pragma omp parallel for
    for (int i = 0; i < size; ++i) {
        float r = scan.ranges[i];
        // 跳过无效点（无穷大或NaN）
        if (std::isinf(r) || std::isnan(r)) continue;
        
        // 1. 基础距离截断：过滤过远或过近的点（过近可能是自身反射）
        if (r > max_dis || r < 0.05) { 
            scan.ranges[i] = std::numeric_limits<float>::infinity(); 
            continue; 
        }

        // 2. 角度扇区判定
        double angle_rad = angle_min + i * angle_inc;
        double angle_deg = angle_rad * 180.0 / M_PI;
        // 将角度归一化到 [0, 360) 范围
        while (angle_deg < 0) angle_deg += 360.0;
        while (angle_deg >= 360.0) angle_deg -= 360.0;

        bool in_sector = false;
        if (min_deg < max_deg) {
            // 普通区间：[min, max]
            in_sector = (angle_deg >= min_deg && angle_deg <= max_deg);
        } else {
            // 跨越0度区间：例如 [350, 10]
            in_sector = (angle_deg >= min_deg || angle_deg <= max_deg);
        }

        // 如果不在扇区内，将距离设为无穷大（即过滤掉）
        if (!in_sector) {
            scan.ranges[i] = std::numeric_limits<float>::infinity();
        }
    }
}

/**
 * @brief 双区间 Scan 过滤器：保留 [a, b] 或 [c, d] 范围内的点
 * @param scan 输入的激光扫描数据
 * @param a, b 第一区间角度范围
 * @param c, d 第二区间角度范围
 * @param max_dis 最大距离
 * @param is_limit_mode 是否启用限制模式
 * @param limit_min_deg, limit_max_deg 限制模式角度范围
 * @param limit_dis 限制模式距离阈值
 * 
 * 支持在特定角度范围内进行额外的距离限制（例如屏蔽特定角度的远距离物体）。
 */
void LidarFilterCore::filterScanMsgDualInterval(sensor_msgs::LaserScan& scan, 
                                                double a, double b, double c, double d,
                                                double max_dis, 
                                                bool is_limit_mode, 
                                                double limit_min_deg, double limit_max_deg,
                                                double limit_dis) 
{
    int size = scan.ranges.size();
    double angle_min = scan.angle_min;
    double angle_inc = scan.angle_increment;
    
    #pragma omp parallel for
    for (int i = 0; i < size; ++i) {
        // 跳过无效点
        if (std::isinf(scan.ranges[i]) || std::isnan(scan.ranges[i])) continue;
        // 距离过滤
        if (scan.ranges[i] > max_dis) { scan.ranges[i] = std::numeric_limits<float>::infinity(); continue; }

        // 计算当前点的角度
        double angle_rad = angle_min + i * angle_inc;
        double angle_deg = angle_rad * 180.0 / M_PI;
        while (angle_deg < 0) angle_deg += 360.0;
        while (angle_deg >= 360.0) angle_deg -= 360.0;

        // 判断是否在两个有效区间内
        bool keep = false;
        if (angle_deg >= a && angle_deg <= b) keep = true;
        if (angle_deg >= c && angle_deg <= d) keep = true;

        bool should_remove = !keep;

        // 如果启用了限制模式，且点在限制角度内，则检查距离是否超限
        if (!should_remove && is_limit_mode) {
            bool in_limit_angle = false;
            // 处理跨越0度的限制区间
            if (limit_min_deg > limit_max_deg) {
                if (angle_deg > limit_min_deg || angle_deg < limit_max_deg) in_limit_angle = true;
            } else {
                if (angle_deg > limit_min_deg && angle_deg < limit_max_deg) in_limit_angle = true;
            }

            // 如果在限制角度内且距离过大，则移除该点
            if (in_limit_angle && scan.ranges[i] > limit_dis) {
                should_remove = true;
            }
        }
        
        if (should_remove) scan.ranges[i] = std::numeric_limits<float>::infinity();
    }
}

/**
 * @brief 检查左右激光雷达数据的一致性
 * @param left 左侧扫描数据
 * @param right 右侧扫描数据
 * @param left_yaw 左侧雷达安装偏航角
 * @param right_yaw 右侧雷达安装偏航角
 * 
 * 逻辑：
 * 1. 遍历左侧雷达的有效点。
 * 2. 将该点转换到车体坐标系。
 * 3. 计算该点在右侧雷达坐标系下的角度和索引。
 * 4. 比较左右雷达在同一位置的测距值。
 * 5. 如果差异过大，则认为距离较短的那个是遮挡（如车身），保留距离较长的那个（真实障碍物）。
 */
void LidarFilterCore::checkScanConsistency(sensor_msgs::LaserScan& left, sensor_msgs::LaserScan& right, 
                                           double left_yaw, double right_yaw)
{
    // 获取配置参数
    NativeFilterConfig local_cfg;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_cfg = config_;
    }

    if (!local_cfg.consistency_enable) return;

    // 将角度转换为弧度
    double min_rad = local_cfg.consistency_min_angle * M_PI / 180.0;
    double max_rad = local_cfg.consistency_max_angle * M_PI / 180.0;
    double tol_dist = local_cfg.consistency_diff_dist;

    int left_size = left.ranges.size();
    double right_angle_min = right.angle_min;
    double right_angle_inc = right.angle_increment;
    int right_size = right.ranges.size();

    // 遍历左侧雷达数据
    for (int i = 0; i < left_size; ++i) {
        float r_left = left.ranges[i];
        if (std::isinf(r_left) || std::isnan(r_left)) continue;

        // 计算左侧点在车体坐标系下的角度
        double angle_local = left.angle_min + i * left.angle_increment;
        double angle_vehicle = angle_local + left_yaw;

        // 归一化角度到 [-PI, PI]
        while (angle_vehicle > M_PI) angle_vehicle -= 2.0 * M_PI;
        while (angle_vehicle < -M_PI) angle_vehicle += 2.0 * M_PI;

        // 如果不在检查范围内，跳过
        if (angle_vehicle < min_rad || angle_vehicle > max_rad) continue;

        // 计算该角度在右侧雷达局部坐标系下的角度
        double angle_right_local = angle_vehicle - right_yaw;
        while (angle_right_local > M_PI) angle_right_local -= 2.0 * M_PI;
        while (angle_right_local < -M_PI) angle_right_local += 2.0 * M_PI;

        // 找到对应的右侧雷达索引
        int j = std::round((angle_right_local - right_angle_min) / right_angle_inc);

        // 如果索引有效，比较距离
        if (j >= 0 && j < right_size) {
            float r_right = right.ranges[j];
            if (!std::isinf(r_right) && !std::isnan(r_right)) {
                // 如果距离差异超过阈值
                if (std::abs(r_left - r_right) > tol_dist) {
                    // 保留距离较大的（假设较短的被车身遮挡）
                    if (r_left < r_right) {
                        right.ranges[j] = std::numeric_limits<float>::infinity(); 
                    } else {
                        left.ranges[i] = std::numeric_limits<float>::infinity();  
                    }
                }
            }
        } 
    }
}

/**
 * @brief 核心点云滤波函数（手动实现版，优化性能）
 * @param in_cloud_ptr 输入点云
 * @param filter_cloud_ptr 输出滤波后的点云
 * @param update_history 是否更新历史帧（用于时间一致性）
 * 
 * 流程：
 * 1. 空间裁剪（半径、高度）。
 * 2. 地面过滤（基于高度阈值）。
 * 3. 简单的体素降采样（基于最近邻距离）。
 * 4. 充电站区域过滤。
 * 使用OpenMP进行并行加速。
 */
void LidarFilterCore::pointcloud_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                                        bool update_history)
{
    if (in_cloud_ptr->empty()) return;
    filter_cloud_ptr->clear();
    filter_cloud_ptr->reserve(in_cloud_ptr->size() * 0.5);

    // 获取本地配置副本
    NativeFilterConfig local_cfg;
    bool charge_active;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_cfg = config_;
        charge_active = charge_enble_ && charge_cache_.valid;
    }

    // 预计算半径平方，避免在循环中重复开方
    double r_sq_max = local_cfg.crop_radius * local_cfg.crop_radius;
    if (r_sq_max > 6400.0) r_sq_max = 6400.0; // 限制最大半径平方为80^2
    
    // 预计算体素滤波的最小距离平方
    double min_dist_sq = local_cfg.voxel_filter * local_cfg.voxel_filter;
    if (min_dist_sq < 0.0001) min_dist_sq = 0.0001;

    // 初始化OpenMP缓冲区
    int num_threads = omp_get_max_threads();
    for (int i = 0; i < num_threads; ++i) {
        omp_buffers_filter_[i].clear();
    }

    // 并行处理点云
    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        omp_buffers_filter_[tid].reserve(in_cloud_ptr->size() / num_threads);
        // 记录上一个点，用于简单的体素/稀疏化过滤
        double last_x = -999, last_y = -999, last_z = -999;
        
        #pragma omp for nowait
        for (size_t i = 0; i < in_cloud_ptr->size(); ++i) {
            const auto& pt = in_cloud_ptr->points[i];
            
            // 跳过无效点
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

            // 1. 充电站区域检查
            bool in_charge_zone = false;
            if (charge_active) {
                in_charge_zone = isPointInChargeArea(pt);
            }

            // 如果在充电站区域，保留该点（不进行后续过滤）并继续下一个点
            if (in_charge_zone) {
                omp_buffers_filter_[tid].push_back(pt);
                continue; 
            }

            // 2. 高度过滤
            if (pt.z < local_cfg.height_min || pt.z > local_cfg.height_max) continue;
            // 地面过滤
            if (local_cfg.filter_floor && pt.z < local_cfg.height_filt) continue;
            
            // 3. 空间裁剪（矩形区域：x偏移 + y半径）
            double dx = pt.x - local_cfg.crop_radius_x;
            if ((dx * dx + pt.y * pt.y) > r_sq_max) continue;
            
            // 4. 简单体素/稀疏化过滤（基于与上一个点的距离）
            double d_sq = (pt.x-last_x)*(pt.x-last_x) + (pt.y-last_y)*(pt.y-last_y) + (pt.z-last_z)*(pt.z-last_z);
            if (d_sq < min_dist_sq) continue;
            
            // 保留该点
            omp_buffers_filter_[tid].push_back(pt);
            last_x = pt.x; last_y = pt.y; last_z = pt.z;
        }
    }

    // 合并所有线程的结果
    for (int i = 0; i < num_threads; ++i) {
        filter_cloud_ptr->points.insert(filter_cloud_ptr->points.end(), 
                                        omp_buffers_filter_[i].begin(), 
                                        omp_buffers_filter_[i].end());
    }

    // 设置输出点云属性
    filter_cloud_ptr->width = filter_cloud_ptr->points.size();
    filter_cloud_ptr->height = 1;
    filter_cloud_ptr->is_dense = true;

    // 更新历史帧（如果需要）
    if (local_cfg.time_consistency_filter && update_history) {
        std::lock_guard<std::mutex> lock(history_mutex_);
        prev_non_ground_cloud_ = *filter_cloud_ptr;
    }
}

/**
 * @brief 核心点云滤波函数（PCL库实现版）
 * @param in_cloud_ptr 输入点云
 * @param filter_cloud_ptr 输出滤波后的点云
 * @param update_history 是否更新历史帧
 * 
 * 流程：
 * 1. 手动裁剪（半径、高度）。
 * 2. PCL VoxelGrid 降采样。
 * 3. RANSAC 地面分割与移除。
 * 4. PCL RadiusOutlierRemoval 离群点移除。
 */
void LidarFilterCore::pointcloud_filter_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                                            bool update_history)
{
    if (in_cloud_ptr->empty()) return;

    // 获取配置
    NativeFilterConfig local_cfg;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_cfg = config_;
    }

    // 静态指针，避免频繁分配内存
    static pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cropped(new pcl::PointCloud<pcl::PointXYZI>);
    static pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_voxel(new pcl::PointCloud<pcl::PointXYZI>);
    static pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_nonground(new pcl::PointCloud<pcl::PointXYZI>);
    
    tmp_cropped->clear(); tmp_voxel->clear(); tmp_nonground->clear();

    // Stage 1: 手动裁剪与高度过滤
    {
        double safe_radius = (local_cfg.crop_radius > 80.0) ? 80.0 : local_cfg.crop_radius;
        double safe_r_sq = safe_radius * safe_radius;
        for (const auto& pt : in_cloud_ptr->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            if (pt.z < local_cfg.height_min || pt.z > local_cfg.height_max) continue;
            double dx = pt.x - local_cfg.crop_radius_x;
            if ((dx * dx + pt.y * pt.y) > safe_r_sq) continue;
            tmp_cropped->push_back(pt);
        }
    }
    if (tmp_cropped->empty()) return;

    // Stage 2: 体素降采样
    {
        // 根据是否启用电梯模式选择不同的体素大小
        double leaf_size = enbleElevator_ ? local_cfg.voxel_filter_eleva : local_cfg.voxel_filter;
        if(leaf_size < 0.02) leaf_size = 0.02; // 最小体素限制
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(tmp_cropped);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.setMinimumPointsNumberPerVoxel(3); // 每个体素至少3个点才保留中心点
        vg.filter(*tmp_voxel);
    }

    // Stage 3: 地面移除
    bool ransac_success = false;
    if (local_cfg.filter_floor) {
        // 选取地面种子点（车前方的低点）
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_seeds(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& pt : tmp_voxel->points) {
            if (pt.x > 0.1 && pt.x < 8.0 && std::abs(pt.y) < 2.5) {
                if (pt.z < -1.75) { // 假设地面高度在 -1.75 以下
                    ground_seeds->push_back(pt);
                }
            }
        }

        // 如果种子点足够，使用RANSAC拟合平面
        if (ground_seeds->size() > 50) {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZI> seg;
            
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // 垂直平面模型（即水平面）
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); // 设置Z轴为法线方向
            seg.setEpsAngle(15.0 * M_PI / 180.0); // 允许15度的偏差
            seg.setDistanceThreshold(0.12);      // 距离阈值
            seg.setMaxIterations(150);
            seg.setInputCloud(ground_seeds);
            seg.segment(*inliers, *coefficients);

            // 如果拟合成功
            if (!inliers->indices.empty()) {
                float c = coefficients->values[2]; // Z方向系数
                float d = coefficients->values[3]; // 常数项
                float ground_h = -d / c; // 计算地面高度

                // 验证平面是否接近水平且高度合理
                if (std::abs(c) > 0.90 && ground_h < -1.75) { 
                    // 使用整个点云进行平面提取
                    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr 
                        model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(tmp_voxel));
                    
                    std::vector<int> final_inliers;
                    Eigen::Vector4f model_vec(coefficients->values[0], coefficients->values[1], c, d);
                    
                    // 选择距离平面小于0.20m的点作为地面点
                    model_p->selectWithinDistance(model_vec, 0.20, final_inliers);

                    pcl::PointIndices::Ptr final_indices(new pcl::PointIndices);
                    final_indices->indices = final_inliers;
                    
                    // 提取非地面点
                    pcl::ExtractIndices<pcl::PointXYZI> extract;
                    extract.setInputCloud(tmp_voxel);
                    extract.setIndices(final_indices);
                    extract.setNegative(true); // true表示提取索引之外的点（即非地面点）
                    extract.filter(*tmp_nonground);
                    ransac_success = true;
                }
            }
        }

        // 如果RANSAC失败，回退到简单的直通滤波
        if (!ransac_success) {
            pcl::PassThrough<pcl::PointXYZI> pass_floor;
            pass_floor.setInputCloud(tmp_voxel);
            pass_floor.setFilterFieldName("z");
            pass_floor.setFilterLimits(local_cfg.height_min, -1.80); // 保留高于-1.80的点
            pass_floor.setFilterLimitsNegative(true); // true表示保留范围外的点（即反向过滤）
            pass_floor.filter(*tmp_nonground);
        }
    }

    // Stage 4: 半径离群点移除
    if (local_cfg.radius_enble && !tmp_nonground->empty()) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
        outrem.setInputCloud(tmp_nonground);
        outrem.setRadiusSearch(local_cfg.radius_radius);
        outrem.setMinNeighborsInRadius(local_cfg.radius_min_neighbors);
        outrem.filter(*tmp_nonground);
    }

    *filter_cloud_ptr = *tmp_nonground;
}

/**
 * @brief 过滤充电站区域内的点云
 * @param cloud_ptr 输入输出点云（会被修改）
 * 
 * 如果启用了充电站过滤且缓存有效，则移除位于充电站区域内的点。
 */
void LidarFilterCore::filterChargingStation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
{
    if (!charge_enble_ || fliter_charge_ == 0 || cloud_ptr->empty()) return;

    // 更新充电站缓存（基于当前位姿）
    updateChargeCache(transPose_);
    
    // 原地过滤：保留不在充电站区域的点
    size_t write_index = 0;
    for (size_t i = 0; i < cloud_ptr->size(); ++i) {
        if (!isPointInChargeArea(cloud_ptr->points[i])) {
            cloud_ptr->points[write_index++] = cloud_ptr->points[i];
        }
    }
    
    // 调整点云大小
    cloud_ptr->points.resize(write_index);
    cloud_ptr->width = cloud_ptr->points.size();
    cloud_ptr->height = 1;
}

/**
 * @brief 更新充电站区域缓存
 * @param pose 充电站中心位姿（车体坐标系）
 * 
 * 计算充电站区域的局部坐标参数，加速后续的点云判断。
 */
void LidarFilterCore::updateChargeCache(const geometry_msgs::Pose& pose) {
    NativeFilterConfig local_cfg;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_cfg = config_;
    }

    std::lock_guard<std::mutex> lock(charge_cache_mutex_);
    charge_cache_.pose = pose;
    // 计算旋转矩阵的三角函数值，用于坐标变换
    double yaw = tf2::getYaw(pose.orientation);
    charge_cache_.cos_theta = std::cos(yaw);
    charge_cache_.sin_theta = std::sin(yaw);
    // 计算半长半宽
    charge_cache_.half_length = local_cfg.charge_length / 2.0;
    charge_cache_.half_width = local_cfg.charge_wide / 2.0;
    // 计算高度范围
    charge_cache_.z_min = pose.position.z; 
    charge_cache_.z_max = pose.position.z + local_cfg.charge_high;
    charge_cache_.valid = true;
}

/**
 * @brief 判断点是否在充电站区域内
 * @param pt 待检测的点
 * @return true: 在区域内, false: 在区域外
 * 
 * 将点转换到充电站局部坐标系，判断是否在矩形长方体内。
 */
bool LidarFilterCore::isPointInChargeArea(const pcl::PointXYZI& pt) {
    if (!charge_cache_.valid) return false;
    
    // 计算点相对于充电站中心的偏移
    double dx = pt.x - charge_cache_.pose.position.x;
    double dy = pt.y - charge_cache_.pose.position.y;
    
    // 旋转变换：转换到充电站局部坐标系
    double local_x = dx * charge_cache_.cos_theta + dy * charge_cache_.sin_theta;
    double local_y = -dx * charge_cache_.sin_theta + dy * charge_cache_.cos_theta;

    // 判断是否在矩形范围内（增加0.1m的余量）
    return (std::abs(local_x) < (charge_cache_.half_length + 0.1) &&
            std::abs(local_y) < (charge_cache_.half_width + 0.1) &&
            // 判断高度
            pt.z > (charge_cache_.z_min - 0.2) && pt.z < (charge_cache_.z_max + 0.2));
}

/**
 * @brief 根据中心位姿和长宽计算矩形顶点
 * @param pose_ 中心位姿
 * @param length 矩形长度
 * @param width 矩形宽度
 * @return 顶点列表（世界坐标系）
 */
std::vector<geometry_msgs::Point> LidarFilterCore::getRectangleVertices(
    const geometry_msgs::Pose& pose_, double length, double width) {
    // 局部坐标系下的四个顶点
    std::vector<Eigen::Vector3d> local_vertices = {
        { length/2,  width/2, 0}, {-length/2,  width/2, 0},
        {-length/2, -width/2, 0}, { length/2, -width/2, 0}
    };
    
    geometry_msgs::Point center = pose_.position;
    geometry_msgs::Quaternion quat = pose_.orientation;
    
    // 将四元数转换为旋转矩阵
    Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    
    std::vector<geometry_msgs::Point> vertices;
    vertices.reserve(4);
    
    // 变换到世界坐标系
    for (const auto& local_vertex : local_vertices) {
        Eigen::Vector3d world_vertex = rotation_matrix * local_vertex;
        geometry_msgs::Point p;
        p.x = world_vertex.x() + center.x; p.y = world_vertex.y() + center.y; p.z = world_vertex.z() + center.z;
        vertices.push_back(p);
    }
    return vertices;
}

/**
 * @brief 过滤自车车身轮廓内的点云
 * @param in_cloud_ptr 输入点云
 * @param out_cloud_ptr 输出点云（车身外的点）
 * @param vehicle_polygon 车辆轮廓多边形顶点
 * 
 * 使用射线法判断点是否在多边形内，并结合高度过滤。
 */
void LidarFilterCore::filterVehicleBody(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                                        const std::vector<geometry_msgs::Point>& vehicle_polygon)
{
    if (in_cloud_ptr->empty()) return;

    double local_v_height;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_v_height = config_.vehicle_height;
    }

    out_cloud_ptr->clear();
    out_cloud_ptr->reserve(in_cloud_ptr->size());

    // 初始化OpenMP缓冲区
    int num_threads = omp_get_max_threads();
    for (int i = 0; i < num_threads; ++i) {
        omp_buffers_vehicle_[i].clear();
    }

    // 并行处理
    #pragma omp parallel
    {
        int thread_id = omp_get_thread_num();
        omp_buffers_vehicle_[thread_id].reserve(in_cloud_ptr->size() / num_threads);
        
        #pragma omp for nowait
        for(size_t i=0; i<in_cloud_ptr->size(); ++i) {
            const auto& pt = in_cloud_ptr->points[i];
            geometry_msgs::Point p; p.x = pt.x; p.y = pt.y; p.z = pt.z;
            // 判断点是否在车身多边形内且高度低于车顶
            bool in_vehicle_zone = pointInPolygon(p, vehicle_polygon) && (pt.z <= local_v_height);
            // 如果不在车身区域，则保留
            if (!in_vehicle_zone) {
                omp_buffers_vehicle_[thread_id].push_back(pt);
            }
        }
    }
    
    // 合并结果
    for (int i = 0; i < num_threads; ++i) {
        out_cloud_ptr->points.insert(out_cloud_ptr->points.end(), 
                                     omp_buffers_vehicle_[i].begin(), 
                                     omp_buffers_vehicle_[i].end());
    }
    out_cloud_ptr->width = out_cloud_ptr->points.size();
    out_cloud_ptr->height = 1;
    out_cloud_ptr->is_dense = true;
}

/**
 * @brief 判断点是否在多边形内（射线法）
 * @param point 待检测的点
 * @param polyCorner 多边形顶点集合
 * @return true: 在内部, false: 在外部
 */
bool LidarFilterCore::pointInPolygon(const geometry_msgs::Point& point, const std::vector<geometry_msgs::Point>& polyCorner) {
    if (polyCorner.empty()) return false;
    
    // 1. 快速包围盒检测
    double min_x=1e9, max_x=-1e9, min_y=1e9, max_y=-1e9;
    for (const auto& p : polyCorner) { if(p.x<min_x) min_x=p.x; if(p.x>max_x) max_x=p.x; if(p.y<min_y) min_y=p.y; if(p.y>max_y) max_y=p.y; }
    if (point.x<min_x || point.x>max_x || point.y<min_y || point.y>max_y) return false; 
    
    // 2. 射线法判断
    bool result = false; int j = polyCorner.size() - 1;
    for (int i = 0; i < polyCorner.size(); j = i++) {
        // 如果点在边的Y范围内，且射线与边相交
        if ((polyCorner[i].y > point.y) != (polyCorner[j].y > point.y) &&
            (point.x < (polyCorner[j].x - polyCorner[i].x) * (point.y - polyCorner[i].y) / (polyCorner[j].y - polyCorner[i].y) + polyCorner[i].x)) result = !result;
    }
    return result;
}

/**
 * @brief 生成车辆模型的RViz可视化Marker
 * @param polyCorner 车辆轮廓顶点
 * @return 可视化Marker消息
 */
visualization_msgs::Marker LidarFilterCore::pubVehicleModel(const std::vector<geometry_msgs::Point>& polyCorner) {
    double local_v_height;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_v_height = config_.vehicle_height;
    }
    
    visualization_msgs::Marker marker; 
    marker.header.frame_id = "velodyne"; 
    marker.header.stamp = ros::Time::now();
    marker.ns = "vehicle_model"; 
    marker.id = 0; 
    marker.type = visualization_msgs::Marker::LINE_LIST; // 线条列表类型
    marker.action = visualization_msgs::Marker::ADD; 
    marker.scale.x = 0.05; // 线宽
    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0; // 青色
    marker.pose.orientation.w = 1.0;
    
    if (polyCorner.empty()) return marker;
    
    // 绘制车身线框（底部矩形、顶部矩形、连接立柱）
    size_t n = polyCorner.size(); 
    double h_min = -1.5; // 假设底部Z坐标
    double h_max = local_v_height; // 顶部Z坐标
    
    for (size_t i = 0; i < n; ++i) {
        size_t next = (i + 1) % n;
        geometry_msgs::Point p1 = polyCorner[i]; geometry_msgs::Point p2 = polyCorner[next];
        
        // 底部边
        geometry_msgs::Point b1; b1.x=p1.x; b1.y=p1.y; b1.z=h_min; 
        geometry_msgs::Point b2; b2.x=p2.x; b2.y=p2.y; b2.z=h_min;
        // 顶部边
        geometry_msgs::Point t1; t1.x=p1.x; t1.y=p1.y; t1.z=h_max; 
        geometry_msgs::Point t2; t2.x=p2.x; t2.y=p2.y; t2.z=h_max;
        
        // 添加线条：底边、顶边、立柱
        marker.points.push_back(b1); marker.points.push_back(b2);
        marker.points.push_back(t1); marker.points.push_back(t2);
        marker.points.push_back(b1); marker.points.push_back(t1);
    }
    return marker;     
}

/**
 * @brief 在RViz中显示矩形框
 * @param publisher Marker发布者
 * @param pose_ 矩形中心位姿
 * @param scale_x 长度
 * @param scale_y 宽度
 * @param scale_z 高度
 */
void LidarFilterCore::displayPolygonInRviz(ros::Publisher& publisher, const geometry_msgs::Pose& pose_, float scale_x, float scale_y, float scale_z) {
    visualization_msgs::Marker m; 
    m.header.frame_id = "map"; 
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.id = 0; 
    m.scale.x = 0.05; // 线宽
    m.color.a = 1.0; 
    m.color.r = 1.0;  // 红色

    // 如果尺寸无效，删除Marker
    if (scale_x <= 0.001 || scale_y <= 0.001) {
        m.action = visualization_msgs::Marker::DELETE;
    } else {
        m.action = visualization_msgs::Marker::ADD;
        // 获取底部顶点
        std::vector<geometry_msgs::Point> bottom_v = getRectangleVertices(pose_, scale_x, scale_y);
        if(!bottom_v.empty()) { 
            // 计算顶部顶点
            std::vector<geometry_msgs::Point> top_v = bottom_v;
            for (auto& p : top_v) {
                p.z += scale_z;
            }
            // 绘制线框（底边、顶边、立柱）
            for (size_t i = 0; i < 4; ++i) {
                size_t next = (i + 1) % 4;
                m.points.push_back(bottom_v[i]); m.points.push_back(bottom_v[next]);
                m.points.push_back(top_v[i]); m.points.push_back(top_v[next]);
                m.points.push_back(bottom_v[i]); m.points.push_back(top_v[i]);
            }
        }
    }
    visualization_msgs::MarkerArray ma; 
    ma.markers.push_back(m); 
    publisher.publish(ma);
}

/**
 * @brief 坐标变换：将Pose从一个坐标系变换到另一个
 * @param transform 变换矩阵
 * @param input_pose 输入位姿
 * @param output_pose 输出位姿
 * @return 是否变换成功
 */
bool LidarFilterCore::transformPose(const geometry_msgs::TransformStamped& transform, const geometry_msgs::Pose& input_pose, geometry_msgs::Pose& output_pose) {
    try { 
        geometry_msgs::PoseStamped i, o; 
        i.pose = input_pose; 
        tf2::doTransform(i, o, transform); 
        output_pose = o.pose; 
        return true; 
    } catch (...) { return false; }
}

/**
 * @brief 关键点回调：处理充电站位置
 * @param msg 关键点数组消息
 */
void LidarFilterCore::keyPointCallback(const autoware_msgs::KeyPointArrayConstPtr &msg) {
    double error, len, wide, high;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        error = config_.charge_error; len = config_.charge_length; wide = config_.charge_wide; high = config_.charge_high;
    }
    
    fliterpose_.clear(); 
    bool found = false;
    if(!msg->path.empty()) {
        for(const auto& point : msg->path) {
            for (const auto& type : point.types) {
                // 如果类型名为 "charges"，则认为是充电站
                if(type.type_name == "charges") {
                    geometry_msgs::Pose p = point.pose.pose;
                    // 根据误差修正位置（沿朝向方向偏移）
                    if(error != 0) { 
                        p.position.x += cos(tf2::getYaw(p.orientation)) * error; 
                        p.position.y += sin(tf2::getYaw(p.orientation)) * error; 
                    }
                    fliterpose_.push_back(p); 
                    // 在RViz中显示充电站区域
                    displayPolygonInRviz(marker_pub_, p, len, wide, high); 
                    found = true;
                }
            }
        }
    }
    // 如果未找到，删除RViz中的Marker
    if(!found) { geometry_msgs::Pose e; displayPolygonInRviz(marker_pub_, e, 0, 0, 0); }
}

/**
 * @brief 控制回调：根据指令切换模式
 * @param msg 控制指令
 * 
 * 指令含义：
 * 2: 充电中
 * <0: 充电完成/离开
 * 4: 电梯模式
 */
void LidarFilterCore::ctrolCallback(const std_msgs::Int8ConstPtr &msg) {
    if(charge_enble_ && msg->data == 2) fliter_charge_ = 1; // 充电中
    else if(charge_enble_ && msg->data < 0) fliter_charge_ = 2; // 充电结束
    else fliter_charge_ = 0; // 正常行驶
    
    if(fabs(msg->data) == 4) enbleElevator_ = true; else enbleElevator_ = false; // 电梯模式切换
}

/**
 * @brief 充电站定时器回调：更新充电站在车体坐标系下的位置
 * @param event 定时器事件
 */
void LidarFilterCore::chargeTimerCallback(const ros::TimerEvent& event) {
    // 如果处于充电状态且存在充电站位姿
    if(fliter_charge_ != 0 && !fliterpose_.empty()) {
        try {
            // 获取从map坐标系到velodyne坐标系的变换
            geometry_msgs::TransformStamped t = tf_buffer_.lookupTransform("velodyne", "map", ros::Time(0));
            // 选择目标位姿（如果只有一个则取第一个，否则根据状态选择）
            geometry_msgs::Pose tp; if(fliterpose_.size() == 1) tp = fliterpose_[0]; else tp = (fliter_charge_ == 2) ? fliterpose_[0] : fliterpose_[1];
            // 执行坐标变换并更新全局变量
            transformPose(t, tp, transPose_);
        } catch (...) {}
    }
}
