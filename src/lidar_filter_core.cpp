#include "lidar_filtering/lidar_filter_core.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
// RANSAC 地面分割相关头文件
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <omp.h>
#include <cmath>

LidarFilterCore::LidarFilterCore(ros::NodeHandle &nh, ros::NodeHandle &private_nh) 
    : nh_(nh), private_nh_(private_nh), tf_listener_(tf_buffer_)
{
    private_nh_.param("crop_radius", crop_radius_, 20.0);
    private_nh_.param("crop_radius_x", crop_radius_x_, 0.0);
    private_nh_.param("height_max", height_max_, 5.0);
    private_nh_.param("height_min", height_min_, -2.5);
    private_nh_.param("height_filt", height_filt_, -1.8);
    private_nh_.param("voxel_filter", voxel_filter_, 0.1);
    private_nh_.param("voxel_filter_auto", voxel_filter_auto_, false);
    private_nh_.param("voxel_filter_eleva", voxel_filter_eleva_, 0.15);
    private_nh_.param("filter_floor", filter_floor_, true);
    private_nh_.param("filter_transient", filter_transient_, false);
    private_nh_.param("neighboring_points", neighboring_points_, 30);
    private_nh_.param("stand_threshold", stand_threshold_, 1.0);
    private_nh_.param("radius_enble", radius_enble_, false);
    private_nh_.param("radius_radius", radius_radius_, 0.2);
    private_nh_.param("radius_min_neighbors", radius_min_neighbors_, 3);
    private_nh_.param("charge_enble", charge_enble_, true);
    private_nh_.param("charge_length", charge_length_, 1.0);
    private_nh_.param("charge_wide", charge_wide_, 1.0);
    private_nh_.param("charge_high", charge_high_, 1.0);
    private_nh_.param("charge_error", charge_error_, 1.5);
    private_nh_.param("vehicle_height", vehicle_height_, 1.0);
    private_nh_.param("time_consistency_filter", time_consistency_filter_, false);

    // [新增] 一致性参数
    private_nh_.param("consistency_enable", consistency_enable_, false);
    private_nh_.param("consistency_min_angle", consistency_min_angle_, -30.0);
    private_nh_.param("consistency_max_angle", consistency_max_angle_, 30.0);
    private_nh_.param("consistency_diff_dist", consistency_diff_dist_, 1.2);

    key_points_sub_ = nh_.subscribe("/keypoint_path", 1, &LidarFilterCore::keyPointCallback, this);
    ctrol_sub_ = nh_.subscribe("/lqr_dire", 1, &LidarFilterCore::ctrolCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("charge_marker", 1, true);
    charge_timer_ = nh_.createTimer(ros::Duration(0.2), &LidarFilterCore::chargeTimerCallback, this);
}

void LidarFilterCore::filterScanMsg(sensor_msgs::LaserScan& scan, double min_angle_deg, double max_angle_deg, double max_dis, bool is_limit_mode, double limit_min_deg, double limit_max_deg) 
{
    int size = scan.ranges.size();
    double angle_min = scan.angle_min;
    double angle_inc = scan.angle_increment;
    
    #pragma omp parallel for
    for (int i = 0; i < size; ++i) {
        if (std::isinf(scan.ranges[i]) || std::isnan(scan.ranges[i])) continue;
        if (scan.ranges[i] > max_dis) { scan.ranges[i] = std::numeric_limits<float>::infinity(); continue; }

        double angle_rad = angle_min + i * angle_inc;
        double angle_deg = angle_rad * 180.0 / M_PI;
        while (angle_deg < 0) angle_deg += 360.0;
        while (angle_deg >= 360.0) angle_deg -= 360.0;

        bool should_remove = false;
        if (min_angle_deg > max_angle_deg) {
            if (angle_deg > min_angle_deg || angle_deg < max_angle_deg) should_remove = true;
        } else {
            if (angle_deg > min_angle_deg && angle_deg < max_angle_deg) should_remove = true;
        }

        if (!should_remove && is_limit_mode) {
             if (limit_min_deg > limit_max_deg) {
                if (angle_deg > limit_min_deg || angle_deg < limit_max_deg) should_remove = true;
            } else {
                if (angle_deg > limit_min_deg && angle_deg < limit_max_deg) should_remove = true;
            }
        }
        if (should_remove) scan.ranges[i] = std::numeric_limits<float>::infinity();
    }
}

// [新增] 核心一致性校验函数
void LidarFilterCore::checkScanConsistency(sensor_msgs::LaserScan& left, sensor_msgs::LaserScan& right, 
                                           double left_yaw, double right_yaw)
{
    if (!consistency_enable_) return;

    double min_rad = consistency_min_angle_ * M_PI / 180.0;
    double max_rad = consistency_max_angle_ * M_PI / 180.0;
    double tol_dist = consistency_diff_dist_;

    // 以左雷达为基准遍历
    int left_size = left.ranges.size();
    
    // 预计算右雷达参数
    double right_angle_min = right.angle_min;
    double right_angle_inc = right.angle_increment;
    int right_size = right.ranges.size();

    // 为了防止并行修改冲突，这里使用单线程（Scan数据量小，单线程足够快）
    for (int i = 0; i < left_size; ++i) {
        float r_left = left.ranges[i];
        if (std::isinf(r_left) || std::isnan(r_left)) continue;

        // 1. 计算左雷达点在车体坐标系下的角度
        // Local -> Vehicle
        double angle_local = left.angle_min + i * left.angle_increment;
        double angle_vehicle = angle_local + left_yaw;

        // 归一化 -PI ~ PI
        while (angle_vehicle > M_PI) angle_vehicle -= 2.0 * M_PI;
        while (angle_vehicle < -M_PI) angle_vehicle += 2.0 * M_PI;

        // 2. 检查是否在校验范围内
        if (angle_vehicle < min_rad || angle_vehicle > max_rad) continue;

        // 3. 计算该车体角度对应右雷达的索引
        // Vehicle -> Right Local
        double angle_right_local = angle_vehicle - right_yaw;
        while (angle_right_local > M_PI) angle_right_local -= 2.0 * M_PI;
        while (angle_right_local < -M_PI) angle_right_local += 2.0 * M_PI;

        int j = std::round((angle_right_local - right_angle_min) / right_angle_inc);

        // 4. 对比
        if (j >= 0 && j < right_size) {
            float r_right = right.ranges[j];
            
            // 右雷达此处无效，或者距离差太大 -> 丢弃左雷达该点
            // (如果是为了互斥，这里只丢左雷达。如果想两边都丢，需要在外面再反向查一次，或者这里直接改)
            if (std::isinf(r_right) || std::isnan(r_right) || std::abs(r_left - r_right) > tol_dist) {
                left.ranges[i] = std::numeric_limits<float>::infinity();
                // 同时也把右雷达该点置无效 (严格一致性)
                right.ranges[j] = std::numeric_limits<float>::infinity(); 
            }
        } else {
            // 右雷达盲区 -> 为了安全，丢弃左雷达
            left.ranges[i] = std::numeric_limits<float>::infinity();
        }
    }
}

void LidarFilterCore::pointcloud_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                                        bool update_history)
{
    if (in_cloud_ptr->empty()) return;
    filter_cloud_ptr->clear();
    filter_cloud_ptr->reserve(in_cloud_ptr->size() * 0.5);

    // 预读参数，减少循环内开销
    double r_sq_max = crop_radius_ * crop_radius_;
    if (r_sq_max > 6400.0) r_sq_max = 6400.0; 
    double z_min = height_min_; 
    double z_max = height_max_;
    bool do_floor = filter_floor_; 
    double z_floor = height_filt_;
    
    double min_dist_sq = voxel_filter_ * voxel_filter_;
    if (min_dist_sq < 0.0001) min_dist_sq = 0.0001;
    
    // 充电桩使能标志
    bool charge_active = charge_enble_ && charge_cache_.valid;

    int num_threads = omp_get_max_threads();
    std::vector<std::vector<pcl::PointXYZI>> thread_buffers(num_threads);

    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        thread_buffers[tid].reserve(in_cloud_ptr->size() / num_threads);
        double last_x = -999, last_y = -999, last_z = -999;
        
        #pragma omp for nowait
        for (size_t i = 0; i < in_cloud_ptr->size(); ++i) {
            const auto& pt = in_cloud_ptr->points[i];
            
            // 基础合法性检查
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

            // --- [核心优化：充电桩保护逻辑] ---
            bool in_charge_zone = false;
            if (charge_active) {
                in_charge_zone = isPointInChargeArea(pt);
            }

            if (in_charge_zone) {
                // 如果是充电桩区域的点，强制保留，不进行高度和地面过滤
                thread_buffers[tid].push_back(pt);
                continue; 
            }
            // --------------------------------

            // 普通区域过滤逻辑
            if (pt.z < z_min || pt.z > z_max) continue;
            if (do_floor && pt.z < z_floor) continue;
            
            double dx = pt.x - crop_radius_x_;
            if ((dx * dx + pt.y * pt.y) > r_sq_max) continue;
            
            // 简单的体素采样 (Last Point Distance Filter)
            double d_sq = (pt.x-last_x)*(pt.x-last_x) + (pt.y-last_y)*(pt.y-last_y) + (pt.z-last_z)*(pt.z-last_z);
            if (d_sq < min_dist_sq) continue;
            
            thread_buffers[tid].push_back(pt);
            last_x = pt.x; last_y = pt.y; last_z = pt.z;
        }
    }

    // 合并多线程结果
    for (const auto& buf : thread_buffers) {
        filter_cloud_ptr->points.insert(filter_cloud_ptr->points.end(), buf.begin(), buf.end());
    }

    filter_cloud_ptr->width = filter_cloud_ptr->points.size();
    filter_cloud_ptr->height = 1;
    filter_cloud_ptr->is_dense = true;

    if (time_consistency_filter_ && update_history) {
        std::lock_guard<std::mutex> lock(history_mutex_);
        prev_non_ground_cloud_ = *filter_cloud_ptr;
    }
}

void LidarFilterCore::pointcloud_filter_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                                            bool update_history)
{
    if (in_cloud_ptr->empty()) return;

    static thread_local pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cropped(new pcl::PointCloud<pcl::PointXYZI>);
    static thread_local pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_voxel(new pcl::PointCloud<pcl::PointXYZI>);
    static thread_local pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_nonground(new pcl::PointCloud<pcl::PointXYZI>);
    
    tmp_cropped->clear(); tmp_voxel->clear(); tmp_nonground->clear();

    // =========================================================
    // Stage 1: 预裁剪 (保留所有雷达数据进入 Voxel)
    // =========================================================
    {
        double safe_radius = (crop_radius_ > 80.0) ? 80.0 : crop_radius_;
        double safe_r_sq = safe_radius * safe_radius;
        for (const auto& pt : in_cloud_ptr->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            if (pt.z < height_min_ || pt.z > height_max_) continue;
            double dx = pt.x - crop_radius_x_;
            if ((dx * dx + pt.y * pt.y) > safe_r_sq) continue;
            tmp_cropped->push_back(pt);
        }
    }
    if (tmp_cropped->empty()) return;

    // =========================================================
    // Stage 2: VoxelGrid 降采样
    // =========================================================
    {
        double leaf_size = enbleElevator_ ? voxel_filter_eleva_ : voxel_filter_;
        if(leaf_size < 0.02) leaf_size = 0.02; 
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(tmp_cropped);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.setMinimumPointsNumberPerVoxel(3); 
        vg.filter(*tmp_voxel);
    }

    // =========================================================
    // Stage 3: 地面去除 (Top引导 + 鲁棒提取)
    // =========================================================
    bool ransac_success = false;
    if (filter_floor_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_seeds(new pcl::PointCloud<pcl::PointXYZI>);
        
        // 3.1 扩大种子搜索范围，不再死磕 -1.75
        // 只要是低高度区域且在车前的点，都给 RANSAC 一个机会
        for (const auto& pt : tmp_voxel->points) {
            if (pt.x > 0.1 && pt.x < 8.0 && std::abs(pt.y) < 2.5) {
                // 根据你的单线雷达 z=-1.8，我们将高度阈值稍稍上移到 -1.7
                // 确保即使在颠簸或坡道，也能抓到足够的地面点
                if (pt.z < -1.75) { 
                    ground_seeds->push_back(pt);
                }
            }
        }

        if (ground_seeds->size() > 50) {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZI> seg;
            
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); 
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); 
            seg.setEpsAngle(15.0 * M_PI / 180.0); // 增加到20度，确保上下坡模型不丢失
            seg.setDistanceThreshold(0.12);      // 适当增加拟合厚度，提高鲁棒性
            seg.setMaxIterations(150);
            seg.setInputCloud(ground_seeds);
            seg.segment(*inliers, *coefficients);

            if (!inliers->indices.empty()) {
                float c = coefficients->values[2];
                float d = coefficients->values[3];
                float ground_h = -d / c;

                // 3.2 验证：只要平面高度在合理范围内（比如底盘以下）
                if (std::abs(c) > 0.90 && ground_h < -1.75) { 
                    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr 
                        model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(tmp_voxel));
                    
                    std::vector<int> final_inliers;
                    Eigen::Vector4f model_vec(coefficients->values[0], coefficients->values[1], c, d);
                    
                    // 3.3 关键：增加剔除半径到 0.20m (对齐注释版代码)
                    // 解决单线雷达扫地形成的“宽带状”点云
                    model_p->selectWithinDistance(model_vec, 0.20, final_inliers);

                    pcl::PointIndices::Ptr final_indices(new pcl::PointIndices);
                    final_indices->indices = final_inliers;
                    
                    pcl::ExtractIndices<pcl::PointXYZI> extract;
                    extract.setInputCloud(tmp_voxel);
                    extract.setIndices(final_indices);
                    extract.setNegative(true); 
                    extract.filter(*tmp_nonground);
                    ransac_success = true;
                }
            }
        }

        // 3.4 增强保底：如果 RANSAC 失败，使用更激进的高度截断
        if (!ransac_success) {
            pcl::PassThrough<pcl::PointXYZI> pass_floor;
            pass_floor.setInputCloud(tmp_voxel);
            pass_floor.setFilterFieldName("z");
            // 如果 RANSAC 挂了，说明当前场景非常复杂，直接切掉 -1.75 以下的所有点
            pass_floor.setFilterLimits(height_min_, -1.75); 
            pass_floor.setFilterLimitsNegative(true); 
            pass_floor.filter(*tmp_nonground);
        }
    }


    // // =========================================================
    // // Stage 3: 地面去除 (Top雷达拟合模型 -> 单线雷达过滤)
    // // =========================================================
    // bool ransac_success = false;
    // if (filter_floor_) {
    //     // 3.1 提取 Top 雷达的特征点云用于拟合地面
    //     // 根据你的 TF，Top 雷达在 z = -0.23，单线在 z = -1.8
    //     // 候选点只选 Top 雷达覆盖且靠近地面的区域
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr ground_seeds(new pcl::PointCloud<pcl::PointXYZI>);
    //     for (const auto& pt : tmp_voxel->points) {
    //         // 关键：只选取 Top 雷达照射范围内的近场地面作为种子
    //         // 假设地面在 Z = -2.0m 左右（根据单线安装高度推算）
    //         if (pt.x > 0.3 && pt.x < 6.0 && std::abs(pt.y) < 1.0 && pt.z < -1.75) {
    //             ground_seeds->push_back(pt);
    //         }
    //     }

    //     if (ground_seeds->size() > 50) {
    //         pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //         pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //         pcl::SACSegmentation<pcl::PointXYZI> seg;
            
    //         seg.setOptimizeCoefficients(true);
    //         seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); 
    //         seg.setMethodType(pcl::SAC_RANSAC);
    //         seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); // 限制地面法向量
    //         seg.setEpsAngle(15.0 * M_PI / 180.0);         // 严控角度，
    //         seg.setDistanceThreshold(0.06);              // 补盲雷达阈值收紧
    //         seg.setMaxIterations(100);
    //         seg.setInputCloud(ground_seeds);
    //         seg.segment(*inliers, *coefficients);

    //         if (!inliers->indices.empty()) {
    //             float c = coefficients->values[2];
    //             float d = coefficients->values[3];
    //             float ground_h = -d / c;

    //             // 3.2 验证拟合出的平面是否真的是脚下的地面
    //             if (std::abs(c) > 0.96 && ground_h < -1.68) { 
    //                 // 3.3 应用模型到“全集”进行剔除（这样单线扫到地面的点也会被包含在 model 距离内）
    //                 pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr 
    //                     model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(tmp_voxel));
                    
    //                 std::vector<int> final_inliers;
    //                 Eigen::Vector4f model_vec(coefficients->values[0], coefficients->values[1], c, d);
                    
    //                 // 单线雷达扫地形成的弧线会被这个模型精准覆盖
    //                 // 阈值 0.10m 足够涵盖单线的射线抖动
    //                 model_p->selectWithinDistance(model_vec, 0.10, final_inliers);

    //                 pcl::PointIndices::Ptr final_indices(new pcl::PointIndices);
    //                 final_indices->indices = final_inliers;
                    
    //                 pcl::ExtractIndices<pcl::PointXYZI> extract;
    //                 extract.setInputCloud(tmp_voxel);
    //                 extract.setIndices(final_indices);
    //                 extract.setNegative(true); // 剔除地面
    //                 extract.filter(*tmp_nonground);
    //                 ransac_success = true;
    //             }
    //         }
    //     }

    //     if (!ransac_success) {
    //         // 保底逻辑：硬截断
    //         pcl::PassThrough<pcl::PointXYZI> pass_floor;
    //         pass_floor.setInputCloud(tmp_voxel);
    //         pass_floor.setFilterFieldName("z");
    //         pass_floor.setFilterLimits(height_min_, -1.85); // 假设地面至少在-1.85以下
    //         pass_floor.setFilterLimitsNegative(true); 
    //         pass_floor.filter(*tmp_nonground);
    //     }
    // } else {
    //     *tmp_nonground = *tmp_voxel;
    // }

    // =========================================================
    // Stage 4: 离群点去噪 (ROR)
    // =========================================================
    if (radius_enble_ && !tmp_nonground->empty()) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
        outrem.setInputCloud(tmp_nonground);
        outrem.setRadiusSearch(radius_radius_);
        outrem.setMinNeighborsInRadius(radius_min_neighbors_);
        outrem.filter(*tmp_nonground);
    }

    *filter_cloud_ptr = *tmp_nonground;
}

// void LidarFilterCore::pointcloud_filter_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
//                                             pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
//                                             bool update_history)
// {
//      if (in_cloud_ptr->empty()) return;

//     static thread_local pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cropped(new pcl::PointCloud<pcl::PointXYZI>);
//     static thread_local pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_voxel(new pcl::PointCloud<pcl::PointXYZI>);
//     static thread_local pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_nonground(new pcl::PointCloud<pcl::PointXYZI>);
    
//     tmp_cropped->clear(); tmp_voxel->clear(); tmp_nonground->clear();

//     // Stage 1: 预裁剪
//     {
//         double safe_radius = (crop_radius_ > 80.0) ? 80.0 : crop_radius_;
//         double safe_r_sq = safe_radius * safe_radius;
//         double z_min = height_min_; double z_max = height_max_;
//         for (const auto& pt : in_cloud_ptr->points) {
//             if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
//             if (pt.z < z_min || pt.z > z_max) continue;
//             double dx = pt.x - crop_radius_x_;
//             if ((dx * dx + pt.y * pt.y) > safe_r_sq) continue;
//             tmp_cropped->push_back(pt);
//         }
//     }
//     if (tmp_cropped->empty()) return;

//     // Stage 2: VoxelGrid (为了解决粘连，建议在launch中把 voxel_filter 调小)
//     {
//         // 这里我们优先使用 voxel_filter_ (通常较小，如 0.03)，而不是 eleva (0.15)
//         // 除非特殊模式开启
//         double leaf_size = enbleElevator_ ? voxel_filter_eleva_ : voxel_filter_;
//         // 保护：不要太小导致溢出，但也不要太大导致粘连
//         if(leaf_size < 0.02) leaf_size = 0.02; 
        
//         pcl::VoxelGrid<pcl::PointXYZI> vg;
//         vg.setInputCloud(tmp_cropped);
//         vg.setLeafSize(leaf_size, leaf_size, leaf_size);
//         // 稍微提高一点门槛，去除极稀疏的噪点
//         vg.setMinimumPointsNumberPerVoxel(3); 
//         vg.filter(*tmp_voxel);
//     }

//     // Stage 3: 地面去除 (增加识别地面)
//     {
//         if (filter_floor_) {
//             // 尝试使用 RANSAC 识别平面 
//             pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//             pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//             pcl::SACSegmentation<pcl::PointXYZI> seg;
            
//             seg.setOptimizeCoefficients(true);
//             seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // 约束为近似水平面
//             seg.setMethodType(pcl::SAC_RANSAC);
//             seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); // Z轴
//             seg.setEpsAngle(15.0 * M_PI / 180.0); // 允许 15 度倾斜
//             seg.setDistanceThreshold(0.20); // 平面厚度阈值 0.2m
//             seg.setMaxIterations(100);
            
//             // 为了提高 RANSAC 速度和稳定性，先用高度做一个粗筛选
//             // 只在 probable_ground 范围内找平面
//             pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_candidate(new pcl::PointCloud<pcl::PointXYZI>);
//             pcl::PassThrough<pcl::PointXYZI> pass_pre;
//             pass_pre.setInputCloud(tmp_voxel);
//             pass_pre.setFilterFieldName("z");
//             pass_pre.setFilterLimits(height_min_, -1.75); // 假设地面不会超过 0.2m 高
//             pass_pre.filter(*cloud_candidate);

//             bool ransac_success = false;
//             // 1. 仅在候选点（低高度区域）中提取平面参数
//             if (cloud_candidate->size() > 50) {
//                 seg.setInputCloud(cloud_candidate);
//                 seg.segment(*inliers, *coefficients);

//                 if (!inliers->indices.empty()) {
//                     // 2. 验证提取出的模型是否真的像地面（二次保险）
//                     // 检查 coefficients->values[2] (C项，即法向量的Z分量) 是否接近 1
//                     if (std::abs(coefficients->values[2]) > 0.9) { 
                        
//                         // 3. 关键：不要重新跑 segment，而是使用 SampleConsensusModelPlane 手动筛选全集
//                         pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr 
//                             model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(tmp_voxel));
                        
//                         std::vector<int> final_inliers;
//                         // 根据第一次算出的 coefficients，找出全集中符合该平面的所有点
//                         model_p->selectWithinDistance(Eigen::Vector4f(coefficients->values[0], 
//                                                                     coefficients->values[1], 
//                                                                     coefficients->values[2], 
//                                                                     coefficients->values[3]), 
//                                                     0.10, // 缩小阈值到 0.1m
//                                                     final_inliers);

//                         // 4. 执行剔除
//                         pcl::PointIndices::Ptr final_indices(new pcl::PointIndices);
//                         final_indices->indices = final_inliers;
                        
//                         pcl::ExtractIndices<pcl::PointXYZI> extract;
//                         extract.setInputCloud(tmp_voxel);
//                         extract.setIndices(final_indices);
//                         extract.setNegative(true); 
//                         extract.filter(*tmp_nonground);
//                         ransac_success = true;
//                     }
//                 }
//             }
//             // if (cloud_candidate->size() > 50) {
//             //     seg.setInputCloud(cloud_candidate);
//             //     seg.segment(*inliers, *coefficients);
                
//             //     if (!inliers->indices.empty()) {
//             //         // 找到了地面模型，现在从 *tmp_voxel* 中剔除符合该模型的点
//             //         // 重新对全集计算距离，或者简单地剔除高度范围内的点
//             //         // 这里采用 robust 做法：回退到 ExtractIndices，但需要全集索引。
//             //         // 简化做法：如果 RANSAC 成功，则对全集再做一次分割以获取 indices
//             //         seg.setInputCloud(tmp_voxel);
//             //         seg.segment(*inliers, *coefficients);
                    
//             //         if (!inliers->indices.empty()) {
//             //             pcl::ExtractIndices<pcl::PointXYZI> extract;
//             //             extract.setInputCloud(tmp_voxel);
//             //             extract.setIndices(inliers);
//             //             extract.setNegative(true); // 去除地面点
//             //             extract.filter(*tmp_nonground);
//             //             ransac_success = true;
//             //         }
//             //     }
//             // }

//             // 如果 RANSAC 失败 (点太少或没找到平面)，回退到高度切割
//             if (!ransac_success) {
//                 pcl::PassThrough<pcl::PointXYZI> pass_floor;
//                 pass_floor.setInputCloud(tmp_voxel);
//                 pass_floor.setFilterFieldName("z");
//                 pass_floor.setFilterLimits(height_min_, height_filt_); 
//                 pass_floor.setFilterLimitsNegative(true); 
//                 pass_floor.filter(*tmp_nonground);
//             }
//         } else {
//             *tmp_nonground = *tmp_voxel;
//         }
//     }

//     // Stage 4: 强力去噪 (解决车体附件噪点)
//     // 即使 launch 文件没开，补盲雷达也强制建议开启 Radius Outlier Removal
//     // 或者在 launch 中把 radius_enble 设为 true
//     if (radius_enble_ && !tmp_nonground->empty()) {
//         pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
//         outrem.setInputCloud(tmp_nonground);
//         // 半径越小，去噪越狠；邻居数越大，去噪越狠
//         // 建议：0.2m 半径内至少要有 4 个点
//         outrem.setRadiusSearch(radius_radius_); // 建议 0.2 或 0.15
//         outrem.setMinNeighborsInRadius(radius_min_neighbors_); // 建议 3 或 4
//         outrem.filter(*tmp_nonground);
//     }

//     // 直接把 tmp_nonground 给 filter_cloud_ptr
//     *filter_cloud_ptr = *tmp_nonground;
// }

void LidarFilterCore::filterChargingStation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
{
    // 1. 检查开关
    if (!charge_enble_ || fliter_charge_ == 0 || cloud_ptr->empty()) return;

    // 2. 更新缓存 (如果 transPose_ 已在 chargeTimerCallback 中更新，这里可以省略，
    // 但为了确保线程安全，建议保留或在 callback 中调用)
    // 注意：transPose_ 是在 chargeTimerCallback 中更新的，我们需要确保它是最新的
    // 如果 chargeTimerCallback 频率足够高，这里可以直接使用 charge_cache_
    // 如果 chargeTimerCallback 频率较低，建议在这里调用 updateChargeCache(transPose_);
    updateChargeCache(transPose_);
    // 3. 执行过滤 (原地过滤 In-Place Filtering)
    // 优化：使用 std::partition 或 OpenMP 并行处理
    size_t original_size = cloud_ptr->size();
    size_t write_index = 0;
    
    // 使用 OpenMP 加速遍历
    #pragma omp parallel for
    for (size_t i = 0; i < cloud_ptr->size(); ++i) {
        // 如果点不在充电区域内，则保留
        if (!isPointInChargeArea(cloud_ptr->points[i])) {
            cloud_ptr->points[write_index++] = cloud_ptr->points[i];
        }
    }
    
    // 调整点云大小
    cloud_ptr->points.resize(write_index);
    cloud_ptr->width = cloud_ptr->points.size();
    cloud_ptr->height = 1;
    
    // debug_mode_

    if (1) {
        ROS_INFO_THROTTLE(1.0, "[Charge Filter] Removed %zu points (Original: %zu)", 
                          original_size - write_index, original_size);
    }
}



void LidarFilterCore::updateChargeCache(const geometry_msgs::Pose& pose) {
    std::lock_guard<std::mutex> lock(charge_cache_mutex_);
    charge_cache_.pose = pose;
    double yaw = tf2::getYaw(pose.orientation);
    charge_cache_.cos_theta = std::cos(yaw);
    charge_cache_.sin_theta = std::sin(yaw);
    
    // 充电桩尺寸参数，建议根据实际充电桩大小从参数服务器读取
    charge_cache_.half_length = charge_length_ / 2.0;
    charge_cache_.half_width = charge_wide_ / 2.0;
    charge_cache_.z_min = pose.position.z; 
    charge_cache_.z_max = pose.position.z + charge_high_;
    
    charge_cache_.valid = true;
}

bool LidarFilterCore::isPointInChargeArea(const pcl::PointXYZI& pt) {
    if (!charge_cache_.valid) return false;

    // 1. 平移到中心
    double dx = pt.x - charge_cache_.pose.position.x;
    double dy = pt.y - charge_cache_.pose.position.y;

    // 2. 旋转到局部坐标系 (Inverse Rotation)
    double local_x = dx * charge_cache_.cos_theta + dy * charge_cache_.sin_theta;
    double local_y = -dx * charge_cache_.sin_theta + dy * charge_cache_.cos_theta;

    // 3. 范围判断 (增加 0.1m 的 buffer 容错)
    return (std::abs(local_x) < (charge_cache_.half_length + 0.1) &&
            std::abs(local_y) < (charge_cache_.half_width + 0.1) &&
            pt.z > (charge_cache_.z_min - 0.2) && pt.z < (charge_cache_.z_max + 0.2));
}

std::vector<geometry_msgs::Point> LidarFilterCore::getRectangleVertices(
    const geometry_msgs::Pose& pose_, double length, double width) {
    
    std::vector<Eigen::Vector3d> local_vertices = {
        { length/2,  width/2, 0},
        {-length/2,  width/2, 0},
        {-length/2, -width/2, 0},
        { length/2, -width/2, 0}
    };
    
    geometry_msgs::Point center = pose_.position;
    geometry_msgs::Quaternion quat = pose_.orientation;
    
    Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    
    std::vector<geometry_msgs::Point> vertices;
    vertices.reserve(4);
    
    for (const auto& local_vertex : local_vertices) {
        Eigen::Vector3d world_vertex = rotation_matrix * local_vertex;
        geometry_msgs::Point p;
        p.x = world_vertex.x() + center.x;
        p.y = world_vertex.y() + center.y;
        p.z = world_vertex.z() + center.z;
        vertices.push_back(p);
    }
    
    return vertices;
}

void LidarFilterCore::filterVehicleBody(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                                        const std::vector<geometry_msgs::Point>& vehicle_polygon)
{
    if (in_cloud_ptr->empty()) return;
    out_cloud_ptr->clear();
    out_cloud_ptr->reserve(in_cloud_ptr->size());
    int num_threads = omp_get_max_threads();
    std::vector<std::vector<pcl::PointXYZI>> thread_buffers(num_threads);
    #pragma omp parallel
    {
        int thread_id = omp_get_thread_num();
        thread_buffers[thread_id].reserve(in_cloud_ptr->size() / num_threads);
        #pragma omp for nowait
        for(size_t i=0; i<in_cloud_ptr->size(); ++i) {
            const auto& pt = in_cloud_ptr->points[i];
            geometry_msgs::Point p; p.x = pt.x; p.y = pt.y; p.z = pt.z;
            bool in_vehicle_zone = pointInPolygon(p, vehicle_polygon) && (pt.z <= vehicle_height_);
            if (!in_vehicle_zone) {
                thread_buffers[thread_id].push_back(pt);
            }
        }
    }
    for (const auto& buf : thread_buffers) {
        out_cloud_ptr->points.insert(out_cloud_ptr->points.end(), buf.begin(), buf.end());
    }
    out_cloud_ptr->width = out_cloud_ptr->points.size();
    out_cloud_ptr->height = 1;
    out_cloud_ptr->is_dense = true;
}

bool LidarFilterCore::pointInPolygon(const geometry_msgs::Point& point, const std::vector<geometry_msgs::Point>& polyCorner) {
    if (polyCorner.empty()) return false;
    double min_x=1e9, max_x=-1e9, min_y=1e9, max_y=-1e9;
    for (const auto& p : polyCorner) { if(p.x<min_x) min_x=p.x; if(p.x>max_x) max_x=p.x; if(p.y<min_y) min_y=p.y; if(p.y>max_y) max_y=p.y; }
    if (point.x<min_x || point.x>max_x || point.y<min_y || point.y>max_y) return false; 
    bool result = false; int j = polyCorner.size() - 1;
    for (int i = 0; i < polyCorner.size(); j = i++) {
        if ((polyCorner[i].y > point.y) != (polyCorner[j].y > point.y) &&
            (point.x < (polyCorner[j].x - polyCorner[i].x) * (point.y - polyCorner[i].y) / (polyCorner[j].y - polyCorner[i].y) + polyCorner[i].x)) result = !result;
    }
    return result;
}

visualization_msgs::Marker LidarFilterCore::pubVehicleModel(const std::vector<geometry_msgs::Point>& polyCorner) {
    visualization_msgs::Marker marker; marker.header.frame_id = "velodyne"; marker.header.stamp = ros::Time::now();
    marker.ns = "vehicle_model"; marker.id = 0; marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD; marker.scale.x = 0.05; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    if (polyCorner.empty()) return marker;
    size_t n = polyCorner.size(); double h_min = -1.5; double h_max = vehicle_height_;
    for (size_t i = 0; i < n; ++i) {
        size_t next = (i + 1) % n;
        geometry_msgs::Point p1 = polyCorner[i]; geometry_msgs::Point p2 = polyCorner[next];
        geometry_msgs::Point b1; b1.x=p1.x; b1.y=p1.y; b1.z=h_min; geometry_msgs::Point b2; b2.x=p2.x; b2.y=p2.y; b2.z=h_min;
        geometry_msgs::Point t1; t1.x=p1.x; t1.y=p1.y; t1.z=h_max; geometry_msgs::Point t2; t2.x=p2.x; t2.y=p2.y; t2.z=h_max;
        marker.points.push_back(b1); marker.points.push_back(b2);
        marker.points.push_back(t1); marker.points.push_back(t2);
        marker.points.push_back(b1); marker.points.push_back(t1);
    }
    return marker;     
}
// std::vector<geometry_msgs::Point> LidarFilterCore::getRectangleVertices(const geometry_msgs::Pose pose_, double length, double width) {
//     std::vector<Eigen::Vector3d> local = {{length/2, width/2, 0}, {-length/2, width/2, 0}, {-length/2, -width/2, 0}, {length/2, -width/2, 0}};
//     geometry_msgs::Point c = pose_.position; Eigen::Quaterniond q(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
//     Eigen::Matrix3d rot = q.toRotationMatrix(); std::vector<geometry_msgs::Point> v;
//     for (const auto& l : local) { Eigen::Vector3d w = rot * l; geometry_msgs::Point p; p.x = w.x() + c.x; p.y = w.y() + c.y; p.z = w.z() + c.z; v.push_back(p); }
//     return v;
// }
void LidarFilterCore::displayPolygonInRviz(ros::Publisher& publisher, const geometry_msgs::Pose& pose_, float scale_x, float scale_y, float scale_z) {
    visualization_msgs::Marker m; m.header.frame_id = "map"; m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD; m.id = 0; m.scale.x = 0.05; m.color.a = 1.0; m.color.r = 1.0;
    std::vector<geometry_msgs::Point> v = getRectangleVertices(pose_, scale_x, scale_y);
    if(!v.empty()) { m.points = v; m.points.push_back(v[0]); }
    visualization_msgs::MarkerArray ma; ma.markers.push_back(m); publisher.publish(ma);
}
bool LidarFilterCore::transformPose(const geometry_msgs::TransformStamped& transform, const geometry_msgs::Pose& input_pose, geometry_msgs::Pose& output_pose) {
    try { geometry_msgs::PoseStamped i, o; i.pose = input_pose; tf2::doTransform(i, o, transform); output_pose = o.pose; return true; } catch (...) { return false; }
}
void LidarFilterCore::keyPointCallback(const autoware_msgs::KeyPointArrayConstPtr &msg) {
    fliterpose_.clear(); bool found = false;
    if(!msg->path.empty()) {
        std::vector<int> idxs = {0, (int)msg->path.size()-1};
        for(int i : idxs) {
            if(i < 0 || i >= msg->path.size()) continue;
            for (const auto& t : msg->path.at(i).types) {
                if(t.type_name == "charges") {
                    geometry_msgs::Pose p = msg->path.at(i).pose.pose;
                    if(charge_error_!=0) { p.position.x += cos(tf2::getYaw(p.orientation))*charge_error_; p.position.y += sin(tf2::getYaw(p.orientation))*charge_error_; }
                    fliterpose_.push_back(p); displayPolygonInRviz(marker_pub_, p, charge_length_, charge_wide_, charge_high_); found = true;
                }
            }
        }
    }
    if(!found) { geometry_msgs::Pose e; displayPolygonInRviz(marker_pub_, e, 0, 0, 0); }
}
void LidarFilterCore::ctrolCallback(const std_msgs::Int8ConstPtr &msg) {
    if(charge_enble_ && msg->data == 2) fliter_charge_ = 1; else if(charge_enble_ && msg->data < 0) fliter_charge_ = 2; else fliter_charge_ = 0;
    if(fabs(msg->data) == 4) enbleElevator_ = true; else enbleElevator_ = false;
}
void LidarFilterCore::chargeTimerCallback(const ros::TimerEvent& event) {
    if(fliter_charge_ != 0 && !fliterpose_.empty()) {
        try {
            geometry_msgs::TransformStamped t = tf_buffer_.lookupTransform("velodyne", "map", ros::Time(0));
            geometry_msgs::Pose tp; if(fliterpose_.size() == 1) tp = fliterpose_[0]; else tp = (fliter_charge_ == 2) ? fliterpose_[0] : fliterpose_[1];
            transformPose(t, tp, transPose_);
        } catch (...) {}
    }
}