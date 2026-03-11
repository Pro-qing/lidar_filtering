#include "lidar_filtering/lidar_filter_core.hpp"

// 强制在头文件展开模板以防链接报错
#define PCL_NO_PRECOMPILE 

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>       

#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>     

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/impl/radius_outlier_removal.hpp> 

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>  

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>        

#include <pcl/sample_consensus/sac_model_plane.h> 
#include <pcl/sample_consensus/impl/sac_model_plane.hpp> 

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <omp.h>
#include <cmath>

LidarFilterCore::LidarFilterCore(ros::NodeHandle &nh, ros::NodeHandle &private_nh) 
    : nh_(nh), private_nh_(private_nh), tf_listener_(tf_buffer_)
{
    // 初始化时从 ROS 参数服务器读取默认值
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

    private_nh_.param("consistency_enable", consistency_enable_, false);
    private_nh_.param("consistency_min_angle", consistency_min_angle_, -30.0);
    private_nh_.param("consistency_max_angle", consistency_max_angle_, 30.0);
    private_nh_.param("consistency_diff_dist", consistency_diff_dist_, 1.2);

    key_points_sub_ = nh_.subscribe("/keypoint_path", 1, &LidarFilterCore::keyPointCallback, this);
    ctrol_sub_ = nh_.subscribe("/lqr_dire", 1, &LidarFilterCore::ctrolCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("charge_marker", 1, true);
    charge_timer_ = nh_.createTimer(ros::Duration(0.2), &LidarFilterCore::chargeTimerCallback, this);
}

// =========================================================
// 【新增】接收 RQT 动态下发的配置参数
// =========================================================
void LidarFilterCore::updateDynamicConfig(const lidar_filtering::LidarFilteringConfig& config) {
    std::lock_guard<std::mutex> lock(core_param_mutex_);
    
    crop_radius_ = config.crop_radius;
    crop_radius_x_ = config.crop_radius_x;
    height_max_ = config.height_max;
    height_min_ = config.height_min;
    height_filt_ = config.height_filt;
    filter_floor_ = config.filter_floor;
    
    voxel_filter_ = config.voxel_filter;
    voxel_filter_auto_ = config.voxel_filter_auto;
    voxel_filter_eleva_ = config.voxel_filter_eleva;

    filter_transient_ = config.filter_transient;
    neighboring_points_ = config.neighboring_points;
    stand_threshold_ = config.stand_threshold;
    time_consistency_filter_ = config.time_consistency_filter;

    radius_enble_ = config.radius_enble;
    radius_radius_ = config.radius_radius;
    radius_min_neighbors_ = config.radius_min_neighbors;

    charge_enble_ = config.charge_enble;
    charge_length_ = config.charge_length;
    charge_wide_ = config.charge_wide;
    charge_high_ = config.charge_high;
    charge_error_ = config.charge_error;

    consistency_enable_ = config.consistency_enable;
    consistency_min_angle_ = config.consistency_min_angle;
    consistency_max_angle_ = config.consistency_max_angle;
    consistency_diff_dist_ = config.consistency_diff_dist;
}

// 静态辅助：原单区间过滤
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

// 静态辅助：双区间保留过滤（带低速安全距离保护）
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
        if (std::isinf(scan.ranges[i]) || std::isnan(scan.ranges[i])) continue;
        if (scan.ranges[i] > max_dis) { scan.ranges[i] = std::numeric_limits<float>::infinity(); continue; }

        double angle_rad = angle_min + i * angle_inc;
        double angle_deg = angle_rad * 180.0 / M_PI;
        while (angle_deg < 0) angle_deg += 360.0;
        while (angle_deg >= 360.0) angle_deg -= 360.0;

        bool keep = false;
        if (angle_deg >= a && angle_deg <= b) keep = true;
        if (angle_deg >= c && angle_deg <= d) keep = true;

        bool should_remove = !keep;

        if (!should_remove && is_limit_mode) {
            bool in_limit_angle = false;
            if (limit_min_deg > limit_max_deg) {
                if (angle_deg > limit_min_deg || angle_deg < limit_max_deg) in_limit_angle = true;
            } else {
                if (angle_deg > limit_min_deg && angle_deg < limit_max_deg) in_limit_angle = true;
            }

            if (in_limit_angle && scan.ranges[i] > limit_dis) {
                should_remove = true;
            }
        }
        
        if (should_remove) scan.ranges[i] = std::numeric_limits<float>::infinity();
    }
}

void LidarFilterCore::checkScanConsistency(sensor_msgs::LaserScan& left, sensor_msgs::LaserScan& right, 
                                           double left_yaw, double right_yaw)
{
    // 【新增】安全获取动态参数
    bool enable;
    double min_angle, max_angle, diff_dist;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        enable = consistency_enable_;
        min_angle = consistency_min_angle_;
        max_angle = consistency_max_angle_;
        diff_dist = consistency_diff_dist_;
    }

    if (!enable) return;

    double min_rad = min_angle * M_PI / 180.0;
    double max_rad = max_angle * M_PI / 180.0;
    double tol_dist = diff_dist;

    int left_size = left.ranges.size();
    double right_angle_min = right.angle_min;
    double right_angle_inc = right.angle_increment;
    int right_size = right.ranges.size();

    for (int i = 0; i < left_size; ++i) {
        float r_left = left.ranges[i];
        if (std::isinf(r_left) || std::isnan(r_left)) continue;

        double angle_local = left.angle_min + i * left.angle_increment;
        double angle_vehicle = angle_local + left_yaw;

        while (angle_vehicle > M_PI) angle_vehicle -= 2.0 * M_PI;
        while (angle_vehicle < -M_PI) angle_vehicle += 2.0 * M_PI;

        if (angle_vehicle < min_rad || angle_vehicle > max_rad) continue;

        double angle_right_local = angle_vehicle - right_yaw;
        while (angle_right_local > M_PI) angle_right_local -= 2.0 * M_PI;
        while (angle_right_local < -M_PI) angle_right_local += 2.0 * M_PI;

        int j = std::round((angle_right_local - right_angle_min) / right_angle_inc);

        if (j >= 0 && j < right_size) {
            float r_right = right.ranges[j];
            
            if (std::isinf(r_right) || std::isnan(r_right)) {
                // do nothing
            } 
            else if (std::abs(r_left - r_right) > tol_dist) {
                if (r_left < r_right) {
                    right.ranges[j] = std::numeric_limits<float>::infinity(); 
                } else {
                    left.ranges[i] = std::numeric_limits<float>::infinity();  
                }
            }
        } 
    }
}

// OpenMP 版点云滤波器
void LidarFilterCore::pointcloud_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                                        bool update_history)
{
    if (in_cloud_ptr->empty()) return;
    filter_cloud_ptr->clear();
    filter_cloud_ptr->reserve(in_cloud_ptr->size() * 0.5);

    // 【新增】安全拷贝局部变量（防线程冲突）
    double local_crop_radius, local_crop_radius_x, z_min, z_max, z_floor;
    double local_voxel_filter;
    bool do_floor, charge_active, do_time_consistency;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_crop_radius = crop_radius_;
        local_crop_radius_x = crop_radius_x_;
        z_min = height_min_;
        z_max = height_max_;
        do_floor = filter_floor_;
        z_floor = height_filt_;
        local_voxel_filter = voxel_filter_;
        charge_active = charge_enble_ && charge_cache_.valid;
        do_time_consistency = time_consistency_filter_;
    }

    double r_sq_max = local_crop_radius * local_crop_radius;
    if (r_sq_max > 6400.0) r_sq_max = 6400.0; 
    
    double min_dist_sq = local_voxel_filter * local_voxel_filter;
    if (min_dist_sq < 0.0001) min_dist_sq = 0.0001;

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
            
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

            bool in_charge_zone = false;
            if (charge_active) {
                in_charge_zone = isPointInChargeArea(pt);
            }

            if (in_charge_zone) {
                thread_buffers[tid].push_back(pt);
                continue; 
            }

            if (pt.z < z_min || pt.z > z_max) continue;
            if (do_floor && pt.z < z_floor) continue;
            
            double dx = pt.x - local_crop_radius_x;
            if ((dx * dx + pt.y * pt.y) > r_sq_max) continue;
            
            double d_sq = (pt.x-last_x)*(pt.x-last_x) + (pt.y-last_y)*(pt.y-last_y) + (pt.z-last_z)*(pt.z-last_z);
            if (d_sq < min_dist_sq) continue;
            
            thread_buffers[tid].push_back(pt);
            last_x = pt.x; last_y = pt.y; last_z = pt.z;
        }
    }

    for (const auto& buf : thread_buffers) {
        filter_cloud_ptr->points.insert(filter_cloud_ptr->points.end(), buf.begin(), buf.end());
    }

    filter_cloud_ptr->width = filter_cloud_ptr->points.size();
    filter_cloud_ptr->height = 1;
    filter_cloud_ptr->is_dense = true;

    if (do_time_consistency && update_history) {
        std::lock_guard<std::mutex> lock(history_mutex_);
        prev_non_ground_cloud_ = *filter_cloud_ptr;
    }
}

// PCL 标准版点云滤波器
void LidarFilterCore::pointcloud_filter_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                                            bool update_history)
{
    if (in_cloud_ptr->empty()) return;

    // 【新增】安全拷贝局部变量
    double local_crop_radius, local_crop_radius_x, z_min, z_max;
    double local_voxel, local_voxel_eleva;
    bool do_floor, do_radius_filter;
    double local_radius_radius;
    int local_radius_neighbors;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_crop_radius = crop_radius_;
        local_crop_radius_x = crop_radius_x_;
        z_min = height_min_;
        z_max = height_max_;
        do_floor = filter_floor_;
        local_voxel = voxel_filter_;
        local_voxel_eleva = voxel_filter_eleva_;
        do_radius_filter = radius_enble_;
        local_radius_radius = radius_radius_;
        local_radius_neighbors = radius_min_neighbors_;
    }

    static thread_local pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cropped(new pcl::PointCloud<pcl::PointXYZI>);
    static thread_local pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_voxel(new pcl::PointCloud<pcl::PointXYZI>);
    static thread_local pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_nonground(new pcl::PointCloud<pcl::PointXYZI>);
    
    tmp_cropped->clear(); tmp_voxel->clear(); tmp_nonground->clear();

    // Stage 1
    {
        double safe_radius = (local_crop_radius > 80.0) ? 80.0 : local_crop_radius;
        double safe_r_sq = safe_radius * safe_radius;
        for (const auto& pt : in_cloud_ptr->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            if (pt.z < z_min || pt.z > z_max) continue;
            double dx = pt.x - local_crop_radius_x;
            if ((dx * dx + pt.y * pt.y) > safe_r_sq) continue;
            tmp_cropped->push_back(pt);
        }
    }
    if (tmp_cropped->empty()) return;

    // Stage 2
    {
        double leaf_size = enbleElevator_ ? local_voxel_eleva : local_voxel;
        if(leaf_size < 0.02) leaf_size = 0.02; 
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(tmp_cropped);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.setMinimumPointsNumberPerVoxel(3); 
        vg.filter(*tmp_voxel);
    }

    // Stage 3
    bool ransac_success = false;
    if (do_floor) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_seeds(new pcl::PointCloud<pcl::PointXYZI>);
        
        for (const auto& pt : tmp_voxel->points) {
            if (pt.x > 0.1 && pt.x < 8.0 && std::abs(pt.y) < 2.5) {
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
            seg.setEpsAngle(15.0 * M_PI / 180.0); 
            seg.setDistanceThreshold(0.12);      
            seg.setMaxIterations(150);
            seg.setInputCloud(ground_seeds);
            seg.segment(*inliers, *coefficients);

            if (!inliers->indices.empty()) {
                float c = coefficients->values[2];
                float d = coefficients->values[3];
                float ground_h = -d / c;

                if (std::abs(c) > 0.90 && ground_h < -1.75) { 
                    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr 
                        model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(tmp_voxel));
                    
                    std::vector<int> final_inliers;
                    Eigen::Vector4f model_vec(coefficients->values[0], coefficients->values[1], c, d);
                    
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

        if (!ransac_success) {
            pcl::PassThrough<pcl::PointXYZI> pass_floor;
            pass_floor.setInputCloud(tmp_voxel);
            pass_floor.setFilterFieldName("z");
            pass_floor.setFilterLimits(z_min, -1.80); 
            pass_floor.setFilterLimitsNegative(true); 
            pass_floor.filter(*tmp_nonground);
        }
    }

    // Stage 4
    if (do_radius_filter && !tmp_nonground->empty()) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
        outrem.setInputCloud(tmp_nonground);
        outrem.setRadiusSearch(local_radius_radius);
        outrem.setMinNeighborsInRadius(local_radius_neighbors);
        outrem.filter(*tmp_nonground);
    }

    *filter_cloud_ptr = *tmp_nonground;
}

void LidarFilterCore::filterChargingStation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
{
    bool do_charge;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        do_charge = charge_enble_;
    }

    if (!do_charge || fliter_charge_ == 0 || cloud_ptr->empty()) return;

    updateChargeCache(transPose_);
    size_t write_index = 0;
    
    #pragma omp parallel for
    for (size_t i = 0; i < cloud_ptr->size(); ++i) {
        if (!isPointInChargeArea(cloud_ptr->points[i])) {
            cloud_ptr->points[write_index++] = cloud_ptr->points[i];
        }
    }
    
    cloud_ptr->points.resize(write_index);
    cloud_ptr->width = cloud_ptr->points.size();
    cloud_ptr->height = 1;
}

void LidarFilterCore::updateChargeCache(const geometry_msgs::Pose& pose) {
    double length, wide, high;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        length = charge_length_;
        wide = charge_wide_;
        high = charge_high_;
    }

    std::lock_guard<std::mutex> lock(charge_cache_mutex_);
    charge_cache_.pose = pose;
    double yaw = tf2::getYaw(pose.orientation);
    charge_cache_.cos_theta = std::cos(yaw);
    charge_cache_.sin_theta = std::sin(yaw);
    
    charge_cache_.half_length = length / 2.0;
    charge_cache_.half_width = wide / 2.0;
    charge_cache_.z_min = pose.position.z; 
    charge_cache_.z_max = pose.position.z + high;
    
    charge_cache_.valid = true;
}

bool LidarFilterCore::isPointInChargeArea(const pcl::PointXYZI& pt) {
    if (!charge_cache_.valid) return false;

    double dx = pt.x - charge_cache_.pose.position.x;
    double dy = pt.y - charge_cache_.pose.position.y;

    double local_x = dx * charge_cache_.cos_theta + dy * charge_cache_.sin_theta;
    double local_y = -dx * charge_cache_.sin_theta + dy * charge_cache_.cos_theta;

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

    double local_v_height;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_v_height = vehicle_height_;
    }

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
            bool in_vehicle_zone = pointInPolygon(p, vehicle_polygon) && (pt.z <= local_v_height);
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
    double local_v_height;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_v_height = vehicle_height_;
    }

    visualization_msgs::Marker marker; marker.header.frame_id = "velodyne"; marker.header.stamp = ros::Time::now();
    marker.ns = "vehicle_model"; marker.id = 0; marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD; marker.scale.x = 0.05; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    if (polyCorner.empty()) return marker;
    size_t n = polyCorner.size(); double h_min = -1.5; double h_max = local_v_height;
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
    double error, len, wide, high;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        error = charge_error_;
        len = charge_length_;
        wide = charge_wide_;
        high = charge_high_;
    }

    fliterpose_.clear(); bool found = false;
    if(!msg->path.empty()) {
        std::vector<int> idxs = {0, (int)msg->path.size()-1};
        for(int i : idxs) {
            if(i < 0 || i >= msg->path.size()) continue;
            for (const auto& t : msg->path.at(i).types) {
                if(t.type_name == "charges") {
                    geometry_msgs::Pose p = msg->path.at(i).pose.pose;
                    if(error != 0) { 
                        p.position.x += cos(tf2::getYaw(p.orientation)) * error; 
                        p.position.y += sin(tf2::getYaw(p.orientation)) * error; 
                    }
                    fliterpose_.push_back(p); 
                    displayPolygonInRviz(marker_pub_, p, len, wide, high); 
                    found = true;
                }
            }
        }
    }
    if(!found) { geometry_msgs::Pose e; displayPolygonInRviz(marker_pub_, e, 0, 0, 0); }
}

void LidarFilterCore::ctrolCallback(const std_msgs::Int8ConstPtr &msg) {
    bool do_charge;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        do_charge = charge_enble_;
    }
    if(do_charge && msg->data == 2) fliter_charge_ = 1; 
    else if(do_charge && msg->data < 0) fliter_charge_ = 2; 
    else fliter_charge_ = 0;
    
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