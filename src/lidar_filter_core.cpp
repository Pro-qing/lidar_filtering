#include "lidar_filtering/lidar_filter_core.hpp"

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE 
#endif

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
    int max_threads = omp_get_max_threads();
    omp_buffers_filter_.resize(max_threads);
    omp_buffers_vehicle_.resize(max_threads);

    key_points_sub_ = nh_.subscribe("/keypoint_path", 1, &LidarFilterCore::keyPointCallback, this);
    ctrol_sub_ = nh_.subscribe("/lqr_dire", 1, &LidarFilterCore::ctrolCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("charge_marker", 1, true);
    charge_timer_ = nh_.createTimer(ros::Duration(0.2), &LidarFilterCore::chargeTimerCallback, this);
}

void LidarFilterCore::updateNativeConfig(const NativeFilterConfig& config) {
    std::lock_guard<std::mutex> lock(core_param_mutex_);
    config_ = config;
    charge_enble_ = config.charge_enble;
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
    NativeFilterConfig local_cfg;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_cfg = config_;
    }

    if (!local_cfg.consistency_enable) return;

    double min_rad = local_cfg.consistency_min_angle * M_PI / 180.0;
    double max_rad = local_cfg.consistency_max_angle * M_PI / 180.0;
    double tol_dist = local_cfg.consistency_diff_dist;

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
            if (!std::isinf(r_right) && !std::isnan(r_right)) {
                if (std::abs(r_left - r_right) > tol_dist) {
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

void LidarFilterCore::pointcloud_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                                        bool update_history)
{
    if (in_cloud_ptr->empty()) return;
    filter_cloud_ptr->clear();
    filter_cloud_ptr->reserve(in_cloud_ptr->size() * 0.5);

    NativeFilterConfig local_cfg;
    bool charge_active;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_cfg = config_;
        charge_active = charge_enble_ && charge_cache_.valid;
    }

    double r_sq_max = local_cfg.crop_radius * local_cfg.crop_radius;
    if (r_sq_max > 6400.0) r_sq_max = 6400.0; 
    double min_dist_sq = local_cfg.voxel_filter * local_cfg.voxel_filter;
    if (min_dist_sq < 0.0001) min_dist_sq = 0.0001;

    int num_threads = omp_get_max_threads();
    for (int i = 0; i < num_threads; ++i) {
        omp_buffers_filter_[i].clear();
    }

    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        omp_buffers_filter_[tid].reserve(in_cloud_ptr->size() / num_threads);
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
                omp_buffers_filter_[tid].push_back(pt);
                continue; 
            }

            if (pt.z < local_cfg.height_min || pt.z > local_cfg.height_max) continue;
            if (local_cfg.filter_floor && pt.z < local_cfg.height_filt) continue;
            
            double dx = pt.x - local_cfg.crop_radius_x;
            if ((dx * dx + pt.y * pt.y) > r_sq_max) continue;
            
            double d_sq = (pt.x-last_x)*(pt.x-last_x) + (pt.y-last_y)*(pt.y-last_y) + (pt.z-last_z)*(pt.z-last_z);
            if (d_sq < min_dist_sq) continue;
            
            omp_buffers_filter_[tid].push_back(pt);
            last_x = pt.x; last_y = pt.y; last_z = pt.z;
        }
    }

    for (int i = 0; i < num_threads; ++i) {
        filter_cloud_ptr->points.insert(filter_cloud_ptr->points.end(), 
                                        omp_buffers_filter_[i].begin(), 
                                        omp_buffers_filter_[i].end());
    }

    filter_cloud_ptr->width = filter_cloud_ptr->points.size();
    filter_cloud_ptr->height = 1;
    filter_cloud_ptr->is_dense = true;

    if (local_cfg.time_consistency_filter && update_history) {
        std::lock_guard<std::mutex> lock(history_mutex_);
        prev_non_ground_cloud_ = *filter_cloud_ptr;
    }
}

void LidarFilterCore::pointcloud_filter_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_ptr,
                                            bool update_history)
{
    if (in_cloud_ptr->empty()) return;

    NativeFilterConfig local_cfg;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_cfg = config_;
    }

    static pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cropped(new pcl::PointCloud<pcl::PointXYZI>);
    static pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_voxel(new pcl::PointCloud<pcl::PointXYZI>);
    static pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_nonground(new pcl::PointCloud<pcl::PointXYZI>);
    
    tmp_cropped->clear(); tmp_voxel->clear(); tmp_nonground->clear();

    // Stage 1
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

    // Stage 2
    {
        double leaf_size = enbleElevator_ ? local_cfg.voxel_filter_eleva : local_cfg.voxel_filter;
        if(leaf_size < 0.02) leaf_size = 0.02; 
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(tmp_cropped);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.setMinimumPointsNumberPerVoxel(3); 
        vg.filter(*tmp_voxel);
    }

    // Stage 3
    bool ransac_success = false;
    if (local_cfg.filter_floor) {
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
            pass_floor.setFilterLimits(local_cfg.height_min, -1.80); 
            pass_floor.setFilterLimitsNegative(true); 
            pass_floor.filter(*tmp_nonground);
        }
    }

    // Stage 4
    if (local_cfg.radius_enble && !tmp_nonground->empty()) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
        outrem.setInputCloud(tmp_nonground);
        outrem.setRadiusSearch(local_cfg.radius_radius);
        outrem.setMinNeighborsInRadius(local_cfg.radius_min_neighbors);
        outrem.filter(*tmp_nonground);
    }

    *filter_cloud_ptr = *tmp_nonground;
}

void LidarFilterCore::filterChargingStation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
{
    if (!charge_enble_ || fliter_charge_ == 0 || cloud_ptr->empty()) return;

    updateChargeCache(transPose_);
    size_t write_index = 0;
    
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
    NativeFilterConfig local_cfg;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        local_cfg = config_;
    }

    std::lock_guard<std::mutex> lock(charge_cache_mutex_);
    charge_cache_.pose = pose;
    double yaw = tf2::getYaw(pose.orientation);
    charge_cache_.cos_theta = std::cos(yaw);
    charge_cache_.sin_theta = std::sin(yaw);
    charge_cache_.half_length = local_cfg.charge_length / 2.0;
    charge_cache_.half_width = local_cfg.charge_wide / 2.0;
    charge_cache_.z_min = pose.position.z; 
    charge_cache_.z_max = pose.position.z + local_cfg.charge_high;
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
        { length/2,  width/2, 0}, {-length/2,  width/2, 0},
        {-length/2, -width/2, 0}, { length/2, -width/2, 0}
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
        p.x = world_vertex.x() + center.x; p.y = world_vertex.y() + center.y; p.z = world_vertex.z() + center.z;
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
        local_v_height = config_.vehicle_height;
    }

    out_cloud_ptr->clear();
    out_cloud_ptr->reserve(in_cloud_ptr->size());

    int num_threads = omp_get_max_threads();
    for (int i = 0; i < num_threads; ++i) {
        omp_buffers_vehicle_[i].clear();
    }

    #pragma omp parallel
    {
        int thread_id = omp_get_thread_num();
        omp_buffers_vehicle_[thread_id].reserve(in_cloud_ptr->size() / num_threads);
        
        #pragma omp for nowait
        for(size_t i=0; i<in_cloud_ptr->size(); ++i) {
            const auto& pt = in_cloud_ptr->points[i];
            geometry_msgs::Point p; p.x = pt.x; p.y = pt.y; p.z = pt.z;
            bool in_vehicle_zone = pointInPolygon(p, vehicle_polygon) && (pt.z <= local_v_height);
            if (!in_vehicle_zone) {
                omp_buffers_vehicle_[thread_id].push_back(pt);
            }
        }
    }
    for (int i = 0; i < num_threads; ++i) {
        out_cloud_ptr->points.insert(out_cloud_ptr->points.end(), 
                                     omp_buffers_vehicle_[i].begin(), 
                                     omp_buffers_vehicle_[i].end());
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
        local_v_height = config_.vehicle_height;
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
    visualization_msgs::Marker m; 
    m.header.frame_id = "map"; 
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.id = 0; 
    m.scale.x = 0.05; 
    m.color.a = 1.0; 
    m.color.r = 1.0;  

    if (scale_x <= 0.001 || scale_y <= 0.001) {
        m.action = visualization_msgs::Marker::DELETE;
    } else {
        m.action = visualization_msgs::Marker::ADD;
        std::vector<geometry_msgs::Point> bottom_v = getRectangleVertices(pose_, scale_x, scale_y);
        if(!bottom_v.empty()) { 
            std::vector<geometry_msgs::Point> top_v = bottom_v;
            for (auto& p : top_v) {
                p.z += scale_z;
            }
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

bool LidarFilterCore::transformPose(const geometry_msgs::TransformStamped& transform, const geometry_msgs::Pose& input_pose, geometry_msgs::Pose& output_pose) {
    try { geometry_msgs::PoseStamped i, o; i.pose = input_pose; tf2::doTransform(i, o, transform); output_pose = o.pose; return true; } catch (...) { return false; }
}

void LidarFilterCore::keyPointCallback(const autoware_msgs::KeyPointArrayConstPtr &msg) {
    double error, len, wide, high;
    {
        std::lock_guard<std::mutex> lock(core_param_mutex_);
        error = config_.charge_error; len = config_.charge_length; wide = config_.charge_wide; high = config_.charge_high;
    }
    fliterpose_.clear(); bool found = false;
    if(!msg->path.empty()) {
        for(const auto& point : msg->path) {
            for (const auto& type : point.types) {
                if(type.type_name == "charges") {
                    geometry_msgs::Pose p = point.pose.pose;
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
    if(charge_enble_ && msg->data == 2) fliter_charge_ = 1; 
    else if(charge_enble_ && msg->data < 0) fliter_charge_ = 2; 
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