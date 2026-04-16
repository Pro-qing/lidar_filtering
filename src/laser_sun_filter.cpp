    // // ================== 单线阳光点过滤 ==================
    // // 阳光引起的噪点通常是孤立的。使用半径滤波剔除这些孤立点。
    // static pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror_filter;
    
    // // 【参数调节建议】
    // // setRadiusSearch: 搜索半径。考虑到单线雷达远处的点距会变大，建议设为 0.2 ~ 0.4 米。
    // // setMinNeighborsInRadius: 邻居数阈值。设为 2 表示：如果一个点半径0.3米内少于2个点，则被当做阳光噪点删掉。
    // ror_filter.setRadiusSearch(0.2);       
    // ror_filter.setMinNeighborsInRadius(2); 

    // if (!buf_left.filt->empty()) {
    //     // 静态指针，防止每帧重复申请内存
    //     static pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_left_cleaned(new pcl::PointCloud<pcl::PointXYZI>());
    //     tmp_left_cleaned->clear();
        
    //     ror_filter.setInputCloud(buf_left.filt);
    //     ror_filter.filter(*tmp_left_cleaned);
        
    //     // 内存优化：使用 swap 高效交换数据，避免深拷贝
    //     buf_left.filt->swap(*tmp_left_cleaned);
    // }

    // if (!buf_right.filt->empty()) {
    //     // 静态指针，防止每帧重复申请内存
    //     static pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_right_cleaned(new pcl::PointCloud<pcl::PointXYZI>());
    //     tmp_right_cleaned->clear();
        
    //     ror_filter.setInputCloud(buf_right.filt);
    //     ror_filter.filter(*tmp_right_cleaned);
        
    //     buf_right.filt->swap(*tmp_right_cleaned);
    // }
    // // ====================================================


// // ================== 改进版单线阳光点过滤 ==================
// // 1. 提取近距离点云 (0.5m内)
// pcl::PointCloud<pcl::PointXYZI>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// pcl::PointCloud<pcl::PointXYZI>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZI>);

// for (const auto& pt : *buf_left.filt) {
//     float dist = std::sqrt(pt.x*pt.x + pt.y*pt.y);
//     if (dist < 0.5f) 
//         near_cloud->push_back(pt);
//     else 
//         far_cloud->push_back(pt);
// }

// // 2. 仅对近距离点进行半径滤波
// if (!near_cloud->empty()) {
//     static pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror_filter;
//     ror_filter.setRadiusSearch(0.15); // 稍微缩小半径，提高针对性
//     ror_filter.setMinNeighborsInRadius(3); // 增加邻居要求，防止误触
    
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cleaned_near(new pcl::PointCloud<pcl::PointXYZI>);
//     ror_filter.setInputCloud(near_cloud);
//     ror_filter.filter(*cleaned_near);

//     // 3. 合并结果
//     buf_left.filt->clear();
//     *buf_left.filt += *cleaned_near;
//     *buf_left.filt += *far_cloud;
// }


#include <map>

// ================== 综合过滤逻辑 (强度 + ROR + 多帧) ==================
// 建议在类成员中定义，或者使用 static 保持状态
static std::map<int, int> near_point_count_map; // 用于存放每个角度（或栅格）点的出现次数
const int FRAME_THRESHOLD = 3;  // 必须连续出现 3 帧才认可

// 1. 准备容器
pcl::PointCloud<pcl::PointXYZI>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZI>);

// 2. 第一步：距离分段 + 强度过滤
for (const auto& pt : *buf_left.filt) {
    float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    
    if (dist < 0.6f) {
        // 【强度过滤】根据实验值调整，通常灯光噪点强度极低（如 < 10）
        if (pt.intensity < 6) continue; 
        near_cloud->push_back(pt);
    } else {
        far_cloud->push_back(pt);
    }
}

// 3. 第二步：半径滤波 (仅针对近距离点)
pcl::PointCloud<pcl::PointXYZI>::Ptr ror_cleaned_near(new pcl::PointCloud<pcl::PointXYZI>);
if (!near_cloud->empty()) {
    static pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror_filter;
    ror_filter.setRadiusSearch(0.1); 
    ror_filter.setMinNeighborsInRadius(4); 
    ror_filter.setInputCloud(near_cloud);
    ror_filter.filter(*ror_cleaned_near);
}

// 4. 第三步：多帧确认 (Temporal Filtering)
// 针对近距离过滤后的点，判断其是否在时间轴上稳定
pcl::PointCloud<pcl::PointXYZI>::Ptr final_near_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// 简单的角度栅格化方法：将 360 度分为 3600 个栅格（0.1度一个）
std::map<int, int> current_frame_hits;

for (const auto& pt : *ror_cleaned_near) {
    // 计算角度索引作为 Key
    float angle = std::atan2(pt.y, pt.x) * 180.0 / M_PI;
    int angle_idx = static_cast<int>(angle * 10); // 0.1度精度

    // 如果这个角度在上一帧也存在，计数加1
    current_frame_hits[angle_idx] = near_point_count_map[angle_idx] + 1;

    if (current_frame_hits[angle_idx] >= FRAME_THRESHOLD) {
        final_near_cloud->push_back(pt);
    }
}
// 更新全局计数地图（只保留当前帧命中的，没命中的会自动清零，实现类似滑动窗口效果）
near_point_count_map = current_frame_hits;

// 5. 合并并写回
buf_left.filt->clear();
*buf_left.filt += *final_near_cloud;
*buf_left.filt += *far_cloud;
// ====================================================