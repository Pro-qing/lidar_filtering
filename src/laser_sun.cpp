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