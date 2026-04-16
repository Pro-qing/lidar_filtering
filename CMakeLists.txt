cmake_minimum_required(VERSION 3.0.2)
project(lidar_filtering)

# 1. 设置 C++ 标准为 14 (PCL 1.8 建议 C++11 或 C++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. 【关键修改】开启 PCL_NO_PRECOMPILE 宏，防止 PointXYZI 报链接错误
add_definitions(-DPCL_NO_PRECOMPILE)

# 3. 指定寻找 PCL 1.8
find_package(PCL 1.8  REQUIRED COMPONENTS common io filters segmentation sample_consensus kdtree search)

# 包含 PCL 的头文件和库
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  message_filters
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  laser_geometry
  dynamic_reconfigure
  autoware_msgs
  autoware_can_msgs
  visualization_msgs
)

find_package(OpenMP REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/LidarFiltering.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros dynamic_reconfigure
)

# 核心处理库
add_library(lidar_filter_core
  src/lidar_filter_core.cpp
)
target_link_libraries(lidar_filter_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)
add_dependencies(lidar_filter_core ${PROJECT_NAME}_gencfg)

# 主节点
add_executable(multi_points src/node.cpp)

target_link_libraries(multi_points
  lidar_filter_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)
add_dependencies(multi_points ${PROJECT_NAME}_gencfg)

# 安装规则
install(TARGETS lidar_filter_core multi_points
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/config_editor_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)