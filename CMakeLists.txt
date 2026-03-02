cmake_minimum_required(VERSION 3.0.2)
project(lidar_filtering)

# 强制 PCL 使用 std::shared_ptr 而不是 boost::shared_ptr
add_definitions(-DPCL_NO_PRECOMPILE)
add_definitions(-DBOOST_ALL_NO_LIB)
# ------------------

add_compile_options(-std=c++14)

# set_pcl
set(PCL_DIR "/usr/include/pcl-1.10") 

# 添加组件 filters 和 segmentation 
find_package(PCL 1.10 EXACT REQUIRED COMPONENTS common io filters segmentation sample_consensus kdtree search)

# 包含 PCL 的头文件和库
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
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