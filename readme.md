# Lidar Filtering ROS Package

![ROS1](https://img.shields.io/badge/ROS-Noetic/Melodic-blue) ![PCL](https://img.shields.io/badge/Library-PCL-green) ![License](https://img.shields.io/badge/License-MIT-yellow)

这是一个专门用于激光雷达点云预处理的 ROS1 功能包。它集成了实时裁剪、动态参数配置以及多雷达数据管理的框架。

## 1. 项目简介
本项目旨在解决移动机器人点云处理中的基础问题：
* **自身遮挡过滤**：通过 `vehicle_size.yaml` 定义的尺寸，自动剔除机器人本体产生的干扰点云。
* **动态参数调节**：利用 `dynamic_reconfigure` 在不重启节点的情况下，实时调整滤波阈值。
* **模块化设计**：核心滤波逻辑与 ROS 节点逻辑解耦，便于算法快速迭代。

## 2. 目录结构
```text
.
├── cfg/                # 动态调参配置文件 (.cfg)
├── include/            # C++ 头文件 (算法类定义)
├── launch/             # 启动文件 (一键启动节点与配置)
├── params/             # 外部参数配置文件 (车辆几何尺寸)
├── src/                # 核心源代码 (node.cpp & filter_core.cpp)
├── CMakeLists.txt      # 编译脚本
└── package.xml         # ROS 包元数据与依赖声明

## 3. 安装依赖

在编译前，请确保安装了以下核心依赖：

sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-pcl-ros \
                     ros-$ROS_DISTRO-pcl-conversions \
                     ros-$ROS_DISTRO-dynamic-reconfigure

pcl-version: PCL-1.10

## 4. 编译与运行指南

### 4.1 编译

建议使用 colcon build：

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-pcl-ros \
                     ros-$ROS_DISTRO-pcl-conversions \
                     ros-$ROS_DISTRO-dynamic-reconfigure

### 4.2 运行

启动滤波主节点：

```bash
roslaunch lidar_filtering lidar_filtering.launch

### 4.3 动态参数调试

在节点运行过程中，可以使用可视化界面实时调整参数：

```bash
rosrun rqt_reconfigure rqt_reconfigure

## 5. 参数说明 (params/vehicle_size.yaml)

本项目通过 YAML 文件定义过滤区域（单位：米）：

参数名称,类型,说明
min_x,double,车辆坐标系下，向前的最小裁剪距离
max_x,double,车辆坐标系下，向前的最大裁剪距离
min_y,double,车辆坐标系下，向左的最小裁剪距离
max_y,double,车辆坐标系下，向左的最大裁剪距离

## 6. 开发状态记录 (Roadmap)

    [x] v1.0: 基础滤波框架构建，支持 PCL 裁剪。

    [x] v1.1: 集成 Dynamic Reconfigure 动态调参。

    [x] Current: 暂时屏蔽两路单线雷达输入（待逻辑优化后恢复）。

    [ ] Next: 增加点云下采样 (Voxel Grid Filter) 功能。

    [ ] Next: 增加统计学离群点去除 (SOR Filter)。

## 7. 维护者

    Pro-qing - [GitHub Profile](https://github.com/Pro-qing)
    