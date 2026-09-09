Autonomous Driving Planning System based on ROS2
https://img.shields.io/badge/ROS2-Humble-blue
https://img.shields.io/badge/C++-17-blue.svg
https://img.shields.io/badge/License-Apache%25202.0-green.svg

项目简介
本项目是一个基于 ROS2 Humble 的模块化自动驾驶局部规划系统，实现了从地图构建、全局路径生成、参考线平滑到局部规划主流程的完整链路。系统采用服务/话题、TF 广播等 ROS2 通信机制，支持直道与 S 弯两种地图场景，并集成了基于 OSQP 的参考线平滑优化算法，为后续路径规划和速度规划提供了可扩展的框架。

功能特性
模块化架构：将系统划分为配置读取、地图创建、全局规划、参考线生成与平滑、车辆状态管理、规划主流程等独立模块，高内聚低耦合。

自定义消息与服务：定义了 PNCMap、Referline 等消息，以及地图服务、全局路径服务等接口。

地图生成：支持直道（STRAIGHT）和 S 弯（STURN）两种地图类型，可发布可视化 MarkerArray 供 RViz 显示。

全局路径规划：基于地图中心线提取全局路径，支持多种规划器扩展（当前提供普通规划器）。

参考线平滑：使用二次规划（QP）方法，结合平滑性、几何相似性、紧凑性代价，并利用 OSQP 求解器高效求解，有效消除路径锯齿和曲率突变。

Frenet 坐标转换：实现笛卡尔坐标与 Frenet 坐标的双向转换，以及匹配点查找、投影点计算、曲率求解等几何工具。

车辆模型与 TF：支持主车与多障碍车的 URDF/Xacro 模型，通过 TF2 广播位姿并实时监听。

参数化配置：所有参数通过 YAML 文件统一管理，便于调参和扩展。

RViz 可视化：提供地图边界、全局路径、参考线等可视化显示。

环境依赖
操作系统：Ubuntu 22.04 LTS

ROS2：Humble Hawksbill

C++ 标准：C++17

依赖库：

Eigen3

OsqpEigen (基于 OSQP)

yaml-cpp

ament_index_cpp

tf2_ros

visualization_msgs, nav_msgs, geometry_msgs, std_msgs

rclcpp

安装与编译
创建 ROS2 工作空间（如果尚未创建）：

bash
mkdir -p ~/autonomous_driving_ws/src
cd ~/autonomous_driving_ws
克隆本仓库到 src 目录：

bash
cd src
git clone https://github.com/HeTaiXin/autonomous_driving_ws.git .
# 或者只克隆 src 下的内容到当前目录，注意仓库结构
注意：本仓库本身就是一个 ROS2 工作空间，建议直接将仓库内容放到 src 下，或直接使用仓库根目录作为工作空间。

安装依赖：

bash
cd ~/autonomous_driving_ws
rosdep install --from-paths src --ignore-src -r -y
此外需安装 Eigen3 和 OsqpEigen：

bash
sudo apt install libeigen3-dev
OsqpEigen 可通过源码编译安装，或参考其官方文档。

编译：

bash
colcon build --symlink-install
设置环境变量：

bash
source install/setup.bash
运行
启动完整系统（包括 RViz、机器人模型、地图服务器、全局路径服务器、规划节点）：

bash
ros2 launch planning planning.launch.py
该 launch 文件会：

启动两个 robot_state_publisher 节点（主车和障碍车）及 joint_state_publisher

启动 RViz2 并加载配置

启动 pnc_map_server、global_path_server、planning_process 节点

启动后，规划节点会按配置请求地图和全局路径，并周期性地发布参考线话题。RViz 中将显示车辆模型、地图、全局路径和参考线。

项目结构
text
autonomous_driving_ws/
├── src/
│   ├── base_msgs/                  # 接口消息定义包
│   │   ├── msg/                    # 自定义消息
│   │   │   ├── PNCMap.msg
│   │   │   ├── Referline.msg
│   │   │   └── ReferlinePoint.msg
│   │   ├── srv/                    # 服务定义
│   │   │   ├── PNCMapService.srv
│   │   │   └── GlobalPathService.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── planning/                   # 规划功能包
│       ├── config/                 # YAML 配置文件
│       │   └── planning_static_obs_config.yaml
│       ├── urdf/                   # 车辆模型
│       │   ├── main_car/
│       │   └── obs_car/
│       ├── launch/                 # 启动文件
│       │   └── planning.launch.py
│       ├── rviz/                   # RViz 配置
│       │   └── planning.rviz
│       └── src/                    # 源代码
│           ├── common/             # 配置读取、数学工具
│           ├── pnc_map_creator/    # 地图创建
│           ├── global_planner/     # 全局路径规划
│           ├── reference_line/     # 参考线生成与平滑
│           ├── vehicle_info/       # 车辆状态管理
│           └── planning_process/   # 规划主流程节点
使用说明
配置参数
所有参数集中位于 src/planning/config/planning_static_obs_config.yaml，可调整：

车辆尺寸、初始位置、速度

地图类型（直道/S弯）、道路长度、车道宽度、分段长度

参考线前后点数

障碍物检测距离等

服务接口
/planning/pnc_map_server（服务类型 base_msgs/srv/PNCMapService）：请求地图，需提供 map_type。

/planning/global_path_server（服务类型 base_msgs/srv/GlobalPathService）：请求全局路径，需提供 global_planner_type 和 pnc_map。

话题
/planning/pnc_map（base_msgs/msg/PNCMap）：发布地图数据。

/planning/pnc_map_markerarray（visualization_msgs/msg/MarkerArray）：用于 RViz 显示地图。

/planning/global_path（nav_msgs/msg/Path）：发布全局路径。

/planning/global_path_rviz（visualization_msgs/msg/Marker）：用于 RViz 显示全局路径。

/planning/reference_line（nav_msgs/msg/Path）：发布参考线（供 RViz 显示）。

主要算法
参考线平滑（QP）
参考线平滑采用二次规划模型，代价函数为：

平滑性：最小化相邻点二阶导（曲率变化）

几何相似性：与原始点偏差最小

紧凑性：相邻点距离均匀

约束条件：首尾点固定，其余点位置偏差限制在 0.2m 内。求解器使用 OSQP，通过 Eigen 构造稀疏矩阵，计算效率高。

Frenet 坐标转换
实现了笛卡尔与 Frenet 坐标系的完整转换，支持匹配点查找、投影点计算、曲率与曲率变化率求解，为后续路径规划提供基础。

未来工作
添加局部路径规划器（如 EM Planner、Lattice Planner）

集成速度规划与轨迹生成

支持更多地图类型（如环形交叉口）

加入障碍物预测与决策模块

许可证
本项目采用 Apache-2.0 许可证。

联系方式
作者：HeTaiXin

邮箱：2396807312@qq.com

GitHub: https://github.com/HeTaiXin
