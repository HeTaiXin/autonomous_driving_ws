# 基于 ROS2 的自动驾驶规划系统

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue" alt="ROS2">
  <img src="https://img.shields.io/badge/C++-17-blue.svg" alt="C++17">
  <img src="https://img.shields.io/badge/License-Apache%202.0-green.svg" alt="License">
  <img src="https://img.shields.io/badge/Platform-Ubuntu%2022.04-orange" alt="Platform">
</p>

## 一、项目简介

本项目是一个基于 **ROS2 Humble** 的模块化自动驾驶**规划（Planning）系统**，参考 Apollo 规划模块的架构思想，实现了从 **地图构建 → 全局路径规划 → 参考线平滑 → Frenet 坐标转换 → 规划主流程编排** 的完整链路。

系统采用 **服务 / 话题 / TF 广播** 等 ROS2 通信机制，支持 **直道** 与 **S 弯** 两种地图场景，并集成了基于 **OSQP-Eigen** 的参考线二次规划（QP）平滑算法，为后续局部路径规划与速度规划提供了可扩展的框架。

> 🎯 **定位**：作为自动驾驶算法岗位的学习与展示项目，覆盖规划算法核心（Frenet 坐标、QP 优化、参考线平滑）与 ROS2 工程化能力（自定义 msg/srv、Launch、TF、URDF、RViz）。

---

## 二、功能特性

- **模块化架构**：划分为配置读取、地图创建、全局规划、参考线与平滑、车辆状态管理、规划主流程等独立模块，高内聚、低耦合。
- **自定义消息与服务**：定义 `PNCMap`、`Referline`、`ReferlinePoint` 等消息，以及 `PNCMapService`、`GlobalPathService` 等服务接口。
- **地图生成**：采用**工厂模式**支持直道（`STRAIGHT`）与 S 弯（`STURN`）两种地图类型，可发布 `MarkerArray` 供 RViz 显示。
- **全局路径规划**：基于地图中心线 / 边界提取全局路径，支持多种规划器扩展。
- **参考线平滑**：使用**二次规划（QP）**方法，结合平滑性、几何相似性、紧凑性代价，利用 **OSQP** 求解器高效求解，消除路径锯齿与曲率突变。
- **Frenet 坐标转换**：实现笛卡尔与 Frenet 坐标的双向转换，以及匹配点查找、投影点计算、曲率求解等几何工具。
- **车辆模型与 TF**：支持主车与多障碍车的 URDF/Xacro 模型，通过 `tf2_ros` 广播位姿并实时监听。
- **参数化配置**：所有参数通过 YAML 文件统一管理，便于调参和扩展。
- **RViz 可视化**：提供地图边界、全局路径、参考线、车辆模型的可视化显示。

---

## 三、系统架构

### 3.1 整体数据流

```
PNCMapServer ──(PNCMap)──┐
                          ├── GlobalPathServer ──(nav_msgs/Path)──┐
                          │                                        ├── PlanningProcess
                          │                                        │   ├── ReferenceLineCreator
                          │                                        │   │   ├── Curve (匹配/投影)
                          │                                        │   │   └── ReferenceLineSmoother (QP平滑)
                          │                                        │   ├── MainCarInfo (主车状态 + Frenet转换)
                          │                                        │   └── ObsCarInfo (障碍物状态)
                          │                                        │
RViz ◄── MarkerArray ─────┘                                        │
RViz ◄── Marker (全局路径) ─────────────────────────────────────────┘
RViz ◄── Path (参考线) ─────────────────────────────────────────────┘
```

### 3.2 通信机制说明

| 机制 | 用途 | 本项目实例 |
|------|------|-----------|
| **话题（Topic）** | 异步、单向、高频数据流 | 地图数据、全局路径、参考线发布 |
| **服务（Service）** | 同步、双向、按需请求-响应 | 获取地图、获取全局路径 |
| **接口（msg/srv）** | 定义数据结构契约 | `PNCMap.msg`、`ReferlinePoint.msg`、`PNCMapService.srv` |
| **TF 广播** | 坐标系变换管理 | 主车 / 障碍车位姿发布与监听 |

> 💡 **设计考量**：地图与全局路径属于**低频、按需**数据，采用 Service（请求-响应）而非 Topic，避免无效带宽占用；车辆状态、规划轨迹属于**高频、连续**数据，采用 Topic 发布。

---

## 四、项目结构

```
autonomous_driving_ws/
├── src/
│   ├── base_msgs/                    # 自定义接口包
│   │   ├── msg/
│   │   │   ├── PNCMap.msg            # 规划地图（中心线、左右边界）
│   │   │   ├── Referline.msg         # 参考线（点序列）
│   │   │   └── ReferlinePoint.msg    # 参考线点（位姿 + Frenet 参数）
│   │   ├── srv/
│   │   │   ├── PNCMapService.srv     # 地图服务（请求 map_type → 响应 PNCMap）
│   │   │   └── GlobalPathService.srv # 全局路径服务（请求类型 + 地图 → 响应 Path）
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── planning/                     # 规划功能包
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       │   └── planning_static_obs_config.yaml  # 参数配置（车辆/地图/路径/参考线）
│       ├── urdf/                     # 车辆 URDF/Xacro 模型
│       │   ├── main_car/             # 主车（底盘 + 车轮 + 摄像头 + 雷达）
│       │   └── obs_car/             # 障碍车
│       ├── launch/
│       │   └── planning.launch.py    # 启动文件
│       ├── rviz/
│       │   └── planning.rviz         # RViz 可视化配置
│       └── src/
│           ├── common/               # 公共模块
│           │   ├── config_reader/    # YAML 配置读取
│           │   └── math/curve.*      # Frenet 转换、匹配/投影、曲率计算
│           ├── pnc_map_creator/      # 地图创建（工厂模式）
│           │   ├── pnc_map_server.*  # 地图服务节点
│           │   ├── pnc_map_creator_base.h
│           │   ├── pnc_map_straight/ # 直道地图
│           │   └── pnc_map_sturn/    # S 弯地图
│           ├── global_planner/       # 全局路径规划
│           │   ├── global_path_server.*  # 全局路径服务节点
│           │   ├── global_planner_base.h
│           │   └── global_planner_normal/ # 普通规划器
│           ├── reference_line/       # 参考线生成与平滑
│           │   ├── reference_line_creator.*  # 截取参考线片段
│           │   └── reference_line_smoother.* # OSQP 平滑优化
│           ├── vehicle_info/         # 车辆状态管理
│           │   ├── vehicle_info_base.h
│           │   ├── main_car/         # 主车（含 Frenet 转换）
│           │   └── obs_car/          # 障碍物车
│           └── planning_process/     # 规划主流程节点
│                   └── planning_process.*
├── .gitignore
└── README.md
```

---

## 五、环境依赖

| 组件 | 版本 / 说明 |
|------|------------|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill |
| C++ 标准 | C++17 |
| Python | 3.10+（用于 Launch / 脚本） |

**依赖库：**

- **Eigen3**：线性代数运算（参考线平滑矩阵构造）
- **OsqpEigen**：二次规划求解器（OSQP 的 Eigen 封装）
- **yaml-cpp**：YAML 配置文件解析
- **ament_index_cpp**：ROS2 包路径查询
- **tf2_ros**：坐标变换广播与监听
- **rclcpp**：ROS2 C++ 客户端库
- **visualization_msgs / nav_msgs / geometry_msgs / std_msgs**：标准消息类型

---

## 六、安装与编译

### 6.1 创建 ROS2 工作空间

```bash
mkdir -p ~/autonomous_driving_ws/src
cd ~/autonomous_driving_ws
```

### 6.2 克隆本仓库

```bash
cd ~/autonomous_driving_ws/src
git clone https://github.com/HeTaiXin/autonomous_driving_ws.git .
```

> 📌 本仓库本身即为一个 ROS2 工作空间，直接将 `src/` 内容置于工作空间 `src/` 下即可。

### 6.3 安装系统依赖

```bash
sudo apt update
sudo apt install -y libeigen3-dev

# OsqpEigen 通过源码编译安装（参考官方文档）
# https://github.com/robotology/osqp-eigen
```

### 6.4 安装 ROS2 依赖

```bash
cd ~/autonomous_driving_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 6.5 编译

```bash
cd ~/autonomous_driving_ws
colcon build --symlink-install
```

### 6.6 设置环境变量

```bash
source install/setup.bash
```

> 💡 建议将 `source` 命令加入 `~/.bashrc`，避免每次新终端重新设置。

---

## 七、运行

### 7.1 启动完整系统

```bash
ros2 launch planning planning.launch.py
```

该 Launch 文件会：

1. 启动 **主车** 与 **障碍车** 的 `robot_state_publisher` 及 `joint_state_publisher`
2. 启动 **RViz2** 并加载可视化配置
3. 启动 `pnc_map_server`（地图服务）、`global_path_server`（全局路径服务）、`planning_process`（规划主流程）
4. 规划节点按配置请求地图与全局路径，周期性发布参考线话题

启动后，RViz 中将显示：**车辆模型、地图边界、全局路径、参考线**。

### 7.2 各节点说明

| 节点 | 类型 | 作用 |
|------|------|------|
| `pnc_map_server` | 服务 + 发布者 | 按 `map_type` 生成地图，发布 `PNCMap` 与 `MarkerArray` |
| `global_path_server` | 服务 + 发布者 | 从 PNCMap 提取全局路径，发布 `Path` 与 `Marker` |
| `planning_process` | 客户端 + 主循环 | 编排整体流程，定时生成参考线 |

### 7.3 话题与服务列表

**服务接口：**

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/planning/pnc_map_server` | `base_msgs/srv/PNCMapService` | 请求地图，提供 `map_type` |
| `/planning/global_path_server` | `base_msgs/srv/GlobalPathService` | 请求全局路径，提供 `global_planner_type` + `pnc_map` |

**话题列表：**

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/planning/pnc_map` | `base_msgs/msg/PNCMap` | 地图数据 |
| `/planning/pnc_map_markerarray` | `visualization_msgs/msg/MarkerArray` | RViz 地图显示 |
| `/planning/global_path` | `nav_msgs/msg/Path` | 全局路径 |
| `/planning/global_path_rviz` | `visualization_msgs/msg/Marker` | RViz 全局路径显示 |
| `/planning/reference_line` | `nav_msgs/msg/Path` | 参考线（供 RViz 显示） |

---

## 八、参数配置

所有参数集中位于 `src/planning/config/planning_static_obs_config.yaml`，可按需调整：

```yaml
vehicle:                      # 车辆
  main_car:                   # 主车
    id: 0
    frame: "base_footprint"
    length: 3.0
    width: 1.5
    pose_x: 0.0
    pose_y: 0.0
    pose_theta: 0.0
    speed_ori: 1.0
  obs_car1:                   # 障碍物 1
    id: 1
    frame: "base_footprint_obs1"
    length: 3.0
    width: 1.6
    pose_x: 60.0
    pose_y: 0.0
    pose_theta: 0.0
    speed_ori: 0.0
  # obs_car2、obs_car3 ...

pnc_map:                      # PNC 地图
  frame: "map"
  type: 1                     # 0: 直道, 1: S 弯
  road_length: 250.0
  road_half_width: 4.0
  segment_len: 0.5
  speed_limit: 1.0

global_path:                  # 全局路径
  type: 0                     # 0: 普通规划器

reference_line:               # 参考线
  type: 0
  front_size: 200             # 前方点数
  back_size: 80               # 后方点数

planning_process:             # 规划主流程
  obs_dis: 100.0              # 障碍物检测距离
```

---

## 九、核心算法

### 9.1 参考线平滑（二次规划 QP）

参考线平滑采用二次规划模型，代价函数为：

$$
J = w_1 \cdot J_{smooth} + w_2 \cdot J_{similarity} + w_3 \cdot J_{compact}
$$

- **平滑性**：最小化相邻点二阶导（曲率变化），权重 $w_1 = 100$
- **几何相似性**：与原始点偏差最小，权重 $w_2 = 10$
- **紧凑性**：相邻点距离均匀，权重 $w_3 = 1$

**约束条件**：首尾点固定，其余点位置偏差限制在 **0.2m** 内。

求解器使用 **OSQP**，通过 **Eigen** 构造稀疏 Hessian 矩阵（$2n \times 2n$ 分块结构），计算效率高。

> 📌 详见 `reference_line/reference_line_smoother.cpp`

### 9.2 Frenet 坐标转换

实现笛卡尔（Cartesian）与 Frenet 坐标系的完整双向转换：

- **匹配点查找**：在全局路径 / 参考线上基于欧氏距离找最近点
- **投影点计算**：简化为匹配点近似（前提：参考点足够密且平滑）
- **坐标转换**：
  - `cartesian_to_frenet`：输入目标点参数 + 投影点参数 → 输出 $(s, l, ds/dt, dl/ds, d^2l/ds^2, ...)$
  - `frenet_to_cartesian`：反向转换
- **参数计算**：为参考线各点计算累积弧长 $r_s$、航向角 $r_\theta$、曲率 $r_\kappa$、曲率变化率 $r_{dkappa}$（差分近似）

> 📌 详见 `common/math/curve.h`、`curve.cpp`

---

## 十、扩展与未来工作

- [ ] 添加局部路径规划器（如 EM Planner、Lattice Planner）
- [ ] 集成速度规划与轨迹生成
- [ ] 完善障碍物 Frenet 投影（`ObsCarInfo::vehicle_cartesian_to_frenet`）
- [ ] 支持更多地图类型（环形交叉口、弯道组合）
- [ ] 加入障碍物预测与决策模块
- [ ] 增加单元测试与 CI/CD（GitHub Actions）

---

## 十一、许可证

本项目采用 **Apache License 2.0** 许可证。

---

## 十二、联系方式

- **作者**：HeTaiXin
- **邮箱**：2396807312@qq.com
- **GitHub**：https://github.com/HeTaiXin
- **项目地址**：https://github.com/HeTaiXin/autonomous_driving_ws

---

> ⭐ 如果这个项目对你有帮助，欢迎 Star 支持！
