# Diff-Planner-PX4 (ROS2)

将**微分智飞**公司开源的 **[Diff-Planner](https://github.com/DifferentialRobotics/Diff-Planner)** 适配了 PX4 SITL Gazebo 仿真环境，并迁移至 **ROS2 Humble**。

> 本项目基于 ROS1 版本修改而来，已全面迁移至 ROS2 Humble (Ubuntu 22.04)。  
> 原 ROS1 版本支持 Ubuntu 18.04 ROS Melodic / Ubuntu 20.04 ROS Noetic。

---

## 目录结构

```
├── diff_planner/
│   ├── plan_env/              # 环境建图（占据地图、ESDF）
│   ├── path_searching/        # 路径搜索（A*、拓扑路径）
│   ├── traj_opt/              # 轨迹优化（MINCO、B样条）
│   ├── traj_utils/            # 轨迹工具（可视化、消息定义）
│   ├── plan_manage/           # 状态机、任务调度、轨迹服务
│   ├── swarm_bridge/          # 多机桥接（TCP/UDP 通信）
│   └── drone_detect/          # 无人机视觉检测（深度图）
├── user_command/
│   └── multipoint/            # 多点航点任务
└── Utils/
    ├── uav_utils/             # 工具函数（坐标系转换、几何计算）
    ├── quadrotor_msgs/        # 自定义消息定义
    ├── odom_visualization/    # 里程计可视化
    ├── rviz_plugins/          # RViz 插件
    ├── random_goals/          # 随机目标点分配（多机）
    ├── assign_goals/          # 批量目标点分配
    ├── moving_obstacles/      # 移动障碍物（摇杆控制）
    ├── manual_take_over/      # 人工接管（地面站/摇杆）
    ├── pose_utils/            # 位姿工具
    └── selected_points_publisher/  # 选中点发布
```

---

## 1. 准备

### 环境要求

- **Ubuntu 22.04**
- **ROS2 Humble**
- **PX4 SITL Gazebo** 仿真环境（参考 [PX4 官方文档](https://docs.px4.io/main/en/simulation/gazebo.html)）
- Gazebo Ignition

### 创建工作空间

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

### 依赖安装

```bash
sudo apt install libgoogle-glog-dev libgflags-dev libeigen3-dev libarmadillo-dev
sudo apt install ros-humble-pcl-ros ros-humble-tf2-geometry-msgs ros-humble-laser-geometry ros-humble-tf2-sensor-msgs ros-humble-cv-bridge ros-humble-image-transport
sudo apt install ros-humble-message-filters ros-humble-visualization-msgs ros-humble-nav-msgs
sudo apt install ros-humble-robot-state-publisher ros-humble-xacro
sudo apt install libyaml-cpp-dev
```

### px4_msgs

编译前需要确保 `px4_msgs` 包在 workspace 中。Micro-XRCE-Agent 与 PX4 通信需要 `px4_msgs` 的 ROS2 接口定义：

```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
# 或使用项目中已包含的版本
```

---

## 2. 编译
**下载源码：** 
```bash
cd ~/ros2_ws/src
git clone https://github.com/Tfly6/Diff-Planner-PX4.git
git checkout ros2
```
**编译：** 
```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> 如果遇到编译错误，可先单独编译某个包定位问题：
> ```bash
> colcon build --packages-select <package_name>
> ```

---

## 3. 运行（单机深度相机）

**运行之前必须打开 QGC**

**终端一**：启动 Gazebo 仿真

```bash
cd ${YOUR_PX4_PATH}
make px4_sitl gz_x500_depth
```

**终端二**：启动 Micro-XRCE-Agent（与 PX4 通信）

```bash
MicroXRCEAgent udp4 -p 8888
```

**终端三**：启动 ROS2 controller

```bash
cd ~/ros2_ws/src
git clone https://github.com/Tfly6/px4_se3Ctrl_ros2.git
cd ../
colcon build --symlink-install --packages-select se3_hopf
source ~/ros2_ws/install/setup.bash
ros2 launch se3_hopf se3_hopf.launch.py
```

**终端四**：启动 Diff-Planner

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch diff_planner run_px4_sitl_gazebo.launch.py
```

使用 RViz 中的 **3D Nav Goal** 插件，在地图上选择目标点即可开始规划。

---

## 4. 主要话题

### diff_planner

| 话题 | 方向 | 说明 |
|---|---|---|
| `/goal` | 输入 | 目标点（RViz 3D Nav Goal 或 user_command） |
| `/mandatory_stop_to_planner` | 输入 | 外部急停信号 |
| `/drone_X_visual_slam/odom` | 输入 | 无人机里程计 |
| `/drone_X_planning/pos_cmd` | 输出 | 位置控制指令（PositionCommand） |
| `/broadcast_traj_from_planner` | 输出 | 多机轨迹广播 |
| `/others_odom` | 输入/输出 | 其他无人机里程计（桥接） |
| `/grid_map/occupancy` | 输出 | 占据栅格可视化 |

---

## 5. ROS1 → ROS2 迁移说明

本项目原为 ROS1 工程，已迁移至 ROS2 Humble。主要变更：

| 项目 | ROS1 | ROS2 |
|---|---|---|
| 构建系统 | `catkin` | `colcon` + `ament_cmake` |
| 构建命令 | `catkin build` | `colcon build` |
| 消息类型 | `nav_msgs::Odometry` | `nav_msgs::msg::Odometry` |
| 节点初始化 | `ros::init` / `ros::NodeHandle` | `rclcpp::init` / `rclcpp::Node` |
| 参数 | `nh.getParam(...)` | `declare_parameter` / `get_parameter` |
| 时间 | `ros::Time::now()` | `node->now()` |
| 日志 | `ROS_INFO(...)` | `RCLCPP_INFO(...)` |
| Launch 文件 | XML `.launch` | Python `.launch.py` |
| TF | `tf::TransformBroadcaster` | `tf2_ros::TransformBroadcaster` |
| PX4 通信 | MAVROS | Micro-XRCE-Agent + `px4_msgs` |
| CMake | `find_package(catkin)` | `find_package(ament_cmake)` |
| 依赖声明 | `<build_depend>` / `<run_depend>` | `<depend>` (format 3) |

---

## 参考

- [Diff-Planner (DifferentialRobotics)](https://github.com/DifferentialRobotics/Diff-Planner)
- [EGO-Planner-v2 (ZJU-FAST-Lab)](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)
- [px4_msgs (PX4)](https://github.com/PX4/px4_msgs)
- [Tfly6/OpenDrone - PX4 and ROS1 SITL](https://github.com/Tfly6/OpenDrone)
- [Tfly6/px4_se3Ctrl_ros2](https://github.com/Tfly6/px4_se3Ctrl_ros2)
