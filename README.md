# Diff-Planner-PX4

将**微分智飞**公司开源的 **[Diff-Planner](https://github.com/DifferentialRobotics/Diff-Planner)** （由 [03563ad](https://github.com/DifferentialRobotics/Diff-Planner/commit/03563adae7315cf3db35494bfac9903093ef5663) commit修改而来）适配了PX4 SITL Gazebo 仿真环境

- `diff_planner/`
  核心导航避障算法本体。包含环境建图、路径搜索、轨迹优化、任务状态机、轨迹发布与多机桥接等主流程模块（如 `plan_env`、`path_searching`、`traj_opt`、`plan_manage`、`swarm_bridge`）。**Diff-Planner** 是为**微分智飞**公司旗下教育无人机子品牌**非凸空间**适配的单机导航避障算法。其基于开源算法 **[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)** ，并由原班人马深度参与算法优化。在继承 **EGO-Planner** 优秀框架的基础上，针对教育无人机平台的特殊需求进行了全面适配和增强，旨在提供更稳定、更可靠的科研体验。
- `se3_controller/`
  飞行控制器侧（SE(3) 控制）。负责把期望轨迹/姿态转换成可执行的控制量（姿态、角速度、推力），主要用于 PX4/MAVROS 场景下的轨迹跟踪与控制。参考了[HITSZ-MAS/se3_controller](https://github.com/HITSZ-MAS/se3_controller) 项目。
- `user_command/`
  用户命令层。用于把“人给的任务”变成规划器可用的目标序列，比如多点航点任务、返航点触发、预设任务流等（当前主要是 `multipoint`，此仿真中默认不用）。
- `Utils/`
  通用功能包集合。放与主算法解耦但运行必需/常用的辅助组件，比如可视化（`odom_visualization`、`rviz_plugins`）、消息定义（`quadrotor_msgs`）、工具函数（`uav_utils`）、人工接管/移动障碍等。

>支持 Ubuntu 18.04 ROS Melodic、Ubuntu 20.04 ROS Noetic

## 1. 准备

- **使用之前必须搭建** [PX4无人机仿真环境](https://blog.csdn.net/weixin_55944949/article/details/130895608?spm=1001.2014.3001.5501)
- **创建工作空间** 没有创建工作空间，可以执行下列代码，如果创建了可以跳过

```bash
sudo apt-get install python-catkin-tools python-rosinstall-generator -y

# For Ros Noetic use that:
# sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
```

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws && catkin init # 初始化工作空间
catkin build
```

- **依赖**

```bash
sudo apt install libgoogle-glog-dev libgflags-dev libeigen3-dev libarmadillo-dev
sudo apt install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-tf2-geometry-msgs ros-$ROS_DISTRO-laser-geometry ros-$ROS_DISTRO-tf2-sensor-msgs
```

## 2. 编译

```bash
cd ~/catkin_ws/src
git clone https://github.com/Tfly6/Diff-Planner-PX4.git
cd ~/catkin_ws
catkin build
```

## 3. 运行（单机）

- 配置仿真

```bash
# model
# PX4 < v1.14
cp -r ~/catkin_ws/src/Diff-Planner-PX4/sitl_config/models/* ${YOUR_PX4_PATH}/Tools/sitl_gazebo/models/
cp ~/catkin_ws/src/Diff-Planner-PX4/sitl_config/worlds/* ${YOUR_PX4_PATH}/Tools/sitl_gazebo/worlds/

# PX4 >= v1.14
cp -r ~/catkin_ws/src/Diff-Planner-PX4/sitl_config/models/* ${YOUR_PX4_PATH}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
cp ~/catkin_ws/src/Diff-Planner-PX4/sitl_config/worlds/* ${YOUR_PX4_PATH}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
```

```bash
# launch
cp ~/catkin_ws/src/Diff-Planner-PX4/sitl_config/outdoor_depth_camera.launch ${YOUR_PX4_PATH}/launch/
cp ~/catkin_ws/src/Diff-Planner-PX4/sitl_config/outdoor_mid360.launch ${YOUR_PX4_PATH}/launch/
cp ~/catkin_ws/src/Diff-Planner-PX4/sitl_config/px4_config.yaml ${YOUR_PX4_PATH}/launch/

```



### 实例一：深度相机

- 终端一：启动gazebo仿真

```bash
roslaunch px4 outdoor_depth_camera.launch # 用自己的也行
```

- 终端二：启动 se3_controller (启动之后，默认会自动进入offboard模式，解锁，然后起飞 2m)

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch se3_controller sitl_se3_controller.launch
```

- 终端三：启动 diff_planner

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch diff_planner run_px4_sitl_gazebo.launch
```

使用rviz中的**3D Nav Goal**插件，在地图上按住左键选择目标点x-y平面位置，按住左键不松手同时按住右键上下拖动调整目标点z轴位置（z轴最好大于 1m），之后松开鼠标即发送目标点，无人机开始规划。

### 实例二：3D激光雷达（Mid360）

- 根据下面仓库配置Mid360仿真 👇

[Tfly6/Mid360_px4_sim_plugin: Plugin for the simulation of the Livox Mid-360 in Gazebo](https://github.com/Tfly6/Mid360_px4_sim_plugin)

- 终端一：启动gazebo仿真

```bash
roslaunch px4 outdoor_mid360.launch # 用自己的也行
```

- 终端二：启动 se3_controller (启动之后，默认会自动进入offboard模式，解锁，然后起飞 2m)

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch se3_controller sitl_se3_controller.launch
```

- 终端三：启动 diff-planner

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch diff_planner run_px4_sitl_gazebo_mid360.launch
```

使用rviz中的**3D Nav Goal**插件，在地图上按住左键选择目标点x-y平面位置，按住左键不松手同时按住右键上下拖动调整目标点z轴位置（z轴最好大于 1m），之后松开鼠标即发送目标点，无人机开始规划。

[Diff-Planner(ego-plannerV2 升级版)+PX4+激光雷达/深度相机 Gazebo 仿真_bilibili](https://www.bilibili.com/video/BV1MRPxzjEPS/?spm_id_from=333.1387.homepage.video_card.click&vd_source=d59e7d5891b69289e548bcfb7a4948a0)

## 4. 主要订阅和发布的话题

**diff_planner**

- **输入话题（主链路）**

  - /goal

    - 作用：给规划器发送目标点（rviz中的**3D Nav Goal**或者user_command中输出的）。

  - /traj_start_trigger
    

    - 作用：触发按预设 waypoint 开始任务（flight_type=2 常见）。

  - /mandatory_stop_to_planner
    

    - 作用：外部急停信号，强制规划器停。

  - /drone\_<id>_<odometry_topic> 或直接odom_topic（由use_drone_topic_prefix决定）
  
    - 作用：无人机当前里程计，规划状态机和地图更新都依赖它。

  - /drone\_<id>\_<depth_topic>、/drone\_<id>\_<camera_pose_topic>、/drone\_<id>_<cloud_topic>（或无前缀）
      - 作用：环境感知输入（深度图/相机位姿/点云），用于构建占据地图。

- **输出话题（主链路）**

  - /drone\_<id>_planning/trajectory
    - 作用：优化后的轨迹（给 `traj_server` 使用）。
  
- /drone\_<id>_planning/data_display
    - 作用：规划调试可视化数据。
    
  - /broadcast_traj_from_planner
    - 作用：本机轨迹广播给桥接层（多机避碰/协同）。
  
- /drone\_<id>_traj_server/heartbeat
    - 作用：轨迹服务心跳，监控节点可据此检测异常。
    
  - cmd_topic（默认/drone\_<id>_planning/pos_cmd）
    - 作用：`traj_server` 输出的位置指令（PositionCommand），供控制器消费。
  
- /command/trajectory（脚本trajectory_msg_converter.py输出）
    - 作用：把 PositionCommand 转成 MultiDOF 轨迹，便于 `se3_controller` 订阅。

- **可选/辅助（你现在也在用）**

  - grid_map/occupancy、grid_map/occupancy_inflate
    - 作用：占据栅格可视化。
    
  - /broadcast_traj_to_planner（来自桥接）
    - 作用：接收其他无人机轨迹用于动态避让。
    
  - /others_odom（桥接/检测链路）
    - 作用：其他无人机里程计聚合输入（例如 `drone_detect`）。

------

**se3_controller**

- **输入话题**

  - /mavros/local_position/odom
    - 作用：当前位姿/速度反馈。
  - /mavros/imu/data
    - 作用：角速度和加速度反馈。
  - /mavros/state
    - 作用：飞控连接、模式、解锁状态（FSM 切换依赖）。
  - /command/trajectory
    - 作用：期望轨迹输入（来自 `diff_planner` 转换脚本）。
- **输出话题**

  - /mavros/setpoint_raw/attitude
    - 作用：姿态+角速度+推力控制指令（核心控制输出）。
    
  - /mavros/setpoint_position/local
    - 作用：等待/起飞阶段的位置 setpoint。
    
  - /desire_odom_pub
    - 作用：发布当前控制器内部期望状态，便于调试可视化。
- **服务接口（不是话题，但实际在流程里很关键）**

  - /mavros/set_mode、/mavros/cmd/arming
    - 作用：切 OFFBOARD / 解锁。
    
  - /land
    - 作用：外部触发降落流程。

------

**user_command（multipoint）**

- **输入话题**

  - odom_topic（launch remap，仿真默认/visual_slam/odom，你可改为/mavros/local_position/odom）
    - 作用：用于判断“是否到达当前航点、是否切下一个点”。
    
  - /move_base_simple/goal
  
    - 作用：手动启动任务或给起始目标。
  
  - /back_trigger
    - 作用：触发返航任务。
    
  - /mavros/rc/in
    - 作用：遥控器通道触发起飞/降落逻辑。
  
- **输出话题**

  - /goal

    - 作用：发送给 `diff_planner` 的目标点（主入口之一）。

  - /planning/yaw
    
    - 作用：单独发送 yaw 参考给轨迹执行侧。

  - /px4ctrl/takeoff_land
    
    - 作用：向控制层发送起飞/降落命令。
  
  - /move_base_simple/goal、/back_trigger

    （自身也会发布）

    - 作用：用于任务触发链路（自触发/转发场景）。

## 参考

[Tfly6/OpenDrone: PX4 and ROS1 SITL](https://github.com/Tfly6/OpenDrone)

[DifferentialRobotics/Diff-Planner](https://github.com/DifferentialRobotics/Diff-Planner)

[HITSZ-MAS/se3_controller](https://github.com/HITSZ-MAS/se3_controller) 

[Tfly6/Mid360_px4_sim_plugin: Plugin for the simulation of the Livox Mid-360 in Gazebo](https://github.com/Tfly6/Mid360_px4_sim_plugin)
