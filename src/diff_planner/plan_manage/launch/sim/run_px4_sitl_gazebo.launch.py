from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def resolved_topic(drone_id, topic, use_prefix):
    return PythonExpression([
        "'/drone_' + str(",
        drone_id,
        ") + '_' + '",
        topic,
        "'.lstrip('/') if '",
        use_prefix,
        "' == 'true' else '",
        topic,
        "'",
    ])


def planner_node_topic(drone_id, topic):
    return PythonExpression([
        "'/drone_' + str(",
        drone_id,
        ") + '_diff_planner_node/' + '",
        topic,
        "'.lstrip('/')",
    ])


def planner_params():
    return {
        'fsm.thresh_replan_time': 1.0,
        'fsm.emergency_time': 1.0,
        'fsm.realworld_experiment': True,
        'fsm.fail_safe': True,
        'fsm.mondify_final_goal': True,
        'fsm.enable_stuck_detect': True,
        'grid_map.resolution': 0.1,
        'grid_map.local_update_range_x': 5.5,
        'grid_map.local_update_range_y': 5.5,
        'grid_map.local_update_range_z': 2.0,
        'grid_map.obstacles_inflation': 0.1,
        'grid_map.local_map_margin': 10,
        'grid_map.enable_virtual_wall': False,
        'grid_map.virtual_ceil': 3.0,
        'grid_map.virtual_ground': -0.1,
        'grid_map.ground_height': -0.01,
        'grid_map.cx': 320.0,
        'grid_map.cy': 240.0,
        # StereoOV7251 depth camera intrinsics from gz_x500_depth /camera_info
        'grid_map.fx': 432.496042035043,
        'grid_map.fy': 432.496042035043,
        'grid_map.use_depth_filter': True,
        'grid_map.depth_filter_tolerance': 0.15,
        'grid_map.depth_filter_maxdist': 5.0,
        'grid_map.depth_filter_mindist': 0.2,
        'grid_map.depth_filter_margin': 2,
        'grid_map.k_depth_scaling_factor': 1000.0,
        'grid_map.skip_pixel': 2,
        'grid_map.p_hit': 0.65,
        'grid_map.p_miss': 0.35,
        'grid_map.p_min': 0.12,
        'grid_map.p_max': 0.90,
        'grid_map.p_occ': 0.80,
        'grid_map.fading_time': 1000.0,
        'grid_map.min_ray_length': 0.1,
        'grid_map.max_ray_length': 4.5,
        'grid_map.visualization_truncate_height': 1.9,
        'grid_map.show_occ_time': False,
        'grid_map.frame_id': 'world',
        'grid_map.odom_depth_timeout': 2.0,
        'manager.polyTraj_piece_length': 1.5,
        'manager.feasibility_tolerance': 0.05,
        'optimization.constraint_points_perPiece': 5,
        'optimization.weight_obstacle': 10000.0,
        'optimization.weight_obstacle_soft': 5000.0,
        'optimization.weight_swarm': 10000.0,
        'optimization.weight_feasibility': 10000.0,
        'optimization.weight_sqrvariance': 10000.0,
        'optimization.weight_time': 10.0,
        'optimization.obstacle_clearance': 0.1,
        'optimization.obstacle_clearance_soft': 0.5,
        'optimization.swarm_clearance': 0.15000000000000002,
        'optimization.vel_tolerance': 1.0,
        'optimization.acc_tolerance': 1.0,
        'optimization.record_opt': True,
        'traj_server.time_forward': 1.0,
    }


def generate_launch_description():
    pkg_share = FindPackageShare('diff_planner')

    drone_id = LaunchConfiguration('drone_id')
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    odom_topic = LaunchConfiguration('odom_topic')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    cmd_topic = LaunchConfiguration('cmd_topic')
    pose_type = LaunchConfiguration('pose_type')
    flight_type = LaunchConfiguration('flight_type')
    point_num = LaunchConfiguration('point_num')
    use_drone_topic_prefix = LaunchConfiguration('use_drone_topic_prefix')
    px4_vehicle_odom_topic = LaunchConfiguration('px4_vehicle_odom_topic')
    camera_extrinsic_topic = LaunchConfiguration('camera_extrinsic_topic')
    gz_depth_topic = LaunchConfiguration('gz_depth_topic')
    gz_camera_info_topic = LaunchConfiguration('gz_camera_info_topic')

    resolved_odom_topic = resolved_topic(drone_id, odom_topic, use_drone_topic_prefix)
    resolved_cloud_topic = resolved_topic(drone_id, cloud_topic, use_drone_topic_prefix)
    resolved_camera_pose_topic = resolved_topic(drone_id, camera_pose_topic, use_drone_topic_prefix)
    resolved_depth_topic = resolved_topic(drone_id, depth_topic, use_drone_topic_prefix)

    common_params = planner_params()

    planner_node_params = {
        **common_params,
        # The planner/traj_server timing chain relies heavily on node->now().
        # In this PX4+GZ setup the ROS clock can remain at zero for these C++ nodes,
        # which leaves PolyTraj.start_time and heartbeat timing stuck at 0.
        # Keep sensor bridge nodes on sim time, but let the planner use wall time.
        'use_sim_time': False,
        'fsm.flight_type': flight_type,
        'fsm.planning_horizon': 7.5,
        'fsm.waypoint_num': point_num,
        'fsm.waypoint0_x': LaunchConfiguration('target0_x'),
        'fsm.waypoint0_y': LaunchConfiguration('target0_y'),
        'fsm.waypoint0_z': LaunchConfiguration('target0_z'),
        'fsm.waypoint1_x': LaunchConfiguration('target1_x'),
        'fsm.waypoint1_y': LaunchConfiguration('target1_y'),
        'fsm.waypoint1_z': LaunchConfiguration('target1_z'),
        'fsm.waypoint2_x': LaunchConfiguration('target2_x'),
        'fsm.waypoint2_y': LaunchConfiguration('target2_y'),
        'fsm.waypoint2_z': LaunchConfiguration('target2_z'),
        'fsm.waypoint3_x': LaunchConfiguration('target3_x'),
        'fsm.waypoint3_y': LaunchConfiguration('target3_y'),
        'fsm.waypoint3_z': LaunchConfiguration('target3_z'),
        'fsm.waypoint4_x': LaunchConfiguration('target4_x'),
        'fsm.waypoint4_y': LaunchConfiguration('target4_y'),
        'fsm.waypoint4_z': LaunchConfiguration('target4_z'),
        'grid_map.map_size_x': map_size_x,
        'grid_map.map_size_y': map_size_y,
        'grid_map.map_size_z': map_size_z,
        'grid_map.pose_type': pose_type,
        'grid_map.extrinsic_topic': camera_extrinsic_topic,
        'manager.max_vel': 1.5,
        'manager.max_acc': 6.0,
        'manager.planning_horizon': 7.5,
        'manager.use_multitopology_trajs': False,
        'manager.drone_id': drone_id,
        'optimization.max_vel': 1.5,
        'optimization.max_acc': 6.0,
        'optimization.max_jer': 20.0,
    }

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='0'),
        DeclareLaunchArgument('map_size_x', default_value='200.0'),
        DeclareLaunchArgument('map_size_y', default_value='200.0'),
        DeclareLaunchArgument('map_size_z', default_value='5.0'),
        DeclareLaunchArgument('init_x', default_value='-15.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='1.0'),
        DeclareLaunchArgument('pose_type', default_value='1'),
        DeclareLaunchArgument('flight_type', default_value='1'),
        DeclareLaunchArgument('point_num', default_value='1'),
        DeclareLaunchArgument('target0_x', default_value='0.0'),
        DeclareLaunchArgument('target0_y', default_value='0.0'),
        DeclareLaunchArgument('target0_z', default_value='0.0'),
        DeclareLaunchArgument('target1_x', default_value='0.0'),
        DeclareLaunchArgument('target1_y', default_value='0.0'),
        DeclareLaunchArgument('target1_z', default_value='0.0'),
        DeclareLaunchArgument('target2_x', default_value='0.0'),
        DeclareLaunchArgument('target2_y', default_value='0.0'),
        DeclareLaunchArgument('target2_z', default_value='0.0'),
        DeclareLaunchArgument('target3_x', default_value='0.0'),
        DeclareLaunchArgument('target3_y', default_value='0.0'),
        DeclareLaunchArgument('target3_z', default_value='0.0'),
        DeclareLaunchArgument('target4_x', default_value='0.0'),
        DeclareLaunchArgument('target4_y', default_value='0.0'),
        DeclareLaunchArgument('target4_z', default_value='0.0'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_world'),
        DeclareLaunchArgument('camera_pose_topic', default_value='/camera/pose'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('cloud_topic', default_value='pcl_render_node/cloud'),
        DeclareLaunchArgument('use_drone_topic_prefix', default_value='false'),
        DeclareLaunchArgument(
            'cmd_topic',
            default_value=PythonExpression(["'/drone_' + str(", drone_id, ") + '_planning/pos_cmd'"]),
        ),
        DeclareLaunchArgument('enable_manual_take_over', default_value='false'),
        DeclareLaunchArgument('enable_multipoint', default_value='false'),
        DeclareLaunchArgument('px4_vehicle_odom_topic', default_value='/fmu/out/vehicle_odometry'),
        DeclareLaunchArgument('camera_extrinsic_topic', default_value='/camera/extrinsics'),
        DeclareLaunchArgument(
            'gz_depth_topic',
            default_value='/depth_camera',
        ),
        DeclareLaunchArgument(
            'gz_camera_info_topic',
            default_value='/camera_info',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_map_linker',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        ),
        Node(
            package='diff_planner',
            executable='camera_pose_publisher.py',
            name='camera_pose_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_topic': resolved_odom_topic,
                'pose_topic': resolved_camera_pose_topic,
                'sync_depth_topic': resolved_depth_topic,
                'world_frame': 'world',
                'body_to_camera_x': 0.13233,
                'body_to_camera_y': 0.0,
                'body_to_camera_z': 0.26078,
            }],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
        Node(
            package='diff_planner',
            executable='gz_depth_bridge.py',
            name='depth_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'gz_depth_topic': gz_depth_topic,
                'gz_camera_info_topic': gz_camera_info_topic,
                'output_topic': resolved_depth_topic,
                'camera_info_topic': '/camera/camera_info',
                'output_frame_id': 'camera_link',
            }],
        ),
        Node(
            package='diff_planner',
            executable='px4_odom_bridge.py',
            name='px4_odom_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_topic': px4_vehicle_odom_topic,
                'output_topic': resolved_odom_topic,
                'frame_id': 'world',
                'child_frame_id': 'base_link',
            }],
        ),
        Node(
            package='diff_planner',
            executable='diff_planner_node',
            name=PythonExpression(["'drone_' + str(", drone_id, ") + '_diff_planner_node'"]),
            output='screen',
            parameters=[planner_node_params],
            remappings=[
                ('odom_world', resolved_odom_topic),
                ('mandatory_stop', '/mandatory_stop_to_planner'),
                ('planning/trajectory', PythonExpression(["'/drone_' + str(", drone_id, ") + '_planning/trajectory'"])),
                ('planning/data_display', PythonExpression(["'/drone_' + str(", drone_id, ") + '_planning/data_display'"])),
                ('planning/broadcast_traj_send', '/broadcast_traj_from_planner'),
                ('planning/broadcast_traj_recv', '/broadcast_traj_to_planner'),
                ('planning/heartbeat', PythonExpression(["'/drone_' + str(", drone_id, ") + '_traj_server/heartbeat'"])),
                ('goal_point', planner_node_topic(drone_id, 'goal_point')),
                ('global_list', planner_node_topic(drone_id, 'global_list')),
                ('init_list', planner_node_topic(drone_id, 'init_list')),
                ('optimal_list', planner_node_topic(drone_id, 'optimal_list')),
                ('failed_list', planner_node_topic(drone_id, 'failed_list')),
                ('a_star_list', planner_node_topic(drone_id, 'a_star_list')),
                ('grid_map/occupancy', planner_node_topic(drone_id, 'grid_map/occupancy')),
                ('grid_map/occupancy_inflate', planner_node_topic(drone_id, 'grid_map/occupancy_inflate')),
                ('grid_map/odom', resolved_odom_topic),
                ('grid_map/cloud', resolved_cloud_topic),
                ('grid_map/pose', resolved_camera_pose_topic),
                ('grid_map/depth', resolved_depth_topic),
            ],
        ),
        Node(
            package='diff_planner',
            executable='traj_server',
            name=PythonExpression(["'drone_' + str(", drone_id, ") + '_traj_server'"]),
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'traj_server.time_forward': 1.0,
            }],
            remappings=[
                ('/position_cmd', cmd_topic),
                ('planning/trajectory', PythonExpression(["'/drone_' + str(", drone_id, ") + '_planning/trajectory'"])),
                ('heartbeat', PythonExpression(["'/drone_' + str(", drone_id, ") + '_traj_server/heartbeat'"])),
            ],
        ),
        Node(
            package='odom_visualization',
            executable='odom_visualization',
            name=PythonExpression(["'drone_' + str(", drone_id, ") + '_odom_visualization'"]),
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'color.a': 1.0,
                'color.r': 0.0,
                'color.g': 0.0,
                'color.b': 0.0,
                'covariance_scale': 100.0,
                'robot_scale': 0.35,
                'tf45': False,
                'drone_id': drone_id,
            }],
            remappings=[
                ('odom', resolved_odom_topic),
            ],
        ),
        Node(
            package='diff_planner',
            executable='trajectory_msg_converter.py',
            name='traj_msg_converter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'traj_topic': cmd_topic,
                'traj_pub_topic': '/command/trajectory',
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'launch', 'include', 'sim.rviz'])],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
