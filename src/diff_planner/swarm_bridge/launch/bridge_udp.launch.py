from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default='0')
    broadcast_ip = LaunchConfiguration('broadcast_ip', default='127.0.0.255')

    bridge_node = Node(
        package='swarm_bridge',
        executable='bridge_node_udp',
        name=['drone_', drone_id, '_bridge_node'],
        output='screen',
        remappings=[
            ('my_odom', '/vins_estimator/imu_propagate'),
        ],
        parameters=[{
            'broadcast_ip': broadcast_ip,
            'drone_id': drone_id,
            'odom_max_freq': 70.0,
        }]
    )

    traj2odom_node = Node(
        package='swarm_bridge',
        executable='traj2odom_node',
        name='traj2odom_node',
        output='screen',
        parameters=[{
            'odom_hz': 30.0,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='0'),
        DeclareLaunchArgument('broadcast_ip', default_value='127.0.0.255'),
        bridge_node,
        traj2odom_node,
    ])
