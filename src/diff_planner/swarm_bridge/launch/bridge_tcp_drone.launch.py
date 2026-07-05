from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default='0')

    bridge_node = Node(
        package='swarm_bridge',
        executable='bridge_node_tcp',
        name=['drone_', drone_id, '_bridge_node_tcp'],
        output='screen',
        remappings=[
            ('my_odom', ['/drone_', drone_id, '_visual_slam/odom']),
        ],
        parameters=[{
            'self_id': drone_id,
            'is_ground_station': False,
            'odom_max_freq': 10.0,
            'drone_num': 10,
            'drone_ip_0': '127.0.0.1',
            'drone_ip_1': '127.0.0.1',
            'drone_ip_2': '127.0.0.1',
            'drone_ip_3': '127.0.0.1',
            'drone_ip_4': '127.0.0.1',
            'drone_ip_5': '127.0.0.1',
            'drone_ip_6': '127.0.0.1',
            'drone_ip_7': '127.0.0.1',
            'drone_ip_8': '127.0.0.1',
            'drone_ip_9': '127.0.0.1',
            'ground_station_num': 1,
            'ground_station_ip_0': '127.0.0.1',
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
        bridge_node,
        traj2odom_node,
    ])
