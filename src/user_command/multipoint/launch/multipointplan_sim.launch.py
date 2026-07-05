import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    odom_topic = LaunchConfiguration('odom_topic', default='/visual_slam/odom')
    next_distance = LaunchConfiguration('next_distance', default='0.3')
    fligt_type = LaunchConfiguration('fligt_type', default='1')
    start_plan = LaunchConfiguration('start_plan', default='1')
    back_plan = LaunchConfiguration('back_plan', default='1')

    pkg_share = get_package_share_directory('multipoint')

    multipoint_node = Node(
        package='multipoint',
        executable='multipointplan',
        name='multipointplan',
        output='screen',
        remappings=[('odom_topic', odom_topic)],
        parameters=[{
            'yaml_path': os.path.join(pkg_share, 'config', 'points.yaml'),
            'next_distance': next_distance,
            'fligt_type': fligt_type,
            'start_plan': start_plan,
            'back_plan': back_plan,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/visual_slam/odom'),
        DeclareLaunchArgument('next_distance', default_value='0.3'),
        DeclareLaunchArgument('fligt_type', default_value='1'),
        DeclareLaunchArgument('start_plan', default_value='1'),
        DeclareLaunchArgument('back_plan', default_value='1'),
        multipoint_node,
    ])
