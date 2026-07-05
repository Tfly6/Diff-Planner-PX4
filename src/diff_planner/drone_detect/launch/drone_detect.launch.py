import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    my_id = LaunchConfiguration('my_id', default='1')
    odom_topic = LaunchConfiguration('odom_topic', default='/vins_estimator/imu_propagate')

    pkg_share = get_package_share_directory('drone_detect')

    drone_detect_node = Node(
        package='drone_detect',
        executable='drone_detect',
        name='test_drone_detect',
        output='screen',
        remappings=[
            ('odometry', odom_topic),
            ('depth', '/camera/depth/image_rect_raw'),
        ],
        parameters=[
            os.path.join(pkg_share, 'config', 'camera.yaml'),
            os.path.join(pkg_share, 'config', 'default.yaml'),
            {'my_id': my_id},
            {'debug_flag': False},
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('my_id', default_value='1'),
        DeclareLaunchArgument('odom_topic', default_value='/vins_estimator/imu_propagate'),
        drone_detect_node,
    ])
