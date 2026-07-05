from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default='999')
    cmd_topic = LaunchConfiguration('cmd_topic', default='/position_cmd')

    manual_take_over_node = Node(
        package='manual_take_over',
        executable='manual_take_over',
        name=['drone_', drone_id, '_manual_take_over'],
        output='screen',
        remappings=[
            ('/position_cmd', cmd_topic),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='999'),
        DeclareLaunchArgument('cmd_topic', default_value='/position_cmd'),
        manual_take_over_node,
    ])
