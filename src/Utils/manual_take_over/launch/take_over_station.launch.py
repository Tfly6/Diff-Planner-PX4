from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        remappings=[('joy', '/joy')],
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 10.0,
            'coalesce_interval': 0.1,
        }]
    )

    station_node = Node(
        package='manual_take_over',
        executable='manual_take_over_station',
        name='manual_take_over_station',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        station_node,
    ])
