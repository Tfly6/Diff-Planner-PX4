from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    drone_num = LaunchConfiguration('drone_num', default='10')
    goal_num = LaunchConfiguration('goal_num', default='20')

    random_goals_node = Node(
        package='random_goals',
        executable='random_goals_node',
        name='random_goals_node',
        output='screen',
        parameters=[{
            'drone_num': drone_num,
            'goal_num': goal_num,
            'goal0': [-7.96076938005819e-06, -2.999999999989438, 1.0],
            'goal1': [-0.927057797149633, -2.8531673348655997, 1.0],
            'goal2': [-1.76336090919162, -2.4270472397411025, 1.0],
            'goal3': [-2.427054258576642, -1.763351248595976, 1.0],
            'goal4': [-2.8531710248896607, -0.9270464404387113, 1.0],
            'goal5': [-2.9999999999973594, 3.980384690032598e-06, 1.0],
            'goal6': [-2.8531685648786427, 0.9270540115809571, 1.0],
            'goal7': [-2.427049579357222, 1.763357688996175, 1.0],
            'goal8': [-1.7633544687976284, 2.4270519189690676, 1.0],
            'goal9': [-0.9270502260106506, 2.853169794886663, 1.0],
            'goal10': [0.0, 3.0, 1.0],
            'goal11': [0.9270502260106492, 2.8531697948866634, 1.0],
            'goal12': [1.763354468797627, 2.427051918969069, 1.0],
            'goal13': [2.4270495793572207, 1.763357688996177, 1.0],
            'goal14': [2.8531685648786422, 0.9270540115809591, 1.0],
            'goal15': [2.9999999999973594, 3.980384691364866e-06, 1.0],
            'goal16': [2.8531710248896607, -0.9270464404387101, 1.0],
            'goal17': [2.4270542585766424, -1.763351248595975, 1.0],
            'goal18': [1.76336090919162, -2.4270472397411025, 1.0],
            'goal19': [0.927057797149633, -2.8531673348655997, 1.0],
        }]
    )

    return LaunchDescription([
        random_goals_node,
    ])
