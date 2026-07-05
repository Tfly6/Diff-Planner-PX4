import sys
import numpy as np
import os
import math

fname = os.path.join(os.path.dirname(__file__), "random_goals.launch.py")


def main(argv):
    goal_num_in = int(argv[1])
    height = 1.0
    theta = np.linspace(-3.14159, 3.14159, goal_num_in + 1)
    r = 3.0

    with open(fname, "w") as file:
        str_head = (
            "from launch import LaunchDescription\n"
            "from launch_ros.actions import Node\n"
            "from launch.actions import DeclareLaunchArgument\n"
            "from launch.substitutions import LaunchConfiguration\n\n\n"
            "def generate_launch_description():\n"
            "    drone_num = LaunchConfiguration('drone_num', default='10')\n"
            "    goal_num = LaunchConfiguration('goal_num', default='{goal_num}')\n\n"
            "    random_goals_node = Node(\n"
            "        package='random_goals',\n"
            "        executable='random_goals_node',\n"
            "        name='random_goals_node',\n"
            "        output='screen',\n"
            "        parameters=[{{\n"
            "            'drone_num': drone_num,\n"
            "            'goal_num': goal_num,\n"
            .format(goal_num=goal_num_in)
        )
        file.write(str_head)

        for i in range(goal_num_in):
            str_goals = (
                "            'goal{goal_id}': [{p0},{p1},{p2}],\n"
                .format(goal_id=i,
                        p0=r * math.sin(theta[i]),
                        p1=r * math.cos(theta[i]),
                        p2=height)
            )
            file.write(str_goals)

        str_tail = (
            "        }}]\n"
            "    )\n\n"
            "    return LaunchDescription([\n"
            "        random_goals_node,\n"
            "    ])\n"
        )
        file.write(str_tail)


if __name__ == '__main__':
    main(sys.argv)

