#!/usr/bin/env python3

"""
Convert planner PositionCommand to MultiDOFJointTrajectory for controllers that consume it.
"""

from geometry_msgs.msg import Transform, Twist
from quadrotor_msgs.msg import PositionCommand
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


def quaternion_from_yaw(yaw: float):
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class MessageConverter(Node):
    def __init__(self):
        super().__init__('trajectory_msg_converter')

        planner_traj_topic = self.declare_parameter(
            'traj_topic', 'planning/pos_cmd'
        ).value
        traj_pub_topic = self.declare_parameter(
            'traj_pub_topic', 'command/trajectory'
        ).value

        self.traj_pub = self.create_publisher(
            MultiDOFJointTrajectory, traj_pub_topic, 1
        )
        self.input_topic = planner_traj_topic
        self.output_topic = traj_pub_topic
        self.msg_count = 0
        self.create_subscription(
            PositionCommand,
            planner_traj_topic,
            self.planner_traj_callback,
            10,
        )
        self.get_logger().info(
            f'Converting {planner_traj_topic} -> {traj_pub_topic}'
        )

    def planner_traj_callback(self, msg: PositionCommand):
        self.msg_count += 1
        pose = Transform()
        pose.translation.x = msg.position.x
        pose.translation.y = msg.position.y
        pose.translation.z = msg.position.z
        q = quaternion_from_yaw(msg.yaw)
        pose.rotation.x = q[0]
        pose.rotation.y = q[1]
        pose.rotation.z = q[2]
        pose.rotation.w = q[3]

        vel = Twist()
        vel.linear = msg.velocity
        vel.angular.z = msg.yaw_dot

        acc = Twist()
        acc.linear = msg.acceleration

        traj_point = MultiDOFJointTrajectoryPoint()
        traj_point.transforms.append(pose)
        traj_point.velocities.append(vel)
        traj_point.accelerations.append(acc)

        traj_msg = MultiDOFJointTrajectory()
        traj_msg.header = msg.header
        traj_msg.points.append(traj_point)
        self.traj_pub.publish(traj_msg)
        if self.msg_count == 1:
            self.get_logger().info(
                f'Received first PositionCommand on {self.input_topic}; '
                f'publishing MultiDOFJointTrajectory to {self.output_topic}'
            )



def main(args=None):
    rclpy.init(args=args)
    node = MessageConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
