#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry


class OdomSender(Node):

    def __init__(self):
        super().__init__("odom_sender")

        self.msg = Odometry()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = "world"

        q = quaternion_from_euler(0, 0, 0, "rzyx")

        self.msg.pose.pose.position.x = 0.0
        self.msg.pose.pose.position.y = 0.0
        self.msg.pose.pose.position.z = 0.0
        self.msg.twist.twist.linear.x = 0.0
        self.msg.twist.twist.linear.y = 0.0
        self.msg.twist.twist.linear.z = 0.0
        self.msg.pose.pose.orientation.x = q[0]
        self.msg.pose.pose.orientation.y = q[1]
        self.msg.pose.pose.orientation.z = q[2]
        self.msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(str(self.msg))

        self.pub = self.create_publisher(Odometry, "odom", 10)
        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.counter += 1
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)
        self.get_logger().info("Send %3d msg(s)." % self.counter)


def main(args=None):
    rclpy.init(args=args)
    node = OdomSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
