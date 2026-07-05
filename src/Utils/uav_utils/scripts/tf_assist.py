#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from math import pi
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped


class OdometryConverter:

    def __init__(self, node, frame_id_in, frame_id_out, broadcast_tf,
                 body_frame_id, intermediate_frame_id, world_frame_id):
        self.node = node
        self.frame_id_in = frame_id_in
        self.frame_id_out = frame_id_out
        self.broadcast_tf = broadcast_tf
        self.body_frame_id = body_frame_id
        self.intermediate_frame_id = intermediate_frame_id
        self.world_frame_id = world_frame_id
        self.in_odom_sub = None
        self.out_odom_pub = None
        self.out_path_pub = None
        self.path_pub_timer = None
        self.tf_pub_flag = True
        if self.broadcast_tf:
            self.node.get_logger().info(
                'ROSTopic: [%s]->[%s] TF: [%s]-[%s]-[%s]' %
                (self.frame_id_in, self.frame_id_out,
                 self.body_frame_id, self.intermediate_frame_id,
                 self.world_frame_id))
        else:
            self.node.get_logger().info(
                'ROSTopic: [%s]->[%s] No TF' %
                (self.frame_id_in, self.frame_id_out))

        self.path = []

    def in_odom_callback(self, in_odom_msg):
        q = np.array([in_odom_msg.pose.pose.orientation.x,
                      in_odom_msg.pose.pose.orientation.y,
                      in_odom_msg.pose.pose.orientation.z,
                      in_odom_msg.pose.pose.orientation.w])
        p = np.array([in_odom_msg.pose.pose.position.x,
                      in_odom_msg.pose.pose.position.y,
                      in_odom_msg.pose.pose.position.z])

        e = euler_from_quaternion(q, 'rzyx')
        wqb = quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
        wqc = quaternion_from_euler(e[0], 0.0, 0.0, 'rzyx')

        # odom
        odom_msg = in_odom_msg
        assert(in_odom_msg.header.frame_id == self.frame_id_in)
        odom_msg.header.frame_id = self.frame_id_out
        odom_msg.child_frame_id = ""
        self.out_odom_pub.publish(odom_msg)

        # tf
        if self.broadcast_tf and self.tf_pub_flag:
            self.tf_pub_flag = False
            stamp = odom_msg.header.stamp

            if not self.frame_id_in == self.frame_id_out:
                self.send_transform(
                    (0.0, 0.0, 0.0),
                    quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx'),
                    stamp, self.frame_id_in, self.frame_id_out)

            if not self.world_frame_id == self.frame_id_out:
                self.send_transform(
                    (0.0, 0.0, 0.0),
                    quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx'),
                    stamp, self.world_frame_id, self.frame_id_out)

            self.send_transform(
                (p[0], p[1], p[2]), wqb, stamp,
                self.body_frame_id, self.world_frame_id)
            self.send_transform(
                (p[0], p[1], p[2]), wqc, stamp,
                self.intermediate_frame_id, self.world_frame_id)

        # path
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.append(pose)

    def send_transform(self, translation, quat, stamp, child, parent):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.br.sendTransform(t)

    def path_pub_callback(self):
        if self.path:
            path = Path()
            path.header = self.path[-1].header
            path.poses = self.path[-30000::1]
            self.out_path_pub.publish(path)

    def tf_pub_callback(self):
        self.tf_pub_flag = True


def main(args=None):
    rclpy.init(args=args)
    node = Node('tf_assist')

    br = TransformBroadcaster(node)

    converters = []
    index = 0
    while True:
        prefix = "converter%d." % index
        try:
            frame_id_in = node.declare_parameter(
                '%sframe_id_in' % prefix, '').value
            if not frame_id_in:
                raise KeyError('frame_id_in not set')
            frame_id_out = node.declare_parameter(
                '%sframe_id_out' % prefix, '').value
            if not frame_id_out:
                raise KeyError('frame_id_out not set')
            broadcast_tf = node.declare_parameter(
                '%sbroadcast_tf' % prefix, False).value
            body_frame_id = node.declare_parameter(
                '%sbody_frame_id' % prefix, 'body').value
            intermediate_frame_id = node.declare_parameter(
                '%sintermediate_frame_id' % prefix, 'intermediate').value
            world_frame_id = node.declare_parameter(
                '%sworld_frame_id' % prefix, 'world').value

            converter = OdometryConverter(
                node, frame_id_in, frame_id_out, broadcast_tf,
                body_frame_id, intermediate_frame_id, world_frame_id)
            converter.br = br
            converter.in_odom_sub = node.create_subscription(
                Odometry, '%sin_odom' % prefix,
                converter.in_odom_callback, 10)
            converter.out_odom_pub = node.create_publisher(
                Odometry, '%sout_odom' % prefix, 10)
            converter.out_path_pub = node.create_publisher(
                Path, '%sout_path' % prefix, 10)

            converter.path_pub_timer = node.create_timer(
                0.5, converter.path_pub_callback)
            converter.tf_pub_timer = node.create_timer(
                0.1, converter.tf_pub_callback)

            index += 1
        except KeyError as e:
            if index == 0:
                raise KeyError(e)
            else:
                if index == 1:
                    node.get_logger().info(
                        'prefix:"%s" not found. Generate %d converter.' %
                        (prefix, index))
                else:
                    node.get_logger().info(
                        'prefix:"%s" not found. Generate %d converters' %
                        (prefix, index))
                break

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
