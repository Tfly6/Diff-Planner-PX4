#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def quat_to_rot_matrix(x: float, y: float, z: float, w: float):
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def rot_matrix_to_quat(rot):
    trace = rot[0][0] + rot[1][1] + rot[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2][1] - rot[1][2]) / s
        y = (rot[0][2] - rot[2][0]) / s
        z = (rot[1][0] - rot[0][1]) / s
    elif rot[0][0] > rot[1][1] and rot[0][0] > rot[2][2]:
        s = math.sqrt(1.0 + rot[0][0] - rot[1][1] - rot[2][2]) * 2.0
        w = (rot[2][1] - rot[1][2]) / s
        x = 0.25 * s
        y = (rot[0][1] + rot[1][0]) / s
        z = (rot[0][2] + rot[2][0]) / s
    elif rot[1][1] > rot[2][2]:
        s = math.sqrt(1.0 + rot[1][1] - rot[0][0] - rot[2][2]) * 2.0
        w = (rot[0][2] - rot[2][0]) / s
        x = (rot[0][1] + rot[1][0]) / s
        y = 0.25 * s
        z = (rot[1][2] + rot[2][1]) / s
    else:
        s = math.sqrt(1.0 + rot[2][2] - rot[0][0] - rot[1][1]) * 2.0
        w = (rot[1][0] - rot[0][1]) / s
        x = (rot[0][2] + rot[2][0]) / s
        y = (rot[1][2] + rot[2][1]) / s
        z = 0.25 * s
    return x, y, z, w


def matmul3(a, b):
    return [
        [
            a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def matvec3(mat, vec):
    return [
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
    ]


class CameraPosePublisher(Node):
    def __init__(self):
        super().__init__('camera_pose_publisher')

        self.pose_topic = self.declare_parameter('pose_topic', '/camera/pose').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom_world').value
        self.sync_depth_topic = self.declare_parameter('sync_depth_topic', '').value
        self.world_frame = self.declare_parameter('world_frame', 'world').value

        # x500_depth + OakD-Lite SDF:
        # base_link -> camera_link = (0.12, 0.03, 0.242)
        # camera_link -> depth sensor = (0.01233, -0.03, 0.01878)
        self.body_to_camera_translation = [
            float(self.declare_parameter('body_to_camera_x', 0.13233).value),
            float(self.declare_parameter('body_to_camera_y', 0.0).value),
            float(self.declare_parameter('body_to_camera_z', 0.26078).value),
        ]

        # Match plan_env's historical optical-frame convention exactly:
        # cam2body = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        self.body_to_camera_rotation = [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ]
        self.msg_count = 0
        self.latest_odom = None

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.depth_sub = None
        if self.sync_depth_topic:
            self.depth_sub = self.create_subscription(
                Image,
                self.sync_depth_topic,
                self.depth_callback,
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            f'Publishing camera pose {self.pose_topic} from odom {self.odom_topic} '
            f'with body->camera translation={self.body_to_camera_translation}, '
            f'sync_depth_topic={self.sync_depth_topic or "(disabled)"}'
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        if not self.sync_depth_topic:
            self.publish_from_odom(msg)

    def depth_callback(self, msg: Image):
        if self.latest_odom is None:
            return
        self.publish_from_odom(self.latest_odom, stamp=msg.header.stamp)

    def publish_from_odom(self, msg: Odometry, stamp=None):
        self.msg_count += 1
        body_rot = quat_to_rot_matrix(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        camera_rot = matmul3(body_rot, self.body_to_camera_rotation)
        camera_offset_world = matvec3(body_rot, self.body_to_camera_translation)

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        if stamp is not None:
            pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose.position.x = float(msg.pose.pose.position.x + camera_offset_world[0])
        pose_msg.pose.position.y = float(msg.pose.pose.position.y + camera_offset_world[1])
        pose_msg.pose.position.z = float(msg.pose.pose.position.z + camera_offset_world[2])

        qx, qy, qz, qw = rot_matrix_to_quat(camera_rot)
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        self.pose_pub.publish(pose_msg)
        if self.msg_count == 1:
            self.get_logger().info(
                'Published first camera pose: '
                f'pos=({pose_msg.pose.position.x:.3f}, {pose_msg.pose.position.y:.3f}, {pose_msg.pose.position.z:.3f}), '
                f'quat=({pose_msg.pose.orientation.x:.3f}, {pose_msg.pose.orientation.y:.3f}, '
                f'{pose_msg.pose.orientation.z:.3f}, {pose_msg.pose.orientation.w:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CameraPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
