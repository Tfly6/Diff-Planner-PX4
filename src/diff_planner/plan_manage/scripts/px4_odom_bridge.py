#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def rotate_vector_ned_to_enu(x: float, y: float, z: float):
    return (float(y), float(x), float(-z))


def rotate_vector_frd_to_flu(x: float, y: float, z: float):
    return (float(x), float(-y), float(-z))


def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        float(w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2),
        float(w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2),
        float(w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2),
        float(w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2),
    )


def quat_conjugate(q):
    w, x, y, z = q
    return (float(w), float(-x), float(-y), float(-z))


def quat_rotate(q, v):
    qv = (0.0, v[0], v[1], v[2])
    return quat_multiply(quat_multiply(q, qv), quat_conjugate(q))[1:]


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quat_normalize(q):
    norm = math.sqrt(sum(component * component for component in q))
    if norm == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(float(component / norm) for component in q)


NED_ENU_Q = quat_from_rpy(math.pi, 0.0, math.pi / 2.0)
AIRCRAFT_BASELINK_Q = quat_from_rpy(math.pi, 0.0, 0.0)


def rotate_quaternion_ned_to_enu_aircraft_to_baselink(q):
    return quat_normalize(
        quat_multiply(quat_multiply(NED_ENU_Q, q), AIRCRAFT_BASELINK_Q)
    )


class Px4OdomBridge(Node):
    def __init__(self):
        super().__init__('px4_odom_bridge')

        self.input_topic = self.declare_parameter(
            'input_topic', '/fmu/out/vehicle_odometry'
        ).value
        self.output_topic = self.declare_parameter(
            'output_topic', '/odom_world'
        ).value
        self.frame_id = self.declare_parameter('frame_id', 'world').value
        self.child_frame_id = self.declare_parameter(
            'child_frame_id', 'base_link'
        ).value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.subscription = self.create_subscription(
            VehicleOdometry, self.input_topic, self.callback, qos
        )
        self.warned_pose_frame = set()
        self.warned_velocity_frame = set()

    def callback(self, msg: VehicleOdometry):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            if msg.pose_frame not in self.warned_pose_frame:
                self.get_logger().warn(
                    'PX4 pose_frame=%d is not NED; px4_odom_bridge expects /fmu/out/vehicle_odometry in NED and will pass position/orientation through once.'
                    % msg.pose_frame
                )
                self.warned_pose_frame.add(msg.pose_frame)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        px, py, pz = msg.position[0], msg.position[1], msg.position[2]
        if msg.pose_frame == VehicleOdometry.POSE_FRAME_NED:
            x_enu, y_enu, z_enu = rotate_vector_ned_to_enu(px, py, pz)
        else:
            x_enu, y_enu, z_enu = (px, py, pz)
        odom.pose.pose.position.x = x_enu
        odom.pose.pose.position.y = y_enu
        odom.pose.pose.position.z = z_enu

        q_px4 = quat_normalize((msg.q[0], msg.q[1], msg.q[2], msg.q[3]))
        if msg.pose_frame == VehicleOdometry.POSE_FRAME_NED:
            q_enu = rotate_quaternion_ned_to_enu_aircraft_to_baselink(q_px4)
        else:
            q_enu = q_px4
        odom.pose.pose.orientation.w = q_enu[0]
        odom.pose.pose.orientation.x = q_enu[1]
        odom.pose.pose.orientation.y = q_enu[2]
        odom.pose.pose.orientation.z = q_enu[3]

        vx, vy, vz = msg.velocity[0], msg.velocity[1], msg.velocity[2]
        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED:
            v_enu = rotate_vector_ned_to_enu(vx, vy, vz)
        elif msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            v_ned = quat_rotate(q_px4, (vx, vy, vz))
            v_enu = rotate_vector_ned_to_enu(v_ned[0], v_ned[1], v_ned[2])
        elif msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_FRD:
            v_enu = (vx, vy, vz)
            if msg.velocity_frame not in self.warned_velocity_frame:
                self.get_logger().warn(
                    'PX4 velocity_frame=FRD is world-fixed with arbitrary heading, so it cannot be converted to ENU without an external heading reference; passing through once.'
                )
                self.warned_velocity_frame.add(msg.velocity_frame)
        else:
            v_enu = (vx, vy, vz)
            if msg.velocity_frame not in self.warned_velocity_frame:
                self.get_logger().warn(
                    f'Unsupported PX4 velocity_frame={msg.velocity_frame}, passing velocity through without transform once.'
                )
                self.warned_velocity_frame.add(msg.velocity_frame)
        odom.twist.twist.linear.x = v_enu[0]
        odom.twist.twist.linear.y = v_enu[1]
        odom.twist.twist.linear.z = v_enu[2]

        avx, avy, avz = rotate_vector_frd_to_flu(
            msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]
        )
        odom.twist.twist.angular.x = avx
        odom.twist.twist.angular.y = avy
        odom.twist.twist.angular.z = avz

        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = Px4OdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
