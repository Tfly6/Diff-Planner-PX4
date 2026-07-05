#!/usr/bin/env python3

import threading

import rclpy
from builtin_interfaces.msg import Time
from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode
from gz.transport13 import SubscribeOptions
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class GzDepthBridge(Node):
    def __init__(self):
        super().__init__('gz_depth_bridge')
        self.gz_depth_topic = self.declare_parameter('gz_depth_topic', '/depth_camera').value
        self.gz_camera_info_topic = self.declare_parameter('gz_camera_info_topic', '/camera_info').value
        self.depth_topic = self.declare_parameter('output_topic', '/camera/depth/image_raw').value
        self.camera_info_topic = self.declare_parameter('camera_info_topic', '/camera/camera_info').value
        self.output_frame_id = self.declare_parameter('output_frame_id', 'camera_link').value

        self.depth_count = 0
        self.camera_info_count = 0
        self.warned_no_depth = False
        self.warned_no_camera_info = False
        self._lock = threading.Lock()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.depth_publisher = self.create_publisher(Image, self.depth_topic, qos)
        self.camera_info_publisher = self.create_publisher(CameraInfo, self.camera_info_topic, qos)

        self.gz_node = GzNode()
        self.gz_subscribe_options = SubscribeOptions()
        self.gz_node.subscribe_raw(
            self.gz_depth_topic,
            self.depth_callback,
            'gz.msgs.Image',
            self.gz_subscribe_options,
        )
        self.gz_node.subscribe_raw(
            self.gz_camera_info_topic,
            self.camera_info_callback,
            'gz.msgs.CameraInfo',
            self.gz_subscribe_options,
        )

        self.get_logger().info(
            f'Bridging GZ depth {self.gz_depth_topic} -> {self.depth_topic}, '
            f'camera_info {self.gz_camera_info_topic} -> {self.camera_info_topic}, '
            f'frame_id={self.output_frame_id}'
        )
        self.create_timer(5.0, self.watchdog_callback)

    def _stamp_to_ros_time(self, stamp) -> Time:
        ros_time = Time()
        ros_time.sec = int(stamp.sec)
        ros_time.nanosec = int(stamp.nsec)
        return ros_time

    def _header_value(self, header, key):
        for item in header.data:
            if item.key == key:
                if item.value:
                    return item.value[0]
                return ''
        return ''

    def depth_callback(self, data, _info):
        gz_msg = GzImage()
        gz_msg.ParseFromString(data)

        ros_msg = Image()
        ros_msg.header.stamp = self._stamp_to_ros_time(gz_msg.header.stamp)
        ros_msg.header.frame_id = self.output_frame_id
        ros_msg.height = int(gz_msg.height)
        ros_msg.width = int(gz_msg.width)
        ros_msg.encoding = '32FC1'
        ros_msg.is_bigendian = False
        ros_msg.step = int(gz_msg.step) if gz_msg.step else int(gz_msg.width) * 4
        ros_msg.data = gz_msg.data

        self.depth_publisher.publish(ros_msg)

        with self._lock:
            self.depth_count += 1
            depth_count = self.depth_count

        if depth_count == 1:
            self.get_logger().info(
                'Received first depth frame: '
                f'{ros_msg.width}x{ros_msg.height}, step={ros_msg.step}, '
                f'src_frame={self._header_value(gz_msg.header, "frame_id")}'
            )

    def camera_info_callback(self, data, _info):
        gz_msg = GzCameraInfo()
        gz_msg.ParseFromString(data)

        ros_msg = CameraInfo()
        ros_msg.header.stamp = self._stamp_to_ros_time(gz_msg.header.stamp)
        ros_msg.header.frame_id = self.output_frame_id
        ros_msg.height = int(gz_msg.height)
        ros_msg.width = int(gz_msg.width)
        ros_msg.k = [float(v) for v in gz_msg.intrinsics.k]
        ros_msg.p = [float(v) for v in gz_msg.projection.p]
        ros_msg.r = [float(v) for v in gz_msg.rectification_matrix] if gz_msg.rectification_matrix else [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        ros_msg.d = [float(v) for v in gz_msg.distortion.k]
        if gz_msg.distortion.model == gz_msg.distortion.PLUMB_BOB:
            ros_msg.distortion_model = 'plumb_bob'
        elif gz_msg.distortion.model == gz_msg.distortion.RATIONAL_POLYNOMIAL:
            ros_msg.distortion_model = 'rational_polynomial'
        elif gz_msg.distortion.model == gz_msg.distortion.EQUIDISTANT:
            ros_msg.distortion_model = 'equidistant'
        else:
            ros_msg.distortion_model = 'plumb_bob'

        self.camera_info_publisher.publish(ros_msg)

        with self._lock:
            self.camera_info_count += 1
            camera_info_count = self.camera_info_count

        if camera_info_count == 1:
            self.get_logger().info(
                'Received first camera_info: '
                f'{ros_msg.width}x{ros_msg.height}, fx={ros_msg.k[0]:.3f}, fy={ros_msg.k[4]:.3f}'
            )

    def watchdog_callback(self):
        with self._lock:
            depth_count = self.depth_count
            camera_info_count = self.camera_info_count

        if depth_count == 0 and not self.warned_no_depth:
            self.get_logger().warn(f'No GZ depth frame received on {self.gz_depth_topic} after 5 seconds.')
            self.warned_no_depth = True

        if camera_info_count == 0 and not self.warned_no_camera_info:
            self.get_logger().warn(
                f'No GZ camera_info received on {self.gz_camera_info_topic} after 5 seconds.'
            )
            self.warned_no_camera_info = True


def main(args=None):
    rclpy.init(args=args)
    node = GzDepthBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
