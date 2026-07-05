#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudToWorld(Node):
    def __init__(self):
        super().__init__('pointcloud_to_world')

        self.target_frame = self.declare_parameter('target_frame', 'world').value
        self.source_topic = self.declare_parameter('source_topic', '/livox/lidar').value
        self.output_topic = self.declare_parameter('output_topic', '/livox/lidar_world').value
        self.source_frame = self.declare_parameter('source_frame', 'livox_link').value
        self.lookup_timeout = float(self.declare_parameter('lookup_timeout', 0.1).value)
        self.use_latest_on_extrapolation = bool(
            self.declare_parameter('use_latest_on_extrapolation', True).value
        )
        self.filter_enable = bool(self.declare_parameter('filter_enable', True).value)
        self.voxel_leaf_size = float(self.declare_parameter('voxel_leaf_size', 0.08).value)
        self.min_range = self.declare_parameter('min_range', 0.2).value
        self.max_range = self.declare_parameter('max_range', 50.0).value
        self.min_z = self.declare_parameter('min_z', -1.0e9).value
        self.max_z = self.declare_parameter('max_z', 1.0e9).value
        self.max_points = int(self.declare_parameter('max_points', 200000).value)
        self.cloud_timeout = float(self.declare_parameter('cloud_timeout', 0.5).value)
        self.log_interval = float(self.declare_parameter('log_interval', 2.0).value)
        self.filter_log = bool(self.declare_parameter('filter_log', True).value)

        self.last_msg_time = None
        self.last_cloud_stamp = None
        self.tf_fail_count = 0
        self.tf_extrap_count = 0
        self.last_log_time = {}

        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 1)
        self.sub = self.create_subscription(PointCloud2, self.source_topic, self.callback, 1)
        self.watchdog_timer = self.create_timer(0.2, self.watchdog_callback)

    def callback(self, msg: PointCloud2):
        self.last_msg_time = self.get_clock().now()
        self.last_cloud_stamp = msg.header.stamp
        frame_id = msg.header.frame_id.strip('/') if msg.header.frame_id else self.source_frame
        if '::' in frame_id:
            frame_id = frame_id.split('::')[-1]

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = rclpy.time.Time()

        transform = self.lookup_transform(frame_id, stamp)
        if transform is None:
            return

        cloud_out = do_transform_cloud(msg, transform)
        cloud_out.header.frame_id = self.target_frame
        if not self.filter_enable:
            self.pub.publish(cloud_out)
            return

        self.pub.publish(self.filter_cloud(cloud_out))

    def lookup_transform(self, frame_id: str, stamp: rclpy.time.Time):
        try:
            return self.buffer.lookup_transform(
                self.target_frame,
                frame_id,
                stamp,
                timeout=Duration(seconds=self.lookup_timeout),
            )
        except (LookupException, ConnectivityException) as exc:
            self.tf_fail_count += 1
            self.log_throttled(
                'tf_lookup_fail',
                f'TF lookup failed: {exc} (fail_count={self.tf_fail_count})',
            )
            return None
        except ExtrapolationException as exc:
            if not self.use_latest_on_extrapolation:
                self.tf_fail_count += 1
                self.log_throttled(
                    'tf_extrap_fail',
                    f'TF extrapolation failed: {exc} (fail_count={self.tf_fail_count})',
                )
                return None

            self.tf_extrap_count += 1
            self.log_throttled(
                'tf_extrap_fallback',
                f'TF extrapolation: {exc} (extrap_count={self.tf_extrap_count}), falling back to latest TF.',
            )
            try:
                return self.buffer.lookup_transform(
                    self.target_frame,
                    frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.lookup_timeout),
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as exc2:
                self.tf_fail_count += 1
                self.log_throttled(
                    'tf_fallback_fail',
                    f'TF lookup failed: {exc2} (fail_count={self.tf_fail_count})',
                )
                return None

    def log_throttled(self, key: str, message: str):
        now = self.get_clock().now()
        last = self.last_log_time.get(key)
        if last is None or (now - last).nanoseconds / 1e9 >= self.log_interval:
            self.get_logger().warn(message)
            self.last_log_time[key] = now

    def watchdog_callback(self):
        if self.last_msg_time is None:
            return

        dt = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if dt > self.cloud_timeout:
            stamp = 0.0
            if self.last_cloud_stamp is not None:
                stamp = self.last_cloud_stamp.sec + self.last_cloud_stamp.nanosec / 1e9
            self.log_throttled(
                'cloud_watchdog',
                'No pointcloud received for %.3fs (last_cloud_stamp=%.3f, tf_fail=%d, tf_extrap=%d)'
                % (dt, stamp, self.tf_fail_count, self.tf_extrap_count),
            )

    def filter_cloud(self, cloud_msg: PointCloud2):
        points = np.array(
            list(pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)),
            dtype=np.float32,
        )
        initial_count = points.shape[0]

        if points.size == 0:
            if self.filter_log:
                self.log_throttled(
                    'empty_after_nan',
                    f'Pointcloud empty after NaN removal (frame={cloud_msg.header.frame_id})',
                )
            header = Header(frame_id=self.target_frame, stamp=cloud_msg.header.stamp)
            return pc2.create_cloud_xyz32(header, [])

        range_count = initial_count
        if self.min_range is not None or self.max_range is not None:
            ranges = np.linalg.norm(points, axis=1)
            range_mask = np.ones(points.shape[0], dtype=bool)
            if self.min_range is not None:
                range_mask &= ranges >= float(self.min_range)
            if self.max_range is not None:
                range_mask &= ranges <= float(self.max_range)
            points = points[range_mask]
            range_count = points.shape[0]

        if points.size == 0:
            if self.filter_log:
                self.log_throttled(
                    'empty_after_range',
                    'Pointcloud filtered out by range (initial=%d, min=%.2f, max=%.2f)'
                    % (
                        initial_count,
                        float(self.min_range) if self.min_range is not None else -1.0,
                        float(self.max_range) if self.max_range is not None else -1.0,
                    ),
                )
            header = Header(frame_id=self.target_frame, stamp=cloud_msg.header.stamp)
            return pc2.create_cloud_xyz32(header, [])

        z_count = points.shape[0]
        if self.min_z is not None:
            points = points[points[:, 2] >= float(self.min_z)]
        if self.max_z is not None:
            points = points[points[:, 2] <= float(self.max_z)]
        z_count = points.shape[0]

        if points.size == 0:
            if self.filter_log:
                self.log_throttled(
                    'empty_after_z',
                    'Pointcloud filtered out by z (initial=%d, range=%d, min_z=%.2f, max_z=%.2f)'
                    % (
                        initial_count,
                        range_count,
                        float(self.min_z) if self.min_z is not None else -1.0,
                        float(self.max_z) if self.max_z is not None else -1.0,
                    ),
                )
            header = Header(frame_id=self.target_frame, stamp=cloud_msg.header.stamp)
            return pc2.create_cloud_xyz32(header, [])

        sample_count = points.shape[0]
        if self.max_points and points.shape[0] > self.max_points:
            idx = np.random.choice(points.shape[0], self.max_points, replace=False)
            points = points[idx]
            sample_count = points.shape[0]

        voxel_count = points.shape[0]
        if self.voxel_leaf_size > 0.0:
            voxel_idx = np.floor(points / self.voxel_leaf_size).astype(np.int32)
            _, unique_indices = np.unique(voxel_idx, axis=0, return_index=True)
            points = points[unique_indices]
            voxel_count = points.shape[0]

        if points.size == 0 and self.filter_log:
            self.log_throttled(
                'empty_after_filter',
                'Pointcloud empty after filtering (initial=%d, range=%d, z=%d, sample=%d, voxel=%d, leaf=%.3f)'
                % (
                    initial_count,
                    range_count,
                    z_count,
                    sample_count,
                    voxel_count,
                    self.voxel_leaf_size,
                ),
            )

        header = Header(frame_id=self.target_frame, stamp=cloud_msg.header.stamp)
        return pc2.create_cloud_xyz32(header, points.tolist())


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToWorld()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
