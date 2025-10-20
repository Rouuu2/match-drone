#!/usr/bin/env python3
"""
pcd_replay_node.py
----------------------------------------
Replays recorded .pcd point clouds and optional IMU CSV data
for simulation, debugging, or SLAM testing.

Publishes:
- /scan_points_fixed  (PointCloud2)
- /livox/imu_fixed    (optional, from CSV)
"""

import os
import glob
import time
import numpy as np
import pandas as pd
import open3d as o3d
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField, Imu
from sensor_msgs_py.point_cloud2 import create_cloud
from geometry_msgs.msg import Vector3, Quaternion

class PCDReplayNode(Node):
    def __init__(self):
        super().__init__('pcd_replay_node')

        # Declare parameters with defaults
        self.declare_parameter('pcd_dir', '')
        self.declare_parameter('imu_csv', '')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('rate_hz', 10.0)

        # Get parameter values
        self.pcd_dir = self.get_parameter('pcd_dir').value
        self.imu_csv = self.get_parameter('imu_csv').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = self.get_parameter('rate_hz').value

        # Create publishers
        self.pub_lidar = self.create_publisher(PointCloud2, '/scan_points_fixed', 10)
        self.pub_imu = self.create_publisher(Imu, '/livox/imu_fixed', 50)

        # Load point clouds
        self.files = sorted(glob.glob(os.path.join(self.pcd_dir, '*.pcd')))
        if not self.files:
            self.get_logger().error(f"No .pcd files found in {self.pcd_dir}")
        else:
            self.get_logger().info(f"Found {len(self.files)} PCD files in {self.pcd_dir}")

        # Load IMU data (if provided)
        self.imu_data = None
        if self.imu_csv and os.path.exists(self.imu_csv):
            self.imu_data = pd.read_csv(self.imu_csv)
            self.imu_idx = 0
            self.get_logger().info(f"Loaded IMU CSV data: {self.imu_csv}")
        else:
            self.get_logger().info("No IMU CSV provided or not found.")

        self.idx = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

    def tick(self):
        if self.idx >= len(self.files):
            self.get_logger().info("✅ Finished PCD replay.")
            rclpy.shutdown()
            return

        fname = self.files[self.idx]
        cloud = o3d.io.read_point_cloud(fname)
        pts = np.asarray(cloud.points, dtype=np.float32)

        if pts.size == 0:
            self.get_logger().warn(f"⚠️ Empty point cloud: {fname}")
            self.idx += 1
            return

        # Build PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = create_cloud(header, fields, pts)
        self.pub_lidar.publish(cloud_msg)

        # Optional IMU publish
        if self.imu_data is not None and self.imu_idx < len(self.imu_data):
            row = self.imu_data.iloc[self.imu_idx]
            imu_msg = Imu()
            imu_msg.header = header
            imu_msg.linear_acceleration = Vector3(x=row['ax'], y=row['ay'], z=row['az'])
            imu_msg.angular_velocity = Vector3(x=row['gx'], y=row['gy'], z=row['gz'])
            imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # dummy
            self.pub_imu.publish(imu_msg)
            self.imu_idx += 1

        self.idx += 1

def main():
    rclpy.init()
    node = PCDReplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
