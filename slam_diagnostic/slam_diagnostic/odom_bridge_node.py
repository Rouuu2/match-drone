#!/usr/bin/env python3
"""
odom_bridge_node.py
----------------------------------------
Computes the transform between FAST-LIO's world frame (camera_init) and odom
by using the transform tree and LIO odometry.
Useful for visualizing / debugging TF frames in SLAM.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import numpy as np

def pose_to_matrix(pose_msg):
    """Convert pose in Odometry message to 4x4 transformation matrix"""
    q = pose_msg.pose.pose.orientation
    t = pose_msg.pose.pose.position
    x, y, z, w = q.x, q.y, q.z, q.w

    # Quaternion to rotation matrix
    R = np.array([
        [1 - 2 * (y**2 + z**2),     2 * (x*y - z*w),     2 * (x*z + y*w)],
        [2 * (x*y + z*w),     1 - 2 * (x**2 + z**2),     2 * (y*z - x*w)],
        [2 * (x*z - y*w),         2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T

def tf_to_matrix(tf_msg):
    """Convert TransformStamped message to 4x4 transformation matrix"""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w

    R = np.array([
        [1 - 2 * (y**2 + z**2),     2 * (x*y - z*w),     2 * (x*z + y*w)],
        [2 * (x*y + z*w),     1 - 2 * (x**2 + z**2),     2 * (y*z - x*w)],
        [2 * (x*z - y*w),         2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T

def matrix_to_tf(T, parent, child, stamp):
    """Convert 4x4 matrix to TransformStamped"""
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x = T[0, 3]
    tf.transform.translation.y = T[1, 3]
    tf.transform.translation.z = T[2, 3]

    # Rotation matrix to quaternion (robust)
    R = T[:3, :3]
    qw = np.sqrt(1.0 + np.trace(R)) / 2.0
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)

    tf.transform.rotation.w = qw
    tf.transform.rotation.x = qx
    tf.transform.rotation.y = qy
    tf.transform.rotation.z = qz

    return tf

class OdomBridgeNode(Node):
    def __init__(self):
        super().__init__('odom_bridge_node')

        # Declare parameters
        #self.declare_parameter('odom_lio_topic', '/Odometry')
        self.declare_parameter('odom_lio_topic', '/odom_lio')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('lidar_link', 'lidar_link')
        self.declare_parameter('world_frame', 'camera_init')

        # Get parameter values
        topic = self.get_parameter('odom_lio_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link = self.get_parameter('base_link').value
        self.lidar_link = self.get_parameter('lidar_link').value
        self.world_frame = self.get_parameter('world_frame').value

        # TF handling
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        qos = rclpy.qos.QoSProfile(depth=10)
        self.create_subscription(Odometry, topic, self.odom_callback, qos)

        self.get_logger().info(f"Odom bridge running: computing {self.world_frame} → {self.odom_frame}")

    def odom_callback(self, odom_msg: Odometry):
        """Compute and publish camera_init → odom transform using current odometry and TF tree"""
        try:
            stamp = rclpy.time.Time.from_msg(odom_msg.header.stamp)

            # Look up TF transforms
            tf_ob = self.buffer.lookup_transform(self.odom_frame, self.base_link, stamp)
            tf_bl_ll = self.buffer.lookup_transform(self.base_link, self.lidar_link, stamp)

            # Convert to matrices
            T_cb = pose_to_matrix(odom_msg)            # camera_init → body (from odometry)
            T_ob = tf_to_matrix(tf_ob)                 # odom → base_link
            T_bl_ll = tf_to_matrix(tf_bl_ll)           # base_link → lidar_link
            T_ll_bl = np.linalg.inv(T_bl_ll)           # lidar_link → base_link

            # Final transform: camera_init → odom
            T_co = T_cb @ T_ll_bl @ np.linalg.inv(T_ob)
            tf_msg = matrix_to_tf(T_co, self.world_frame, self.odom_frame, odom_msg.header.stamp)
            self.broadcaster.sendTransform(tf_msg)

        except Exception as e:
            self.get_logger().warn(f"[odom_bridge] Failed to compute TF: {e}")

def main():
    rclpy.init()
    node = OdomBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
