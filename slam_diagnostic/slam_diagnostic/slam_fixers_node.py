#!/usr/bin/env python3
"""
slam_fixers_node.py
----------------------------------
Combines multiple utility fixers into a single ROS 2 node:
- IMU frame fixer
- PointCloud2 frame fixer
- Odometry TF broadcaster (odom → base_link)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.topic_endpoint_info import TopicEndpointInfo
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
import time

class SLAMFixers(Node):
    def __init__(self):
        super().__init__('slam_fixers')

        # Declare parameters
        self.declare_parameter('imu_in_topic', '/livox/imu')
        self.declare_parameter('imu_out_topic', '/livox/imu_fixed')
        self.declare_parameter('imu_target_frame','base_link' )#'lidar_link')

        self.declare_parameter('pc_in_topic', '/gz_scan/points')
        self.declare_parameter('pc_out_topic', '/scan_points_fixed')
        self.declare_parameter('pc_target_frame', 'lidar_link')

        self.declare_parameter('odom_topic', '/odom_lio')
        self.declare_parameter('odom_child_frame', 'base_link')

        # Get parameter values
        imu_in_topic  = self.get_parameter('imu_in_topic').value
        imu_out_topic = self.get_parameter('imu_out_topic').value
        imu_frame     = self.get_parameter('imu_target_frame').value

        pc_in_topic   = self.get_parameter('pc_in_topic').value
        pc_out_topic  = self.get_parameter('pc_out_topic').value
        pc_frame      = self.get_parameter('pc_target_frame').value

        odom_topic    = self.get_parameter('odom_topic').value
        odom_child    = self.get_parameter('odom_child_frame').value

        # QoS setup
        qos_sensor = QoSProfile(depth=100)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_sensor.history = HistoryPolicy.KEEP_LAST

        # ---- Publishers & Subscribers ----
        self.imu_frame = imu_frame
        self.pc_frame = pc_frame
        self.odom_child_frame = odom_child
        self.br = TransformBroadcaster(self)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, imu_out_topic, 10)
        self.pc_pub = self.create_publisher(PointCloud2, pc_out_topic, 10)

        # Subscriptions
        self.imu_sub = self.create_subscription(Imu, imu_in_topic, self.imu_callback, qos_sensor)
        self.pc_sub = self.create_subscription(PointCloud2, pc_in_topic, self.pc_callback, qos_sensor)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info(" SLAM fixers initialized: IMU, PointCloud, TF broadcaster")

    # -------------------------------------------------------------------------
    # --- CALLBACKS WITH SAFETY GUARDS ---
    # -------------------------------------------------------------------------

    def imu_callback(self, msg: Imu):
        try:
            if not msg or msg.header is None:
                self.get_logger().warn("Received empty IMU message – skipped")
                return
            msg.header.frame_id = self.imu_frame
            self.imu_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"IMU callback error: {e}")

    def pc_callback(self, msg: PointCloud2):
        try:
            if not msg or len(msg.data) == 0:
                self.get_logger().warn("Received empty PointCloud2 – skipped")
                return
            msg.header.frame_id = self.pc_frame
            self.pc_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"PointCloud2 callback error: {e}")

    def odom_callback(self, msg: Odometry):
        try:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = self.odom_child_frame
            t.transform.translation = msg.pose.pose.position
            t.transform.rotation = msg.pose.pose.orientation
            self.br.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f"Odometry callback error: {e}")

# -----------------------------------------------------------------------------
# --- MAIN ENTRY ---
# -----------------------------------------------------------------------------
def main():
    rclpy.init()
    node = SLAMFixers()

    # --- Wait for IMU and PointCloud topics before spinning ---
    imu_topic = node.get_parameter('imu_in_topic').value
    pc_topic = node.get_parameter('pc_in_topic').value

    node.get_logger().info("⏳ Waiting for IMU and PointCloud2 topics...")
    found_imu = found_pc = False
    start = time.time()
    while not (found_imu and found_pc):
        topics = [t for t, _ in node.get_topic_names_and_types()]
        if imu_topic in topics:
            found_imu = True
        if pc_topic in topics:
            found_pc = True
        if time.time() - start > 10:  # wait up to 10 seconds
            node.get_logger().warn("Timeout waiting for topics; starting anyway.")
            break
        time.sleep(0.5)

    node.get_logger().info("✅ Topics active — starting spin loop.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
