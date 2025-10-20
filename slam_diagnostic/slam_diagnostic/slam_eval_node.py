#!/usr/bin/env python3
"""
slam_eval_node.py
----------------------------------------
Records odometry data from ROS 2 topics and evaluates them using
the `evo` toolkit (APE + RPE metrics).

Subscribes to:
 - /odom        → ground truth trajectory (from simulation or PX4)
 - /odom_lio    → estimated trajectory (from FAST-LIO2)

When you stop the node (Ctrl+C), it automatically runs evo_ape and evo_rpe
in both 2D and 3D and saves plots + results in a timestamped folder.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import subprocess
import os
import datetime


class SLAMEvalNode(Node):
    def __init__(self):
        super().__init__('slam_eval_node')

        # Create a timestamped folder for results
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.results_dir = os.path.join("evo_evaluation", timestamp)
        os.makedirs(self.results_dir, exist_ok=True)

        # Output files
        self.gt_file = open(os.path.join(self.results_dir, "gt.txt"), "w")
        self.est_file = open(os.path.join(self.results_dir, "est.txt"), "w")

        # --- Subscriptions ---
        # Ground truth from Gazebo (/odom)
        self.create_subscription(Odometry, "/odom", self.gt_callback, 10)

        # Estimated odometry from FAST-LIO2 (/odom_lio)
        self.create_subscription(Odometry, "/odom_lio", self.est_callback, 10)

        self.get_logger().info(
            f"Recording odometry:\n"
            f"  → /odom      → {self.results_dir}/gt.txt\n"
            f"  → /odom_lio  → {self.results_dir}/est.txt"
        )

    # --------------------------------------------------------------
    #                      CALLBACKS
    # --------------------------------------------------------------
    def gt_callback(self, msg):
        """Save ground-truth odometry in TUM format"""
        self.write_tum_line(self.gt_file, msg)

    def est_callback(self, msg):
        """Save estimated odometry in TUM format"""
        self.write_tum_line(self.est_file, msg)

    # --------------------------------------------------------------
    #                      HELPERS
    # --------------------------------------------------------------
    def write_tum_line(self, file, msg):
        """Convert ROS Odometry → TUM format"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        line = f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} " \
               f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"
        file.write(line)

    def run_evo_eval(self):
        """Run evo_ape and evo_rpe after recording"""
        log_file = os.path.join(self.results_dir, "evo_stats.txt")

        def run(cmd):
            """Helper to run evo commands and log output"""
            self.get_logger().info(f"Running: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True)
            with open(log_file, "a") as f:
                f.write("\n>>> " + " ".join(cmd) + "\n")
                f.write(result.stdout + "\n")
            if result.returncode != 0:
                self.get_logger().error(result.stderr)

        gt_path = os.path.join(self.results_dir, "gt.txt")
        est_path = os.path.join(self.results_dir, "est.txt")

        # Absolute Trajectory Error (2D)
        run([
            "evo_ape", "tum", gt_path, est_path,
            "-va", "--plot", "--plot_mode", "xy",
            "--save_plot", os.path.join(self.results_dir, "ate_xy.pdf"),
            "--save_results", os.path.join(self.results_dir, "ate_xy.zip")
        ])

        # Absolute Trajectory Error (3D)
        run([
            "evo_ape", "tum", gt_path, est_path,
            "-va", "--plot", "--plot_mode", "xyz",
            "--save_plot", os.path.join(self.results_dir, "ate_xyz.pdf"),
            "--save_results", os.path.join(self.results_dir, "ate_xyz.zip")
        ])

        # Relative Pose Error (2D)
        run([
            "evo_rpe", "tum", gt_path, est_path,
            "-va", "--plot", "--plot_mode", "xy",
            "--save_plot", os.path.join(self.results_dir, "rpe_xy.pdf"),
            "--save_results", os.path.join(self.results_dir, "rpe_xy.zip")
        ])

        # Relative Pose Error (3D)
        run([
            "evo_rpe", "tum", gt_path, est_path,
            "-va", "--plot", "--plot_mode", "xyz",
            "--save_plot", os.path.join(self.results_dir, "rpe_xyz.pdf"),
            "--save_results", os.path.join(self.results_dir, "rpe_xyz.zip")
        ])

        self.get_logger().info(f"✅ EVO evaluation complete. Results in: {self.results_dir}")

    def destroy_node(self):
        """Close files and trigger evaluation on shutdown"""
        self.get_logger().info("Stopping recorder — running EVO evaluation...")
        self.gt_file.close()
        self.est_file.close()
        self.run_evo_eval()
        super().destroy_node()


# --------------------------------------------------------------
#                         MAIN
# --------------------------------------------------------------
def main():
    rclpy.init()
    node = SLAMEvalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
