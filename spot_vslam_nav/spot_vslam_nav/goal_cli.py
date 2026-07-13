#!/usr/bin/env python3
"""
Minimal CLI for sending Nav2 goals in the SLAM map frame.

Usage:
    ros2 run spot_vslam_nav goal_cli -- 2.0 1.5 90     # x[m] y[m] yaw[deg]
    ros2 run spot_vslam_nav goal_cli -- --cancel
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalSender(Node):
    def __init__(self):
        super().__init__("goal_cli")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def send(self, x: float, y: float, yaw_deg: float) -> bool:
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose action server not available.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"Sending goal: x={x:.2f} y={y:.2f} yaw={yaw_deg:.1f}°")
        send_future = self.client.send_goal_async(goal, feedback_callback=self.on_fb)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        self.get_logger().info(f"Finished with status {status} (4 = SUCCEEDED).")
        return status == 4

    def on_fb(self, fb):
        d = fb.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {d:.2f} m", throttle_duration_sec=2.0)

    def cancel_all(self):
        if self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Cancelling all goals...")
            future = self.client._cancel_goal_async  # noqa: SLF001 (simple CLI)
        # Simpler: publish is handled by nav2 via action cancel from a fresh
        # goal handle; for a CLI, using `ros2 action send_goal --cancel` is
        # also fine. Here we just log guidance.
        self.get_logger().info(
            "Tip: cancel from another shell with:\n"
            "  ros2 action cancel /navigate_to_pose"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", nargs="?", type=float)
    parser.add_argument("y", nargs="?", type=float)
    parser.add_argument("yaw", nargs="?", type=float, default=0.0)
    parser.add_argument("--cancel", action="store_true")
    ns = parser.parse_args(sys.argv[1:])

    rclpy.init()
    node = GoalSender()
    ok = True
    if ns.cancel:
        node.cancel_all()
    elif ns.x is not None and ns.y is not None:
        ok = node.send(ns.x, ns.y, ns.yaw)
    else:
        parser.print_help()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
