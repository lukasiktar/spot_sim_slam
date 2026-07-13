#!/usr/bin/env python3
"""
Watches Isaac ROS Visual SLAM tracking health and publishes /vslam_ok (Bool).

Health signals:
  - /visual_slam/status (isaac_ros_visual_slam_interfaces/VisualSlamStatus):
      vo_state 1/2 = good tracking, anything else = degraded/lost.
  - Staleness: if no status arrives for `stale_timeout` s, report unhealthy.

The message import is done defensively so this node still runs (staleness
check only) if the interface package layout changes between Isaac ROS
releases.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    HAS_STATUS_MSG = True
except ImportError:  # older/newer Isaac ROS layouts
    HAS_STATUS_MSG = False

# cuVSLAM vo_state values (Isaac ROS 3.x)
VO_STATE_OK = {1, 2}  # 1 = tracking, 2 = tracking with map


class VslamHealthMonitor(Node):
    def __init__(self):
        super().__init__("vslam_health_monitor")
        self.declare_parameter("spot_name", "")
        self.declare_parameter("stale_timeout", 1.5)
        self.stale_timeout = float(self.get_parameter("stale_timeout").value)

        self.last_status_time = None
        self.tracking_ok = False

        self.pub = self.create_publisher(Bool, "/vslam_ok", 10)

        if HAS_STATUS_MSG:
            self.create_subscription(
                VisualSlamStatus, "/visual_slam/status", self.on_status, 10
            )
        else:
            self.get_logger().warn(
                "VisualSlamStatus msg not found — health check limited to "
                "topic staleness on /visual_slam/tracking/odometry."
            )
            from nav_msgs.msg import Odometry

            self.create_subscription(
                Odometry, "/visual_slam/tracking/odometry", self.on_any, 10
            )

        self.create_timer(0.2, self.tick)

    def on_status(self, msg):
        self.last_status_time = self.get_clock().now()
        self.tracking_ok = int(msg.vo_state) in VO_STATE_OK

    def on_any(self, _msg):
        self.last_status_time = self.get_clock().now()
        self.tracking_ok = True

    def tick(self):
        ok = self.tracking_ok
        if self.last_status_time is None:
            ok = False
        else:
            age = (self.get_clock().now() - self.last_status_time).nanoseconds * 1e-9
            if age > self.stale_timeout:
                ok = False
        self.pub.publish(Bool(data=ok))


def main():
    rclpy.init()
    node = VslamHealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
