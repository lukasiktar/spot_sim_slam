#!/usr/bin/env python3
"""
cmd_vel safety gate between Nav2 and the spot_ros2 driver.

Responsibilities:
  1. Forward cmd_vel_in -> <output_topic> (the Spot driver's cmd_vel).
  2. Watchdog: if Nav2 stops publishing for `input_timeout` s, send one
     zero-twist so Spot stops walking instead of coasting on the last command.
  3. Kill switch: subscribes to /vslam_ok (from vslam_health_monitor).
     When VSLAM tracking is lost, commands are blocked and a zero-twist is
     sent — Spot should not keep executing a plan in a broken map frame.
  4. Hard velocity clamp as a last line of defense.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class CmdVelGate(Node):
    def __init__(self):
        super().__init__("cmd_vel_gate")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("input_timeout", 0.5)
        self.declare_parameter("max_vx", 1.0)
        self.declare_parameter("max_vy", 0.5)
        self.declare_parameter("max_wz", 1.2)

        out = self.get_parameter("output_topic").value
        self.timeout = float(self.get_parameter("input_timeout").value)
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)

        self.vslam_ok = True
        self.last_rx = self.get_clock().now()
        self.stopped = True  # avoid spamming zero-twists

        self.pub = self.create_publisher(Twist, out, 10)
        self.create_subscription(Twist, "cmd_vel_in", self.on_cmd, 10)
        self.create_subscription(Bool, "/vslam_ok", self.on_health, 10)
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(f"Gating cmd_vel -> {out}")

    @staticmethod
    def _clamp(v, lim):
        return max(-lim, min(lim, v))

    def on_health(self, msg: Bool):
        if self.vslam_ok and not msg.data:
            self.get_logger().warn("VSLAM unhealthy -> blocking cmd_vel, stopping Spot.")
            self.send_stop()
        self.vslam_ok = msg.data

    def on_cmd(self, msg: Twist):
        self.last_rx = self.get_clock().now()
        if not self.vslam_ok:
            return
        out = Twist()
        out.linear.x = self._clamp(msg.linear.x, self.max_vx)
        out.linear.y = self._clamp(msg.linear.y, self.max_vy)
        out.angular.z = self._clamp(msg.angular.z, self.max_wz)
        self.pub.publish(out)
        self.stopped = (
            out.linear.x == 0.0 and out.linear.y == 0.0 and out.angular.z == 0.0
        )

    def watchdog(self):
        elapsed = (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9
        if elapsed > self.timeout and not self.stopped:
            self.get_logger().warn("cmd_vel input timed out -> stopping Spot.")
            self.send_stop()

    def send_stop(self):
        self.pub.publish(Twist())
        self.stopped = True


def main():
    rclpy.init()
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
