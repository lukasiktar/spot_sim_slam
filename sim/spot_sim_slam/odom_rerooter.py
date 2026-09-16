#!/usr/bin/env python3
"""
Re-roots Gazebo's odom->base_link transform at Spot's spawn point.

The upstream Spot world spawns the robot far from world-origin (this world:
~(80.6, 33.6)), and the champ/Gazebo "OdometryPublisher" plugin reports
odom->base_link in that same world-anchored frame rather than relative to
spawn. Every downstream consumer of "odom" (cuVSLAM, nvblox, Nav2's costmaps)
assumes odom is a local, near-zero frame -- fed the raw pose, cuVSLAM's
map->odom correction and this ~84 m lever arm compound, which has been
observed to place base_link tens of metres off the ground plane.

gz_bridge.yaml bridges the raw Gazebo pose to /gz/odom_raw (NOT /tf) so this
node is the only publisher of odom->base_link. It pins the first message's
(x, y) as the origin and subtracts it from every subsequent pose -- z and
rotation pass through unchanged, since those aren't the lever-arm problem.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"


class OdomRerooter(Node):
    def __init__(self):
        super().__init__("odom_rerooter")

        self._origin = None  # (x0, y0), pinned from the first message

        self._broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(TFMessage, "/gz/odom_raw", self._on_raw_tf, qos)

    def _on_raw_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == ODOM_FRAME and t.child_frame_id == BASE_FRAME:
                self._republish(t)

    def _republish(self, raw):
        x = raw.transform.translation.x
        y = raw.transform.translation.y

        if self._origin is None:
            self._origin = (x, y)
            self.get_logger().info(
                f"Pinning odom origin at world ({x:.2f}, {y:.2f})"
            )
        x0, y0 = self._origin

        out = TransformStamped()
        out.header.stamp = raw.header.stamp
        out.header.frame_id = ODOM_FRAME
        out.child_frame_id = BASE_FRAME
        out.transform.translation.x = x - x0
        out.transform.translation.y = y - y0
        out.transform.translation.z = raw.transform.translation.z
        out.transform.rotation = raw.transform.rotation
        self._broadcaster.sendTransform(out)

        odom = Odometry()
        odom.header.stamp = raw.header.stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = out.transform.translation.x
        odom.pose.pose.position.y = out.transform.translation.y
        odom.pose.pose.position.z = out.transform.translation.z
        odom.pose.pose.orientation = raw.transform.rotation
        self._odom_pub.publish(odom)


def main():
    rclpy.init()
    node = OdomRerooter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
