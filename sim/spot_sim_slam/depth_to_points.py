#!/usr/bin/env python3
"""
Depth image -> PointCloud2, replacing Gazebo's own depth_camera point-cloud
output.

This Gazebo version's depth_camera point-cloud generation has a bug: every
point comes out with x >= 0 regardless of which image column it came from,
even though the depth image itself and its camera_info are both correctly
centered/symmetric (verified independently). ROS's own depth_image_proc
package would be the obvious fix, but its point_cloud_xyz_node requires the
depth image and camera_info to share an (approximately) exact timestamp,
and here they come from independent Gazebo publish schedules with a
consistent ~60-70ms gap -- they never sync. Since our camera's intrinsics
are static, there's no need for message-level sync at all: cache the latest
camera_info and apply it to every incoming depth frame directly.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header


class DepthToPoints(Node):
    def __init__(self):
        super().__init__("depth_to_points")
        self.declare_parameter("image_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("points_topic", "/camera/depth/color/points")

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        points_topic = self.get_parameter("points_topic").value

        self.fx = self.fy = self.cx = self.cy = None
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.create_subscription(Image, image_topic, self.on_image, 10)
        self.pub = self.create_publisher(PointCloud2, points_topic, 10)

        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    def on_info(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def on_image(self, msg: Image):
        if self.fx is None:
            return
        if msg.encoding != "32FC1":
            self.get_logger().warn(f"Expected 32FC1 depth, got {msg.encoding}", once=True)
            return

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        v, u = np.indices(depth.shape)
        valid = np.isfinite(depth) & (depth > 0.0)

        z = depth[valid]
        x = (u[valid] - self.cx) / self.fx * z
        y = (v[valid] - self.cy) / self.fy * z

        points = np.empty((z.size, 3), dtype=np.float32)
        points[:, 0] = x
        points[:, 1] = y
        points[:, 2] = z

        cloud = PointCloud2()
        cloud.header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = self._fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = points.tobytes()
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = DepthToPoints()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
