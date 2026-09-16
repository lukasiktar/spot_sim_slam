"""Render the finished search as a PNG: the 2D map with a pin per person.

RViz shows this live, but a run is worth keeping after the containers are
gone. Both inputs are latched, so this is a one-shot: subscribe, take the
retained map and pin list, draw, exit.

    ros2 run spot_search save_search_map --output /maps/search_result.png
"""

import argparse
import sys

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

UNKNOWN_GREY = 128
FREE_WHITE = 255
OCCUPIED_BLACK = 0


class MapSnapshot(Node):
    def __init__(self):
        super().__init__("save_search_map")
        self.map = None
        self.poses = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, LATCHED)
        self.create_subscription(
            PoseArray, "/found_persons/poses", self._on_poses, LATCHED
        )

    def _on_map(self, msg):
        self.map = msg

    def _on_poses(self, msg):
        self.poses = msg


def render(grid_msg, poses, scale):
    info = grid_msg.info
    cells = np.asarray(grid_msg.data, dtype=np.int8).reshape(info.height, info.width)

    image = np.full(cells.shape, UNKNOWN_GREY, dtype=np.uint8)
    image[cells == 0] = FREE_WHITE
    image[cells >= 50] = OCCUPIED_BLACK

    # Occupancy rows run +y upward; image rows run downward.
    image = np.flipud(image)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    image = cv2.resize(
        image,
        (info.width * scale, info.height * scale),
        interpolation=cv2.INTER_NEAREST,
    )

    for index, pose in enumerate(poses, start=1):
        col = (pose.position.x - info.origin.position.x) / info.resolution
        row = (pose.position.y - info.origin.position.y) / info.resolution
        px = int(col * scale)
        py = int((info.height - row) * scale)
        cv2.circle(image, (px, py), 9 * scale // 2, (0, 0, 220), -1)
        cv2.circle(image, (px, py), 9 * scale // 2, (255, 255, 255), max(1, scale // 2))

        label = f"PERSON {index}"
        font, font_scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 0.5 * scale, max(1, scale)
        (label_w, _), _ = cv2.getTextSize(label, font, font_scale, thickness)
        # Right of the pin by default, flipped to the left when that would run
        # off the image -- a pin near the map's edge is exactly where a
        # cropped label is most likely and least readable.
        label_x = px + 8 * scale
        if label_x + label_w > image.shape[1]:
            label_x = px - 8 * scale - label_w
        cv2.putText(
            image, label, (label_x, py - 4 * scale),
            font, font_scale, (0, 0, 220), thickness, cv2.LINE_AA,
        )
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="/maps/search_result.png")
    parser.add_argument("--scale", type=int, default=3, help="pixels per map cell")
    parser.add_argument("--timeout", type=float, default=15.0)
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = MapSnapshot()

    deadline = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    while rclpy.ok() and node.map is None:
        if node.get_clock().now().nanoseconds * 1e-9 > deadline:
            print("No /map received -- is the nvblox container running?", file=sys.stderr)
            node.destroy_node()
            rclpy.try_shutdown()
            return 1
        rclpy.spin_once(node, timeout_sec=0.2)

    # Pins are optional: a map with zero people found is still worth saving.
    for _ in range(10):
        if node.poses is not None:
            break
        rclpy.spin_once(node, timeout_sec=0.2)

    poses = node.poses.poses if node.poses is not None else []
    image = render(node.map, poses, args.scale)
    cv2.imwrite(args.output, image)
    print(f"wrote {args.output} ({image.shape[1]}x{image.shape[0]} px, "
          f"{len(poses)} person pin(s))")

    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
