"""Autonomous search mission: explore until N people are found.

The mission is a small state machine over two Nav2 behaviours:

    SCAN     spin in place so the camera sweeps the full circle. The colour
             camera only sees 69 degrees, so without this the robot would walk
             straight past anyone standing off to the side.
    EXPLORE  pick a frontier -- the boundary between mapped-free and unknown
             space in nvblox's 2D slice -- and navigate to it.

Alternating the two means every newly reached area is both mapped and looked
at. The mission ends when the pin count reaches the target, or when no
frontier remains, i.e. the reachable space has been exhausted.
"""

import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from spot_search.frontier import find_frontiers

IDLE = "IDLE"
SCANNING = "SCANNING"
EXPLORING = "EXPLORING"
DONE = "DONE"
EXHAUSTED = "EXHAUSTED"
STOPPED = "STOPPED"


class SearchMission(Node):
    def __init__(self):
        super().__init__("search_mission")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("min_frontier_cells", 8)
        self.declare_parameter("robot_radius", 0.45)
        self.declare_parameter("frontier_size_weight", 0.15)
        self.declare_parameter("blacklist_radius", 2.0)
        self.declare_parameter("goal_timeout", 180.0)
        self.declare_parameter("scan_segments", 2)
        self.declare_parameter("scan_time_allowance", 30.0)

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._min_frontier_cells = self.get_parameter("min_frontier_cells").value
        self._robot_radius = self.get_parameter("robot_radius").value
        self._size_weight = self.get_parameter("frontier_size_weight").value
        self._blacklist_radius = self.get_parameter("blacklist_radius").value
        self._goal_timeout = self.get_parameter("goal_timeout").value
        self._scan_segments = self.get_parameter("scan_segments").value
        self._scan_time_allowance = self.get_parameter("scan_time_allowance").value

        self._state = IDLE
        self._target = 0
        self._found = 0
        self._map = None
        self._blacklist = []
        self._goal_handle = None
        self._busy = False
        self._goal_deadline = None
        self._scans_left = 0
        self._current_goal = None

        callbacks = ReentrantCallbackGroup()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._status_pub = self.create_publisher(String, "/search/status", latched)
        self._frontier_pub = self.create_publisher(
            MarkerArray, "/search/frontiers", 1
        )

        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, latched, callback_group=callbacks
        )
        self.create_subscription(
            Int32, "/found_persons/count", self._on_count, latched,
            callback_group=callbacks,
        )
        self.create_subscription(
            Int32, "/search/start", self._on_start, 10, callback_group=callbacks
        )
        self.create_service(
            Trigger, "/search/stop", self._on_stop, callback_group=callbacks
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=callbacks
        )
        self._spin_client = ActionClient(
            self, Spin, "spin", callback_group=callbacks
        )

        self.create_timer(0.5, self._tick, callback_group=callbacks)
        self._publish_status("idle -- waiting for a search request on /search/start")

    # ---------------------------------------------------------------- inputs

    def _on_map(self, msg):
        self._map = msg

    def _on_count(self, msg):
        self._found = msg.data

    def _on_start(self, msg):
        target = int(msg.data)
        if target < 0:
            self.get_logger().warn(f"ignoring search request for {target} persons")
            return
        # 0 means explore-only: keep exploring until frontiers run out (or
        # /search/stop), never completing via the found >= target check.
        self._target = target if target > 0 else math.inf
        self._blacklist = []
        self._state = SCANNING
        self._scans_left = self._scan_segments
        self._busy = False
        if target > 0:
            self.get_logger().info(
                f"MISSION START -- searching for {target} person(s). "
                f"{self._found} already pinned."
            )
            self._publish_status(f"searching for {target} person(s)")
        else:
            self.get_logger().info("MISSION START -- explore only, no person target.")
            self._publish_status("exploring (no person target)")

    def _on_stop(self, _request, response):
        self._cancel_goal()
        self._state = STOPPED
        self._publish_status("stopped by operator")
        response.success = True
        response.message = "search mission stopped"
        return response

    # ----------------------------------------------------------- state machine

    def _tick(self):
        if self._state in (IDLE, DONE, EXHAUSTED, STOPPED):
            return

        if self._found >= self._target:
            self._cancel_goal()
            self._state = DONE
            self.get_logger().info(
                f"MISSION COMPLETE -- found all {self._target} person(s). "
                f"Pins are on /found_persons/markers."
            )
            self._publish_status(f"complete: found {self._found}/{self._target}")
            return

        if self._busy:
            if (
                self._goal_deadline is not None
                and self.get_clock().now().nanoseconds * 1e-9 > self._goal_deadline
            ):
                self.get_logger().warn("goal timed out, abandoning it")
                self._blacklist_current()
                self._cancel_goal()
                self._busy = False
            return

        if self._state == SCANNING:
            if self._scans_left <= 0:
                self._state = EXPLORING
                return
            self._send_spin()
        elif self._state == EXPLORING:
            self._send_next_frontier()

    # -------------------------------------------------------------- behaviours

    def _send_spin(self):
        if not self._spin_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("spin action server unavailable", throttle_duration_sec=10.0)
            return
        self._scans_left -= 1
        goal = Spin.Goal()
        # A full turn is split into segments: Nav2's Spin normalises large
        # targets inconsistently across versions, and shorter arcs also give
        # the detector settled frames between rotations.
        goal.target_yaw = float(2.0 * math.pi / self._scan_segments)
        goal.time_allowance = rclpy.duration.Duration(
            seconds=self._scan_time_allowance
        ).to_msg()
        self._busy = True
        self._current_goal = None
        self._goal_deadline = (
            self.get_clock().now().nanoseconds * 1e-9 + self._scan_time_allowance + 10.0
        )
        self._publish_status(
            f"scanning for people ({self._found}/{self._target} found)"
        )
        self._spin_client.send_goal_async(goal).add_done_callback(self._on_goal_sent)

    def _send_next_frontier(self):
        if self._map is None:
            self.get_logger().warn(
                "no /map yet -- is nvblox publishing?", throttle_duration_sec=10.0
            )
            return

        robot = self._robot_xy()
        if robot is None:
            return

        candidates = self._frontiers()
        self._publish_frontier_markers(candidates)

        best = None
        best_score = -1e9
        for candidate in candidates:
            if self._is_blacklisted(candidate["x"], candidate["y"]):
                continue
            distance = math.hypot(candidate["x"] - robot[0], candidate["y"] - robot[1])
            if distance < 0.5:
                continue
            score = self._size_weight * min(candidate["size"], 60) - distance
            if score > best_score:
                best_score = score
                best = candidate

        if best is None:
            self._state = EXHAUSTED
            self.get_logger().info(
                f"EXPLORATION EXHAUSTED -- no reachable frontier left. "
                f"Found {self._found} of {self._target} person(s)."
            )
            self._publish_status(
                f"exhausted: found {self._found}/{self._target}, nothing left to explore"
            )
            return

        yaw = math.atan2(best["y"] - robot[1], best["x"] - robot[0])
        self._send_nav_goal(best["x"], best["y"], yaw)

    def _send_nav_goal(self, x, y, yaw):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                "navigate_to_pose server unavailable", throttle_duration_sec=10.0
            )
            return
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._busy = True
        self._current_goal = (x, y)
        self._goal_deadline = (
            self.get_clock().now().nanoseconds * 1e-9 + self._goal_timeout
        )
        self.get_logger().info(f"exploring frontier at ({x:.2f}, {y:.2f})")
        self._publish_status(
            f"exploring towards ({x:.1f}, {y:.1f}) "
            f"-- {self._found}/{self._target} found"
        )
        self._nav_client.send_goal_async(goal).add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("goal rejected")
            self._blacklist_current()
            self._busy = False
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        self._goal_handle = None
        self._busy = False
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"goal ended with status {status}")
            self._blacklist_current()
        elif self._state == EXPLORING:
            # Arrived somewhere new -- look around before moving on.
            self._state = SCANNING
            self._scans_left = self._scan_segments

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._busy = False

    # ------------------------------------------------------------------ utils

    def _frontiers(self):
        info = self._map.info
        grid = np.asarray(self._map.data, dtype=np.int8).reshape(
            info.height, info.width
        )
        return find_frontiers(
            grid,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            min_cluster_cells=self._min_frontier_cells,
            clearance_cells=int(math.ceil(self._robot_radius / info.resolution)),
        )

    def _robot_xy(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"no {self._map_frame}<-{self._base_frame} TF: {exc}",
                throttle_duration_sec=10.0,
            )
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _is_blacklisted(self, x, y):
        return any(
            math.hypot(x - bx, y - by) < self._blacklist_radius
            for bx, by in self._blacklist
        )

    def _blacklist_current(self):
        if self._current_goal is not None:
            self._blacklist.append(self._current_goal)
            self._current_goal = None

    def _publish_status(self, text):
        self._status_pub.publish(String(data=f"[{self._state}] {text}"))

    def _publish_frontier_markers(self, candidates):
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._map_frame
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, candidate in enumerate(candidates):
            marker = Marker()
            marker.header.frame_id = self._map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontier"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = candidate["x"]
            marker.pose.position.y = candidate["y"]
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color.b = 1.0
            marker.color.g = 0.6
            marker.color.a = 0.8
            markers.markers.append(marker)
        self._frontier_pub.publish(markers)


def main():
    rclpy.init()
    node = SearchMission()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
