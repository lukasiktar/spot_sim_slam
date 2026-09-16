"""Operator front-end: tell Spot how many people to go and find.

    ros2 run spot_search search_cli 2
    ros2 run spot_search search_cli 0   # explore only, no person target

Publishes the request, then follows the mission's status and the pin count
until the search finishes, so the terminal that started the mission is also
the one that reports on it.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String

LATCHED = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

TERMINAL_STATES = ("[DONE]", "[EXHAUSTED]", "[STOPPED]")


class SearchCli(Node):
    def __init__(self, target, follow):
        super().__init__("search_cli")
        self._target = target
        self._follow = follow
        self._last_status = None
        self._saw_running = False
        self.finished = False

        self._start_pub = self.create_publisher(Int32, "/search/start", 10)
        self.create_subscription(String, "/search/status", self._on_status, LATCHED)
        self.create_subscription(Int32, "/found_persons/count", self._on_count, LATCHED)

        self._send_timer = self.create_timer(0.25, self._try_send)
        self._sent = False

    def _try_send(self):
        if self._sent:
            return
        if self._start_pub.get_subscription_count() == 0:
            return
        self._start_pub.publish(Int32(data=self._target))
        self._sent = True
        self._send_timer.cancel()
        print(f"Mission sent: find {self._target} person(s).")
        if not self._follow:
            self.finished = True

    def _on_status(self, msg):
        if msg.data == self._last_status:
            return
        self._last_status = msg.data
        print(f"  {msg.data}")
        if not self._sent:
            return
        if any(msg.data.startswith(s) for s in TERMINAL_STATES):
            if self._saw_running:
                self.finished = True
        else:
            self._saw_running = True

    def _on_count(self, msg):
        if msg.data:
            print(f"  pins on the map: {msg.data}")


def main():
    parser = argparse.ArgumentParser(
        description="Send Spot on an autonomous search for people."
    )
    parser.add_argument(
        "count", type=int, help="how many people to find (0 = explore only)"
    )
    parser.add_argument(
        "--no-follow", action="store_true", help="send the request and exit"
    )
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    if args.count < 0:
        parser.error("count must be at least 0")

    rclpy.init()
    node = SearchCli(args.count, not args.no_follow)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        print("\ninterrupted -- the mission keeps running; "
              "stop it with: ros2 service call /search/stop std_srvs/srv/Trigger")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
