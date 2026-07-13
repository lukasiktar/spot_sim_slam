#!/usr/bin/env python3
"""
Auto-bringup for Spot: claim lease -> power on -> stand.

Calls the std_srvs/Trigger services exposed by the spot_ros2 driver:
    /<spot_name>/claim
    /<spot_name>/power_on
    /<spot_name>/stand

Exits after success so it doesn't hold resources. Safe to re-run.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class SpotAutoBringup(Node):
    def __init__(self):
        super().__init__("spot_auto_bringup")
        self.declare_parameter("spot_name", "")
        name = self.get_parameter("spot_name").get_parameter_value().string_value
        self.prefix = f"/{name}" if name else ""

    def call(self, srv_name: str, timeout: float = 30.0) -> bool:
        full = f"{self.prefix}/{srv_name}"
        client = self.create_client(Trigger, full)
        self.get_logger().info(f"Waiting for {full} ...")
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Service {full} unavailable.")
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error(f"{full} call timed out.")
            return False
        res = future.result()
        level = self.get_logger().info if res.success else self.get_logger().error
        level(f"{full}: success={res.success} msg='{res.message}'")
        return res.success

    def run(self) -> bool:
        for srv, wait_after in (("claim", 1.0), ("power_on", 3.0), ("stand", 2.0)):
            if not self.call(srv):
                return False
            time.sleep(wait_after)
        self.get_logger().info("Spot is up: claimed, powered, standing.")
        return True


def main():
    rclpy.init()
    node = SpotAutoBringup()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
