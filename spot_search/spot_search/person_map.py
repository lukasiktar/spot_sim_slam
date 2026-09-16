"""Turn a stream of per-frame person detections into stable map pins.

"""

import json
import math
import os

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Int32
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


class Track:
    def __init__(self, track_id, x, y, z, score):
        self.id = track_id
        self.x = x
        self.y = y
        self.z = z
        self.hits = 1
        self.score = score
        self.confirmed = False

    def update(self, x, y, z, score):
        # Running mean: every observation gets equal weight, so a single bad
        # depth reading cannot yank a well-established pin across the map.
        self.hits += 1
        weight = 1.0 / self.hits
        self.x += (x - self.x) * weight
        self.y += (y - self.y) * weight
        self.z += (z - self.z) * weight
        self.score = max(self.score, score)


class PersonMap(Node):
    def __init__(self):
        super().__init__("person_map")

        self.declare_parameter("association_radius", 1.5)
        self.declare_parameter("min_hits", 4)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("output_file", "/maps/found_persons.json")

        self._association_radius = self.get_parameter("association_radius").value
        self._min_hits = self.get_parameter("min_hits").value
        self._map_frame = self.get_parameter("map_frame").value
        self._output_file = self.get_parameter("output_file").value

        self._tracks = []
        self._next_id = 1

        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/found_persons/markers", latched
        )
        self._pose_pub = self.create_publisher(PoseArray, "/found_persons/poses", latched)
        self._count_pub = self.create_publisher(Int32, "/found_persons/count", latched)

        self.create_subscription(
            Detection3DArray, "/person_detector/detections", self._on_detections, 10
        )
        self.create_service(Trigger, "/found_persons/reset", self._on_reset)

        self._publish()

    def _on_detections(self, msg):
        changed = False
        for detection in msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            position = result.pose.pose.position
            if self._integrate(
                position.x, position.y, position.z, result.hypothesis.score
            ):
                changed = True
        if changed:
            self._publish()

    def _integrate(self, x, y, z, score):
        """Associate to the nearest track or start a new one.

        Returns True when the set of *confirmed* pins changed, which is the
        only event the mission controller cares about.
        """
        best = None
        best_distance = self._association_radius
        for track in self._tracks:
            distance = math.hypot(track.x - x, track.y - y)
            if distance < best_distance:
                best_distance = distance
                best = track

        if best is None:
            self._tracks.append(Track(self._next_id, x, y, z, score))
            self._next_id += 1
            return False

        was_confirmed = best.confirmed
        best.update(x, y, z, score)
        if not best.confirmed and best.hits >= self._min_hits:
            best.confirmed = True
            self.get_logger().info(
                f"PERSON {best.id} CONFIRMED at "
                f"({best.x:.2f}, {best.y:.2f}) in '{self._map_frame}' "
                f"[{best.hits} sightings, best score {best.score:.2f}]"
            )
        return best.confirmed != was_confirmed

    def _confirmed(self):
        return [t for t in self._tracks if t.confirmed]

    def _publish(self):
        confirmed = self._confirmed()

        self._count_pub.publish(Int32(data=len(confirmed)))

        poses = PoseArray()
        poses.header.frame_id = self._map_frame
        poses.header.stamp = self.get_clock().now().to_msg()
        for track in confirmed:
            pose = Pose()
            pose.position.x = track.x
            pose.position.y = track.y
            pose.position.z = track.z
            pose.orientation.w = 1.0
            poses.poses.append(pose)
        self._pose_pub.publish(poses)

        self._marker_pub.publish(self._markers(confirmed))
        self._save(confirmed)

    def _markers(self, confirmed):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        clear = Marker()
        clear.header.frame_id = self._map_frame
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        red = ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0)
        for track in confirmed:
            # The pin is drawn as a pole from the floor plus a head
            pole = Marker()
            pole.header.frame_id = self._map_frame
            pole.header.stamp = stamp
            pole.ns = "person_pin"
            pole.id = track.id
            pole.type = Marker.CYLINDER
            pole.action = Marker.ADD
            pole.pose.position.x = track.x
            pole.pose.position.y = track.y
            pole.pose.position.z = track.z + 0.6
            pole.pose.orientation.w = 1.0
            pole.scale.x = 0.08
            pole.scale.y = 0.08
            pole.scale.z = 1.2
            pole.color = red
            markers.markers.append(pole)

            head = Marker()
            head.header.frame_id = self._map_frame
            head.header.stamp = stamp
            head.ns = "person_pin_head"
            head.id = track.id
            head.type = Marker.SPHERE
            head.action = Marker.ADD
            head.pose.position.x = track.x
            head.pose.position.y = track.y
            head.pose.position.z = track.z + 1.3
            head.pose.orientation.w = 1.0
            head.scale.x = head.scale.y = head.scale.z = 0.35
            head.color = red
            markers.markers.append(head)

            label = Marker()
            label.header.frame_id = self._map_frame
            label.header.stamp = stamp
            label.ns = "person_label"
            label.id = track.id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = track.x
            label.pose.position.y = track.y
            label.pose.position.z = track.z + 1.8
            label.pose.orientation.w = 1.0
            label.scale.z = 0.4
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f"PERSON {track.id}"
            markers.markers.append(label)

        return markers

    def _save(self, confirmed):
        payload = {
            "frame_id": self._map_frame,
            "count": len(confirmed),
            "persons": [
                {
                    "id": t.id,
                    "x": round(t.x, 3),
                    "y": round(t.y, 3),
                    "z": round(t.z, 3),
                    "sightings": t.hits,
                    "score": round(t.score, 3),
                }
                for t in confirmed
            ],
        }
        try:
            os.makedirs(os.path.dirname(self._output_file), exist_ok=True)
            with open(self._output_file, "w") as handle:
                json.dump(payload, handle, indent=2)
        except OSError as exc:
            self.get_logger().warn(
                f"could not write {self._output_file}: {exc}",
                throttle_duration_sec=30.0,
            )

    def _on_reset(self, _request, response):
        self._tracks = []
        self._next_id = 1
        self._publish()
        response.success = True
        response.message = "person pins cleared"
        return response


def main():
    rclpy.init()
    node = PersonMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
