"""Detect people in the RGB stream and locate them in 3D.

YOLO gives us pixel boxes; depth turns each box into a metric point; TF puts
that point in the map frame so the rest of the stack can reason about *where*
a person is rather than merely that one is visible.

"""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

import tf2_ros


def _rotate(quat_xyzw, vec):
    """Rotate `vec` by a quaternion, without pulling in tf2_geometry_msgs."""
    axis = np.asarray(quat_xyzw[:3], dtype=np.float64)
    w = float(quat_xyzw[3])
    t = 2.0 * np.cross(axis, vec)
    return vec + w * t + np.cross(axis, t)


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")

        self.declare_parameter("model_path", "/models/yolov8n.pt")
        self.declare_parameter("confidence", 0.4)
        self.declare_parameter("max_rate_hz", 4.0)
        self.declare_parameter("max_range", 8.0)
        self.declare_parameter("min_range", 0.4)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("depth_is_aligned_to_color", False)
        self.declare_parameter("publish_debug_image", True)

        self._confidence = self.get_parameter("confidence").value
        self._max_range = self.get_parameter("max_range").value
        self._min_range = self.get_parameter("min_range").value
        self._target_frame = self.get_parameter("target_frame").value
        self._aligned = self.get_parameter("depth_is_aligned_to_color").value
        self._debug = self.get_parameter("publish_debug_image").value
        self._min_period = 1.0 / float(self.get_parameter("max_rate_hz").value)

        self._bridge = CvBridge()
        self._color_info = None
        self._depth_info = None
        self._depth = None
        self._last_run = 0.0

        from ultralytics import YOLO  # heavy import, only once the node is alive

        model_path = self.get_parameter("model_path").value
        self._model = YOLO(model_path)
        self.get_logger().info(f"loaded YOLO weights from {model_path}")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(
            Detection3DArray, "/person_detector/detections", 10
        )
        self._debug_pub = self.create_publisher(
            Image, "/person_detector/debug_image", 1
        )

        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self._on_color_info, 10
        )
        self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self._on_depth_info, 10
        )
        self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self._on_depth, 1
        )
        self.create_subscription(Image, "/camera/color/image_raw", self._on_color, 1)

    def _on_color_info(self, msg):
        self._color_info = msg

    def _on_depth_info(self, msg):
        self._depth_info = msg

    def _on_depth(self, msg):
        self._depth = msg

    def _depth_at(self, depth_image, col, row, half_window=4):
        """Median depth in a small patch, ignoring invalid returns.

        The patch is sampled at the box centre rather than over the whole box:
        a person's bounding box always catches background around the limbs and
        the mean of that is somewhere behind them.
        """
        height, width = depth_image.shape
        r_lo, r_hi = max(0, row - half_window), min(height, row + half_window + 1)
        c_lo, c_hi = max(0, col - half_window), min(width, col + half_window + 1)
        patch = depth_image[r_lo:r_hi, c_lo:c_hi].astype(np.float32).ravel()
        valid = patch[np.isfinite(patch) & (patch > self._min_range) & (patch < self._max_range)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _on_color(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_run < self._min_period:
            return
        if self._depth is None or self._color_info is None or self._depth_info is None:
            return
        self._last_run = now

        color_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        depth_image = self._bridge.imgmsg_to_cv2(
            self._depth, desired_encoding="passthrough"
        )
        # 16UC1 depth (real RealSense) is in millimetres; 32FC1 (sim) is metres.
        if depth_image.dtype == np.uint16:
            depth_image = depth_image.astype(np.float32) * 0.001

        results = self._model.predict(
            color_image, conf=self._confidence, classes=[0], verbose=False
        )

        # Published before the TF check so the annotated view still works as a
        # "is the detector seeing anything at all" probe when localisation is
        # not up yet.
        if self._debug and results:
            annotated = results[0].plot()
            debug_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug_msg.header = msg.header
            self._debug_pub.publish(debug_msg)

        transform = self._lookup(msg.header.frame_id, msg.header.stamp)
        if transform is None:
            return

        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._target_frame

        color_k = self._color_info.k
        depth_k = self._depth_info.k
        boxes = results[0].boxes if results else None

        for index in range(0 if boxes is None else len(boxes)):
            x1, y1, x2, y2 = boxes.xyxy[index].tolist()
            score = float(boxes.conf[index])
            u_color = 0.5 * (x1 + x2)
            v_color = 0.5 * (y1 + y2)

            # Colour pixel -> bearing, using the colour intrinsics.
            bearing_x = (u_color - color_k[2]) / color_k[0]
            bearing_y = (v_color - color_k[5]) / color_k[4]

            if self._aligned:
                u_depth, v_depth = u_color, v_color
            else:
                u_depth = bearing_x * depth_k[0] + depth_k[2]
                v_depth = bearing_y * depth_k[4] + depth_k[5]

            row, col = int(round(v_depth)), int(round(u_depth))
            if not (0 <= row < depth_image.shape[0] and 0 <= col < depth_image.shape[1]):
                continue
            distance = self._depth_at(depth_image, col, row)
            if distance is None:
                continue

            point_camera = np.array(
                [bearing_x * distance, bearing_y * distance, distance], dtype=np.float64
            )
            rotation, translation = transform
            point_map = _rotate(rotation, point_camera) + translation

            detection = Detection3D()
            detection.header = out.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = "person"
            hypothesis.hypothesis.score = score
            hypothesis.pose.pose.position.x = float(point_map[0])
            hypothesis.pose.pose.position.y = float(point_map[1])
            hypothesis.pose.pose.position.z = float(point_map[2])
            hypothesis.pose.pose.orientation.w = 1.0
            detection.results.append(hypothesis)
            detection.bbox = BoundingBox3D()
            detection.bbox.center = hypothesis.pose.pose
            detection.bbox.size.x = 0.6
            detection.bbox.size.y = 0.6
            detection.bbox.size.z = 1.7
            out.detections.append(detection)

        if out.detections:
            self._pub.publish(out)

    def _lookup(self, source_frame, stamp):
        """Camera-optical -> target frame as (quaternion_xyzw, translation).

        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame, source_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except tf2_ros.TransformException:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._target_frame, source_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
            except tf2_ros.TransformException as exc:
                self.get_logger().warn(
                    f"no {self._target_frame}<-{source_frame} TF: {exc}",
                    throttle_duration_sec=5.0,
                )
                return None
        rotation = tf.transform.rotation
        translation = tf.transform.translation
        return (
            np.array([rotation.x, rotation.y, rotation.z, rotation.w]),
            np.array([translation.x, translation.y, translation.z]),
        )


def main():
    rclpy.init()
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
