"""Publish nvblox's 2D ESDF slice as a standard nav_msgs/OccupancyGrid.

nvblox reconstructs in 3D and exposes the navigable plane as a
nvblox_msgs/DistanceMapSlice: a raster of *distances to the nearest surface*
(metres), with unobserved cells carrying a sentinel `unknown_value`. 

"""

import array

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from nvblox_msgs.msg import DistanceMapSlice
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

UNKNOWN = -1


class MapSliceToOccupancy(Node):
    def __init__(self):
        super().__init__("map_slice_to_occupancy")

        self.declare_parameter("slice_topic", "/nvblox_node/static_map_slice")
        self.declare_parameter("map_topic", "/map")
        # ESDF distance at or below which a cell counts as an obstacle. 
        self.declare_parameter("occupied_distance", 0.1)

        self._occupied_distance = (
            self.get_parameter("occupied_distance").get_parameter_value().double_value
        )


        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter("map_topic").get_parameter_value().string_value,
            latched,
        )
        self._sub = self.create_subscription(
            DistanceMapSlice,
            self.get_parameter("slice_topic").get_parameter_value().string_value,
            self._on_slice,
            10,
        )
        self._logged_first = False

        # Persistent accumulated grid.
        self._resolution = None
        self._anchor_x = None
        self._anchor_y = None
        self._min_col = 0  # accumulated grid's [*, 0] column, in cells from anchor
        self._min_row = 0  # accumulated grid's [0, *] row, in cells from anchor
        self._grid = None  # 2D int8 array, shape (n_rows, n_cols)

    def _on_slice(self, msg):
        distances = np.asarray(msg.data, dtype=np.float32)


        known = ~np.isclose(distances, msg.unknown_value, rtol=1e-3, atol=1e-3)

        cells = np.full(distances.shape, UNKNOWN, dtype=np.int8)
        cells[known & (distances <= self._occupied_distance)] = 100
        cells[known & (distances > self._occupied_distance)] = 0
        incoming = cells.reshape(msg.height, msg.width)

        self._merge(incoming, msg.resolution, msg.origin.x, msg.origin.y)

        grid = OccupancyGrid()
        grid.header = msg.header
        grid.info.resolution = self._resolution
        grid.info.width = self._grid.shape[1]
        grid.info.height = self._grid.shape[0]
        grid.info.origin.position.x = self._anchor_x + self._min_col * self._resolution
        grid.info.origin.position.y = self._anchor_y + self._min_row * self._resolution
        grid.info.origin.orientation.w = 1.0
        grid.data = array.array("b", self._grid.tobytes())
        self._pub.publish(grid)

        if not self._logged_first:
            self._logged_first = True
            self.get_logger().info(
                f"first nvblox slice -> /map: {msg.width}x{msg.height} @ "
                f"{msg.resolution:.3f} m, frame '{msg.header.frame_id}'"
            )

    def _merge(self, incoming, resolution, origin_x, origin_y):
        """Write `incoming`'s known cells into the persistent grid, growing
        it as needed. Cells the persistent grid already knows are left alone
        wherever the incoming slice reports them as unknown -- that's nvblox
        having cleared/forgotten the area, not new evidence that it's
        actually unexplored."""
        height, width = incoming.shape

        if self._grid is None:
            self._resolution = resolution
            self._anchor_x = origin_x
            self._anchor_y = origin_y
            self._min_col = 0
            self._min_row = 0
            self._grid = incoming.copy()
            return

        if abs(resolution - self._resolution) > 1e-6:
            self.get_logger().warn(
                f"nvblox resolution changed ({self._resolution:.3f} -> "
                f"{resolution:.3f} m) -- resetting the accumulated map."
            )
            self._grid = None
            self._merge(incoming, resolution, origin_x, origin_y)
            return

        col0 = int(round((origin_x - self._anchor_x) / self._resolution))
        row0 = int(round((origin_y - self._anchor_y) / self._resolution))

        new_min_col = min(self._min_col, col0)
        new_min_row = min(self._min_row, row0)
        new_max_col = max(self._min_col + self._grid.shape[1] - 1, col0 + width - 1)
        new_max_row = max(self._min_row + self._grid.shape[0] - 1, row0 + height - 1)
        new_n_cols = new_max_col - new_min_col + 1
        new_n_rows = new_max_row - new_min_row + 1

        if (new_min_col, new_min_row, new_n_cols, new_n_rows) != (
            self._min_col,
            self._min_row,
            self._grid.shape[1],
            self._grid.shape[0],
        ):
            grown = np.full((new_n_rows, new_n_cols), UNKNOWN, dtype=np.int8)
            r = self._min_row - new_min_row
            c = self._min_col - new_min_col
            grown[r : r + self._grid.shape[0], c : c + self._grid.shape[1]] = self._grid
            self._grid = grown
            self._min_col, self._min_row = new_min_col, new_min_row

        r0 = row0 - self._min_row
        c0 = col0 - self._min_col
        target = self._grid[r0 : r0 + height, c0 : c0 + width]
        mask = incoming != UNKNOWN
        target[mask] = incoming[mask]


def main():
    rclpy.init()
    node = MapSliceToOccupancy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
