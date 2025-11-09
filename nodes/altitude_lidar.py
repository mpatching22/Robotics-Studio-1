#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file altitude_lidar.py
@brief Computes height-above-ground (HAG) from a down-facing LaserScan.

@details
This node ingests a `sensor_msgs/LaserScan` from a down-facing or tilted LiDAR,
samples a small angular window around a desired direction, and publishes the
vertical projection of the median range as HAG. It also computes a forward
look-ahead HAG using a second window aimed ahead of the vehicle to anticipate
terrain changes.

@par Parameters
- `scan_topic`            (string, default: "/downscan")
    LaserScan input topic.
- `center_deg`            (double, default: 0.0)
    Central bearing in degrees for the DOWN beam (0° = straight down in the scan frame).
- `window_deg`            (double, default: 5.0)
    Half-width of the sampling window around `center_deg` (degrees).
- `lookahead_center_deg`  (double, default: 20.0)
    Central bearing (deg) for the FORWARD-leaning look-ahead beam (+X direction).
- `lookahead_window_deg`  (double, default: 6.0)
    Half-width of the look-ahead sampling window (degrees).

@par Subscriptions
- `/downscan` (`sensor_msgs/LaserScan`) by default.

@par Publications
- `/altitude/hag`         (`std_msgs/Float32`)       - vertical height below vehicle
- `/altitude/hag_forward` (`std_msgs/Float32`)       - vertical height in forward look-ahead

@note
Angles follow the incoming `LaserScan` frame: we convert degrees → radians and
select ranges within each window. The "height" is the vertical projection:
`h = median_range * cos(angle_offset)`. If no valid samples exist, we publish NaN.
"""

from __future__ import annotations

import math
from typing import Iterable, List

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class DownscanAltitude(Node):
    """Derives HAG (and forward HAG) from a LaserScan using angular windows."""

    def __init__(self) -> None:
        super().__init__('downscan_altitude')
        self.get_logger().info('DownscanAltitude: starting')

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter('scan_topic', '/downscan')
        self.declare_parameter('center_deg', 0.0)
        self.declare_parameter('window_deg', 5.0)
        self.declare_parameter('lookahead_center_deg', 20.0)
        self.declare_parameter('lookahead_window_deg', 6.0)

        self._scan_topic: str = self.get_parameter('scan_topic').get_parameter_value().string_value
        self._center_deg: float = float(self.get_parameter('center_deg').get_parameter_value().double_value)
        self._window_deg: float = float(self.get_parameter('window_deg').get_parameter_value().double_value)
        self._fwd_center_deg: float = float(self.get_parameter('lookahead_center_deg').get_parameter_value().double_value)
        self._fwd_window_deg: float = float(self.get_parameter('lookahead_window_deg').get_parameter_value().double_value)

        # ---------------------------
        # Publishers
        # ---------------------------
        self.pub_hag = self.create_publisher(Float32, '/altitude/hag', 10)
        self.pub_hag_forward = self.create_publisher(Float32, '/altitude/hag_forward', 10)

        # ---------------------------
        # Subscription
        # ---------------------------
        self.sub = self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)

        self.get_logger().info(
            f'DownscanAltitude: scan="{self._scan_topic}", '
            f'center={self._center_deg}±{self._window_deg} deg, '
            f'lookahead={self._fwd_center_deg}±{self._fwd_window_deg} deg'
        )

    # ---------------------------
    # Helpers
    # ---------------------------
    @staticmethod
    def _window_indices(angle_min: float, angle_inc: float, n: int, center_rad: float, half_width_rad: float) -> np.ndarray:
        """
        @brief Compute indices for samples within [center - half_width, center + half_width].

        @param angle_min      Scan angle_min (rad).
        @param angle_inc      Scan angle_increment (rad).
        @param n              Number of ranges.
        @param center_rad     Window centre (rad).
        @param half_width_rad Half window width (rad).

        @return Array of integer indices within the window bounds (clamped to [0, n-1]).
        """
        idx_center = int(round((center_rad - angle_min) / max(angle_inc, 1e-9)))
        width_bins = max(1, int(round(half_width_rad / max(angle_inc, 1e-9))))
        lo = max(0, idx_center - width_bins)
        hi = min(n - 1, idx_center + width_bins)
        return np.arange(lo, hi + 1, dtype=int)

    @staticmethod
    def _finite(vals: Iterable[float]) -> np.ndarray:
        """Return finite-only numpy array from an iterable of floats."""
        arr = np.asarray(list(vals), dtype=float)
        return arr[np.isfinite(arr)]

    # ---------------------------
    # Callback
    # ---------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        """LaserScan callback: compute HAG and forward HAG and publish."""
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            # No data → publish NaN
            self.pub_hag.publish(Float32(data=float('nan')))
            self.pub_hag_forward.publish(Float32(data=float('nan')))
            return

        # Convert degrees → radians for windows
        center_rad = math.radians(self._center_deg)
        half_width_rad = math.radians(self._window_deg)
        fwd_center_rad = math.radians(self._fwd_center_deg)
        fwd_half_width_rad = math.radians(self._fwd_window_deg)

        # Indices for both windows
        idx_down = self._window_indices(msg.angle_min, msg.angle_increment, n, center_rad, half_width_rad)
        idx_fwd = self._window_indices(msg.angle_min, msg.angle_increment, n, fwd_center_rad, fwd_half_width_rad)

        # Extract ranges and filter invalids
        down_ranges = self._finite(msg.ranges[i] for i in idx_down)
        fwd_ranges = self._finite(msg.ranges[i] for i in idx_fwd)

        # Median distances (if any)
        down_med = float(np.median(down_ranges)) if down_ranges.size > 0 else float('nan')
        fwd_med = float(np.median(fwd_ranges)) if fwd_ranges.size > 0 else float('nan')

        # Vertical projection: h = r * cos(angle_offset)
        # angle_offset is absolute bearing relative to vertical beam centre (we use the window centre).
        # If your scan frame uses a different convention, adjust centre parameters accordingly.
        hag = down_med * math.cos(0.0) if math.isfinite(down_med) else float('nan')
        hag_fwd = fwd_med * math.cos(fwd_center_rad - center_rad) if math.isfinite(fwd_med) else float('nan')

        # Publish
        self.pub_hag.publish(Float32(data=hag))
        self.pub_hag_forward.publish(Float32(data=hag_fwd))


def main() -> None:
    """Entry point."""
    rclpy.init()
    node = DownscanAltitude()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
