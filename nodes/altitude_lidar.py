#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class DownscanAltitude(Node):
    def __init__(self):
        super().__init__('downscan_altitude')
        self.get_logger().info("Altitude Lidar Relay Node Started")
        self.declare_parameter('scan_topic', '/downscan')
        self.declare_parameter('center_deg', 0.0)   # target angle (deg), 0 = straight down
        self.declare_parameter('window_deg', 5.0)   # +/- degrees around center to sample
        self.declare_parameter('lookahead_center_deg', 20.0)   # e.g., 20° off-vertical in +X (forward)
        self.declare_parameter('lookahead_window_deg', 5.0)
        self.lookahead_center_rad = math.radians(float(self.get_parameter('lookahead_center_deg').value))
        self.lookahead_window_rad = math.radians(float(self.get_parameter('lookahead_window_deg').value))
        self.scan_topic = self.get_parameter('scan_topic').value
        self.center_rad = math.radians(float(self.get_parameter('center_deg').value))
        self.window_rad = math.radians(float(self.get_parameter('window_deg').value))

        self.pub_hag_forward = self.create_publisher(Float32, '/altitude/hag_forward', 10)
        self.pub_hag   = self.create_publisher(Float32, '/altitude/hag', 10)
        self.pub_angle = self.create_publisher(Float32, '/altitude/down_angle', 10)
        self.create_subscription(LaserScan, self.scan_topic, self.cb, 10)
        self.get_logger().info(f'Reading {self.scan_topic} for altitude (center={self.center_rad:.3f} rad)')

    def _median_in_window(self, scan, center_rad, halfwin_rad):
        n = len(scan.ranges)
        if n == 0:
            return None, None

        amin, ainc = scan.angle_min, scan.angle_increment
        a0, a1 = center_rad - halfwin_rad, center_rad + halfwin_rad
        i0 = max(0, int(round((a0 - amin) / ainc)))
        i1 = min(n - 1, int(round((a1 - amin) / ainc)))

        vals, angs = [], []
        for i in range(i0, i1 + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and (scan.range_min <= r <= scan.range_max):
                theta = amin + i * ainc
                vals.append(r); angs.append(theta)

        if not vals:
            return None, None

        # robust median; treat <= 4cm as “on ground”
        close_mask = np.array(vals) <= 0.04
        if np.any(close_mask):
            hag = 0.0 if np.all(close_mask) else float(np.median(np.array(vals)[~close_mask]))
        else:
            hag = float(np.median(vals))

        # diagnostic: angle where range is minimum
        theta_min = float(angs[int(np.argmin(vals))])
        return hag, theta_min


    def cb(self, scan: LaserScan):
        # 1) Straight-down HAG (preserves existing behavior via helper)
        hag_down, down_theta = self._median_in_window(scan, self.center_rad, self.window_rad)
        if hag_down is not None:
            self.pub_hag.publish(Float32(data=float(hag_down)))
            # keep your existing angle diagnostic
            self.pub_angle.publish(Float32(data=float(down_theta)))

        # 2) Forward-looking HAG (new preview)
        hag_fwd, _ = self._median_in_window(scan, self.lookahead_center_rad, self.lookahead_window_rad)
        if hag_fwd is None:
            # publish NaN if nothing valid ahead
            self.pub_hag_forward.publish(Float32(data=float('nan')))
        else:
            self.pub_hag_forward.publish(Float32(data=float(hag_fwd)))

        
def main():
    rclpy.init()
    n = DownscanAltitude()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()