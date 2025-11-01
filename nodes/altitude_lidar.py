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
        self.scan_topic = self.get_parameter('scan_topic').value
        self.center_rad = math.radians(float(self.get_parameter('center_deg').value))
        self.window_rad = math.radians(float(self.get_parameter('window_deg').value))

        self.pub_hag   = self.create_publisher(Float32, '/altitude/hag', 10)
        self.pub_angle = self.create_publisher(Float32, '/altitude/down_angle', 10)
        self.create_subscription(LaserScan, self.scan_topic, self.cb, 10)
        self.get_logger().info(f'Reading {self.scan_topic} for altitude (center={self.center_rad:.3f} rad)')

    def cb(self, scan: LaserScan):
        # build angle vector lazily
        n = len(scan.ranges)
        if n == 0:
            return
        amin, ainc = scan.angle_min, scan.angle_increment

        # indices within [center - window, center + window]
        a0, a1 = self.center_rad - self.window_rad, self.center_rad + self.window_rad
        i0 = max(0, int(round((a0 - amin)/ainc)))
        i1 = min(n-1, int(round((a1 - amin)/ainc)))

        vals, angs = [], []
        for i in range(i0, i1+1):
            r = scan.ranges[i]
            if math.isfinite(r) and (scan.range_min <= r <= scan.range_max):
                theta = amin + i*ainc
                vals.append(r)
                angs.append(theta)

        if not vals:
            return

        # choose median for stability; also report the angle of the minimum range
        vals = np.array(vals, dtype=np.float32)
        angs = np.array(angs, dtype=np.float32)

        # Treat anything <= 4 cm as "on ground". Don't collapse vals to a scalar.
        close_mask = vals <= 0.04
        if np.any(close_mask):
            # If every reading is very close, HAG = 0; else use median of non-close readings.
            hag = 0.0 if np.all(close_mask) else float(np.median(vals[~close_mask]))
        else:
            hag = float(np.median(vals))

        idx_min = int(np.argmin(vals))
        down_angle = float(angs[idx_min])

        # --- publish results ---
        msg_hag = Float32()
        msg_hag.data = hag
        self.pub_hag.publish(msg_hag)

        msg_ang = Float32()
        msg_ang.data = down_angle
        self.pub_angle.publish(msg_ang)
        
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
