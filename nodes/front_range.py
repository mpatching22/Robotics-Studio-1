#!/usr/bin/env python3
import rclpy, math, sys
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class FrontRange(Node):
    def __init__(self):
        super().__init__('front_range')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )
        self.sub = self.create_subscription(LaserScan, '/scan_front', self.cb, qos)
        self.pub = self.create_publisher(Float32, '/front_distance', 1)

    def cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return

        # Build a small forward sector around angle 0 (±5°)
        sector = math.radians(5.0)
        amin = msg.angle_min
        inc  = msg.angle_increment
        amax = amin + (n - 1) * inc

        # If 0 rad is outside scan (shouldn’t be with your config), clamp to center
        angle0 = 0.0
        if angle0 < min(amin, amax) or angle0 > max(amin, amax):
            angle0 = (amin + amax) * 0.5

        i0 = int(round((angle0 - amin) / inc))
        w  = max(1, int(round(abs(sector / inc))))  # indices covering ±5°
        iL = max(0, i0 - w)
        iR = min(n - 1, i0 + w)

        vals = []
        for i in range(iL, iR + 1):
            r = msg.ranges[i]
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                vals.append(r)

        if not vals:
            sys.stdout.write("\rno valid forward ranges                 ")
            sys.stdout.flush()
            return

        # More robust than average in clutter: use the minimum hit in the forward sector
        dist = min(vals)

        # Pretty console bar
        bar_len = 32
        clamped = max(msg.range_min, min(dist, msg.range_max))
        filled = int((1.0 - clamped / msg.range_max) * bar_len)
        bar = '█' * filled + '-' * (bar_len - filled)
        sys.stdout.write(f"\rfront distance: {dist:6.3f} m  [{bar}]  ")
        sys.stdout.flush()

        self.pub.publish(Float32(data=float(dist)))

def main():
    rclpy.init()
    rclpy.spin(FrontRange())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
