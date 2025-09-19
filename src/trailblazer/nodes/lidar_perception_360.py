#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

class LidarPerception360(Node):
    """
    Perception products from a 360° 2D LiDAR (/scan):
      - /front_distance  (Float32, min distance in forward sector)
      - /nearest_obstacle (Point, nearest hit in laser frame)
      - /detection_status (String, human-readable status)
      - /perception/markers (MarkerArray, sectors + nearest arrow for RViz)
    """
    def __init__(self):
        super().__init__('lidar_perception_360')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('num_sectors', 8)         # 360° / 8 = 45° sectors
        self.declare_parameter('front_sector_deg', 30.0) # ±15° about 0 rad
        self.declare_parameter('min_obs_dist', 3.0)      # “nearby” threshold

        scan_topic = self.get_parameter('scan_topic').value
        self.sub = self.create_subscription(LaserScan, scan_topic, self._cb_scan, 10)

        self.pub_front   = self.create_publisher(Float32, '/front_distance', 10)
        self.pub_nearest = self.create_publisher(Point,   '/nearest_obstacle', 10)
        self.pub_status  = self.create_publisher(String,  '/detection_status', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/perception/markers', 10)

        self.get_logger().info(f'Perception online; listening to {scan_topic}')

    def _cb_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        # sanitize ranges
        rng = np.array([r if (np.isfinite(r) and msg.range_min < r < msg.range_max)
                        else msg.range_max for r in msg.ranges], dtype=float)
        ang = msg.angle_min + np.arange(n, dtype=float) * msg.angle_increment

        # front sector (around 0 rad)
        half = math.radians(self.get_parameter('front_sector_deg').value / 2.0)
        front_mask = np.abs(ang) <= half
        front_dist = float(np.min(rng[front_mask])) if np.any(front_mask) else float(np.min(rng))
        self.pub_front.publish(Float32(data=front_dist))

        # nearest obstacle
        i_min = int(np.argmin(rng))
        d_min = float(rng[i_min])
        a_min = float(ang[i_min])
        nearest = Point(x=d_min * math.cos(a_min), y=d_min * math.sin(a_min), z=0.0)
        self.pub_nearest.publish(nearest)

        # sectorization
        nsec = int(self.get_parameter('num_sectors').value)
        step = max(1, n // nsec)
        sector_mins = []
        for s in range(nsec):
            start = s * step
            end   = min((s + 1) * step, n)
            seg = rng[start:end]
            sector_mins.append(float(np.min(seg)) if seg.size > 0 else msg.range_max)

        # status string
        near_thr = float(self.get_parameter('min_obs_dist').value)
        num_near = int(np.sum(rng < near_thr))
        status = (
            f"Min: {d_min:.2f} m @ {math.degrees(a_min):.1f}°, "
            f"Near(<{near_thr:.1f}m): {num_near}, "
            "Sectors(min m): " + ", ".join(f"{v:.1f}" for v in sector_mins)
        )
        self.pub_status.publish(String(data=status))
        self.get_logger().info(status)

        # RViz markers
        self.pub_markers.publish(self._make_markers(msg, sector_mins, step, i_min, d_min, a_min))

    def _make_markers(self, scan: LaserScan, sector_mins, step, i_min, d_min, a_min):
        ma = MarkerArray()

        # delete-all
        mdel = Marker()
        mdel.action = Marker.DELETEALL
        ma.markers.append(mdel)

        n = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1

        # sector rays
        for i, d in enumerate(sector_mins):
            m = Marker()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = scan.header.frame_id  # e.g., base_scan
            m.ns = "sectors"
            m.id = i
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.02  # line width
            # light grey so it shows up
            m.color.r = 0.8; m.color.g = 0.8; m.color.b = 0.8; m.color.a = 1.0

            # two edge rays per sector
            start_idx = i * step
            end_idx   = min((i + 1) * step - 1, n - 1)
            a0 = scan.angle_min + start_idx * scan.angle_increment
            a1 = scan.angle_min + end_idx   * scan.angle_increment

            m.points.append(self._pt(0.0, 0.0)); m.points.append(self._pt(d*math.cos(a0), d*math.sin(a0)))
            m.points.append(self._pt(0.0, 0.0)); m.points.append(self._pt(d*math.cos(a1), d*math.sin(a1)))
            ma.markers.append(m)

        # nearest arrow
        arrow = Marker()
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.header.frame_id = scan.header.frame_id
        arrow.ns = "nearest"
        arrow.id = 999
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.05   # shaft dia
        arrow.scale.y = 0.10   # head dia
        arrow.scale.z = 0.10   # head len
        arrow.color.r = 1.0; arrow.color.g = 0.2; arrow.color.b = 0.2; arrow.color.a = 1.0
        arrow.points.append(self._pt(0.0, 0.0))
        arrow.points.append(self._pt(d_min*math.cos(a_min), d_min*math.sin(a_min)))
        ma.markers.append(arrow)

        return ma

    @staticmethod
    def _pt(x, y):
        p = Point(); p.x = x; p.y = y; p.z = 0.0
        return p

def main():
    rclpy.init()
    rclpy.spin(LidarPerception360())
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# export LIBGL_ALWAYS_SOFTWARE=1
# ros2 launch trailblazer mission.launch.py rviz:=True nav2:=False world:=simple_trees.sdf


# ros2 topic list | grep scan
# ros2 topic echo /scan --qos-profile sensor_data --once

# ros2 topic echo /detection_status --once
# ros2 topic echo /front_distance --once
# ros2 topic echo /nearest_obstacle --once
