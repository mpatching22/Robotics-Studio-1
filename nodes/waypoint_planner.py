#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class WaypointPlanner(Node):
    """
    Reactive gap-seeking waypoint/vel planner for drones (2.5D):
      - Reads 2D LiDAR (/scan) to find the 'best' free heading
      - (Optional) Reads /terrain_slope in degrees to limit forward motion
      - Publishes /nav/cmd_vel (Twist), to be mixed with Z by altitude_mixer

    Key params (override in launch):
      scan_topic: /scan
      nav_cmd_topic: /nav/cmd_vel
      slope_topic: /terrain_slope  (optional; set '' to disable)
      forward_speed: 1.0           (m/s max)
      slow_distance: 5.0           (m; start slowing if obstacle closer than this)
      stop_distance: 2.0           (m; stop/turn if obstacle closer than this)
      yaw_rate_max: 1.0            (rad/s)
      sector_count: 12             (for 360/12 = 30° sectors)
      front_fov_deg: 90.0
      slope_limit_deg: 15.0        (if slope > this, slow/stop)
      slope_stop_deg: 25.0
      hz: 10.0

      goal_distance: 0.0           (meters to travel before auto-stop; <=0 disables)
    """

    def __init__(self):
        super().__init__('waypoint_planner')

        # Params
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('nav_cmd_topic', '/nav/cmd_vel')
        self.declare_parameter('slope_topic', '/terrain_slope')  # '' means disabled

        self.declare_parameter('forward_speed', 1.0)
        self.declare_parameter('slow_distance', 5.0)
        self.declare_parameter('stop_distance', 2.0)
        self.declare_parameter('yaw_rate_max', 1.0)

        self.declare_parameter('sector_count', 12)
        self.declare_parameter('front_fov_deg', 90.0)

        self.declare_parameter('slope_limit_deg', 15.0)
        self.declare_parameter('slope_stop_deg', 25.0)

        self.declare_parameter('hz', 10.0)
        self.declare_parameter('goal_distance', 0.0)  # optional “go N m”

        # Read params
        self.scan_topic = self.get_parameter('scan_topic').value
        self.nav_cmd_topic = self.get_parameter('nav_cmd_topic').value
        self.slope_topic = self.get_parameter('slope_topic').value

        self.v_max = float(self.get_parameter('forward_speed').value)
        self.dist_slow = float(self.get_parameter('slow_distance').value)
        self.dist_stop = float(self.get_parameter('stop_distance').value)
        self.yaw_rate_max = float(self.get_parameter('yaw_rate_max').value)

        self.sectors = int(self.get_parameter('sector_count').value)
        self.front_fov = math.radians(float(self.get_parameter('front_fov_deg').value))

        self.slope_limit = float(self.get_parameter('slope_limit_deg').value)
        self.slope_stop = float(self.get_parameter('slope_stop_deg').value)

        self.hz = float(self.get_parameter('hz').value)
        self.goal_distance = float(self.get_parameter('goal_distance').value)

        # State
        self.last_scan = None
        self.last_slope_deg = None
        self.odom_start = None
        self.last_odom = None

        # IO
        self.pub_cmd = self.create_publisher(Twist, self.nav_cmd_topic, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/planner/markers', 10)

        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        if self.slope_topic:
            self.sub_slope = self.create_subscription(Float32, self.slope_topic, self._on_slope, 10)

        if self.goal_distance > 0.0:
            self.sub_odom = self.create_subscription(Odometry, '/odometry', self._on_odom, 10)

        self.timer = self.create_timer(1.0 / max(1.0, self.hz), self._tick)

        self.get_logger().info(
            f'WaypointPlanner up: LiDAR={self.scan_topic}, out={self.nav_cmd_topic}, '
            f'slope_topic={"(none)" if not self.slope_topic else self.slope_topic}'
        )

    # ----------------- Callbacks -----------------

    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def _on_slope(self, msg: Float32):
        self.last_slope_deg = float(msg.data)

    def _on_odom(self, msg: Odometry):
        if self.odom_start is None:
            self.odom_start = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.last_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # ----------------- Core planning -----------------

    def _tick(self):
        # If we have a goal_distance, check if we’ve traveled it
        if self.goal_distance > 0.0 and self.odom_start and self.last_odom:
            dx = self.last_odom[0] - self.odom_start[0]
            dy = self.last_odom[1] - self.odom_start[1]
            dist_travelled = math.hypot(dx, dy)
            if dist_travelled >= self.goal_distance:
                self._publish_stop()
                self.get_logger().info(f'✓ Reached goal distance {self.goal_distance:.1f} m, stopping.')
                return

        if self.last_scan is None:
            return

        # Compute sector minima + pick best heading within front FOV
        best_heading, min_front = self._pick_heading(self.last_scan)

        # Speed profile vs obstacle distance
        v_cmd = 0.0
        wz_cmd = 0.0

        # Terrain slope gating (optional)
        slope_ok = True
        if self.last_slope_deg is not None:
            if self.last_slope_deg >= self.slope_stop:
                slope_ok = False
            elif self.last_slope_deg >= self.slope_limit:
                # allow but slow
                pass

        if min_front is None:
            # No valid returns => cautious rotate
            wz_cmd = 0.5 * self.yaw_rate_max
        else:
            if min_front <= self.dist_stop or not slope_ok:
                # Too close (or slope too steep): turn towards best heading
                wz_cmd = self._yaw_towards(best_heading)
                v_cmd = 0.0
            elif min_front <= self.dist_slow:
                # Slow proportionally
                ratio = (min_front - self.dist_stop) / max(1e-6, (self.dist_slow - self.dist_stop))
                v_cmd = max(0.0, min(self.v_max, self.v_max * ratio))
                wz_cmd = self._yaw_towards(best_heading)
            else:
                # Clear path
                v_cmd = self.v_max
                wz_cmd = self._yaw_towards(best_heading)

            # If slope between limit & stop: cap speed
            if slope_ok and self.last_slope_deg is not None and self.slope_limit <= self.last_slope_deg < self.slope_stop:
                v_cmd = min(v_cmd, 0.3 * self.v_max)

        # Publish command
        tw = Twist()
        tw.linear.x = float(v_cmd)
        tw.angular.z = float(wz_cmd)
        self.pub_cmd.publish(tw)

        # RViz markers (optional)
        self._publish_markers(self.last_scan, best_heading, min_front)

    # ----------------- Helpers -----------------

    def _pick_heading(self, scan: LaserScan):
        """Return (best_heading_rad, min_front_distance)."""
        # Angles & cleaned ranges
        n = len(scan.ranges)
        if n == 0 or scan.angle_increment == 0.0:
            return (0.0, None)

        ang = scan.angle_min + np.arange(n, dtype=float) * scan.angle_increment
        rng = np.array([
            r if (math.isfinite(r) and scan.range_min <= r <= scan.range_max) else np.nan
            for r in scan.ranges
        ], dtype=float)

        # FRONT window for “min_front”
        half = self.front_fov / 2.0
        # Wrap angles so 0 is at front
        ang_wrapped = np.array([wrap_to_pi(a) for a in ang])
        front_mask = (np.abs(ang_wrapped) <= half) & ~np.isnan(rng)
        min_front = float(np.nanmin(rng[front_mask])) if np.any(front_mask) else None

        # Sector minima over 360°
        sectors = max(1, self.sectors)
        step = max(1, n // sectors)
        sector_angles = []
        sector_mins = []
        for i in range(sectors):
            s0 = i * step
            s1 = min((i + 1) * step, n)
            seg = rng[s0:s1]
            a0 = ang[s0]
            a1 = ang[s1 - 1]
            a_mid = wrap_to_pi((a0 + a1) * 0.5)
            d_min = float(np.nanmin(seg)) if np.any(~np.isnan(seg)) else 0.0
            sector_angles.append(a_mid)
            sector_mins.append(d_min)

        # Score sectors: prefer ones near heading=0 and with large clearance
        # Score = w_clear * d_min - w_yaw * |angle|
        w_clear = 1.0
        w_yaw = 1.0
        scores = [w_clear * d - w_yaw * abs(a) for a, d in zip(sector_angles, sector_mins)]

        idx_best = int(np.argmax(scores))
        best_heading = float(sector_angles[idx_best])

        return best_heading, min_front

    def _yaw_towards(self, heading):
        # Turn direction toward desired heading, capped at yaw_rate_max
        heading = wrap_to_pi(heading)
        gain = 1.0
        wz = gain * heading
        wz = max(-self.yaw_rate_max, min(self.yaw_rate_max, wz))
        return wz

    def _publish_markers(self, scan: LaserScan, best_heading: float, min_front: float):
        ma = MarkerArray()

        mdel = Marker()
        mdel.action = Marker.DELETEALL
        ma.markers.append(mdel)

        # Arrow for best heading
        arrow = Marker()
        arrow.header.frame_id = scan.header.frame_id
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'planner'; arrow.id = 1
        arrow.type = Marker.ARROW; arrow.action = Marker.ADD
        arrow.scale.x = 0.4; arrow.scale.y = 0.08; arrow.scale.z = 0.08
        arrow.color.r = 0.2; arrow.color.g = 0.9; arrow.color.b = 0.2; arrow.color.a = 1.0

        L = 2.0
        p0 = self._pt(0.0, 0.0)
        p1 = self._pt(L * math.cos(best_heading), L * math.sin(best_heading))
        arrow.points.append(p0); arrow.points.append(p1)
        ma.markers.append(arrow)

        # Text for min_front
        if min_front is not None:
            txt = Marker()
            txt.header.frame_id = scan.header.frame_id
            txt.header.stamp = self.get_clock().now().to_msg()
            txt.ns = 'planner'; txt.id = 2
            txt.type = Marker.TEXT_VIEW_FACING; txt.action = Marker.ADD
            txt.scale.z = 0.4
            txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; txt.color.a = 1.0
            txt.text = f"front: {min_front:.1f} m"
            txt.pose.position.x = 0.0; txt.pose.position.y = 0.0; txt.pose.position.z = 0.0
            ma.markers.append(txt)

        self.pub_markers.publish(ma)

    @staticmethod
    def _pt(x, y):
        from geometry_msgs.msg import Point
        p = Point(); p.x = float(x); p.y = float(y); p.z = 0.0
        return p

    def _publish_stop(self):
        tw = Twist()
        self.pub_cmd.publish(tw)


def main():
    rclpy.init()
    node = WaypointPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
