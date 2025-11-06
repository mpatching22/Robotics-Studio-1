#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray


def wrap_to_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi


class LidarPerception360(Node):
    """
    Enhanced 360° 2D LiDAR perception with:
      - Adaptive sector resolution
      - Temporal filtering for noise reduction
      - Gap detection and width estimation
      - Dynamic obstacle tracking
    
    Published Topics:
      - /front_distance (Float32)      : min distance in forward cone
      - /nearest_obstacle (Point)      : nearest hit in laser frame
      - /detection_status (String)     : human-readable summary
      - /perception/markers (MarkerArray): sectors + nearest arrow + gaps (RViz)
      - /sector_mins (Float32MultiArray): minimum distances per sector
      - /gap_info (Float32MultiArray)  : [gap_angle, gap_width, gap_clearance] for largest gap
    """
    def __init__(self):
        super().__init__('lidar_perception_360')

        # Basic parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('num_sectors', 16)          # Increased from 8 to 16 for finer resolution
        self.declare_parameter('front_sector_deg', 60.0)
        self.declare_parameter('min_obs_dist', 3.0)
        
        # Temporal filtering parameters
        self.declare_parameter('temporal_buffer_size', 5)   # Number of scans to average
        self.declare_parameter('outlier_threshold', 0.5)    # Meters - reject readings changing > this
        
        # Gap detection parameters
        self.declare_parameter('min_gap_width_deg', 30.0)   # Minimum angular width for a gap
        self.declare_parameter('min_gap_clearance', 3.0)    # Minimum clearance to consider as gap
        self.declare_parameter('gap_search_range', 8.0)     # Max range to search for gaps
        
        # Adaptive resolution parameters
        self.declare_parameter('enable_adaptive_sectors', True)
        self.declare_parameter('high_res_range', 5.0)       # Use fine sectors within this range
        self.declare_parameter('high_res_sectors', 32)      # Fine sector count for nearby obstacles

        scan_topic = self.get_parameter('scan_topic').value
        self.buffer_size = int(self.get_parameter('temporal_buffer_size').value)
        self.outlier_thresh = float(self.get_parameter('outlier_threshold').value)
        self.min_gap_width = math.radians(float(self.get_parameter('min_gap_width_deg').value))
        self.min_gap_clearance = float(self.get_parameter('min_gap_clearance').value)
        self.gap_search_range = float(self.get_parameter('gap_search_range').value)
        self.enable_adaptive = bool(self.get_parameter('enable_adaptive_sectors').value)
        self.high_res_range = float(self.get_parameter('high_res_range').value)
        self.high_res_sectors = int(self.get_parameter('high_res_sectors').value)

        # Temporal filtering buffers
        self.range_history = deque(maxlen=self.buffer_size)
        self.angle_cache = None
        
        # Publishers
        self.sub = self.create_subscription(LaserScan, scan_topic, self._cb_scan, 10)
        self.pub_front   = self.create_publisher(Float32, '/front_distance', 10)
        self.pub_nearest = self.create_publisher(Point, '/nearest_obstacle', 10)
        self.pub_status  = self.create_publisher(String, '/detection_status', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/perception/markers', 10)
        self.pub_sectors = self.create_publisher(Float32MultiArray, '/sector_mins', 10)
        self.pub_gap_info = self.create_publisher(Float32MultiArray, '/gap_info', 10)

        self.get_logger().info(f'Enhanced Perception online; listening to {scan_topic}')
        self.get_logger().info(f'Temporal filtering: {self.buffer_size} scans, Adaptive sectors: {self.enable_adaptive}')


    def _temporal_filter(self, rng):
        """Apply temporal filtering to reduce noise and outliers"""
        # Add current scan to history
        self.range_history.append(rng.copy())
        
        if len(self.range_history) < 2:
            return rng
        
        # Stack all historical ranges
        history_stack = np.array(self.range_history)
        
        # Outlier rejection: flag readings that change too rapidly
        current = history_stack[-1]
        previous = history_stack[-2]
        
        valid_mask = np.where(~np.isnan(current) & ~np.isnan(previous))[0]
        rapid_change = np.abs(current[valid_mask] - previous[valid_mask]) > self.outlier_thresh
        outlier_indices = valid_mask[rapid_change]
        
        # Replace outliers with previous valid value
        filtered = current.copy()
        filtered[outlier_indices] = previous[outlier_indices]
        
        # Apply moving average filter over temporal buffer
        # Use median for robustness against remaining outliers
        filtered_stack = np.where(np.isnan(history_stack), np.nan, history_stack)
        smoothed = np.nanmedian(filtered_stack, axis=0)
        
        return smoothed


    def _detect_gaps(self, ang, rng):
        """
        Detect navigable gaps in obstacle field
        Returns: list of (gap_center_angle, gap_width, gap_clearance)
        """
        gaps = []
        n = len(rng)
        
        # Find contiguous regions with clearance > threshold
        in_gap = False
        gap_start_idx = 0
        
        for i in range(n):
            r = rng[i]
            is_clear = (not np.isnan(r)) and (r > self.min_gap_clearance) and (r < self.gap_search_range)
            
            if is_clear and not in_gap:
                # Start of new gap
                gap_start_idx = i
                in_gap = True
            elif (not is_clear or i == n-1) and in_gap:
                # End of gap
                gap_end_idx = i - 1 if not is_clear else i
                
                # Calculate gap properties
                a_start = ang[gap_start_idx]
                a_end = ang[gap_end_idx]
                
                # Handle wrap-around at +/- pi
                gap_width = wrap_to_pi(a_end - a_start)
                if gap_width < 0:
                    gap_width += 2 * math.pi
                
                # Only consider if gap is wide enough
                if gap_width >= self.min_gap_width:
                    gap_center = wrap_to_pi(a_start + gap_width / 2.0)
                    
                    # Gap clearance is the minimum range in gap
                    gap_ranges = rng[gap_start_idx:gap_end_idx+1]
                    gap_clearance = float(np.nanmin(gap_ranges))
                    
                    gaps.append((gap_center, gap_width, gap_clearance))
                
                in_gap = False
        
        return gaps


    def _adaptive_sector_resolution(self, ang, rng):
        """
        Use finer sector resolution for nearby obstacles
        Returns: sector_mins, sector_angles, num_sectors_used
        """
        nsec = int(self.get_parameter('num_sectors').value)
        
        # Check if any obstacles are within high-res range
        close_obstacle = np.any((~np.isnan(rng)) & (rng < self.high_res_range))
        
        if self.enable_adaptive and close_obstacle:
            # Use high resolution sectors
            nsec = self.high_res_sectors
        
        n = len(rng)
        step = max(1, n // nsec)
        sector_mins = []
        sector_angles = []
        
        for s in range(nsec):
            seg = rng[s*step : min((s+1)*step, n)]
            ang_seg = ang[s*step : min((s+1)*step, n)]
            
            if np.any(~np.isnan(seg)):
                min_val = float(np.nanmin(seg))
                # Find angle of minimum in this sector
                min_idx = np.nanargmin(seg)
                min_angle = float(ang_seg[min_idx])
            else:
                min_val = float('inf')
                min_angle = float(np.mean(ang_seg))
            
            sector_mins.append(min_val)
            sector_angles.append(min_angle)
        
        return sector_mins, sector_angles, nsec


    def _cb_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return

        # Build angle array (cache for efficiency)
        if self.angle_cache is None or len(self.angle_cache) != n:
            self.angle_cache = msg.angle_min + np.arange(n, dtype=float) * msg.angle_increment
        ang = self.angle_cache
        
        # Sanitize ranges
        rng = np.array([
            r if (np.isfinite(r) and msg.range_min <= r <= msg.range_max)
            else np.nan
            for r in msg.ranges
        ], dtype=float)

        if np.all(np.isnan(rng)):
            return
        
        # Apply temporal filtering
        rng = self._temporal_filter(rng)

        # FRONT CONE DISTANCE
        half = math.radians(self.get_parameter('front_sector_deg').value / 2.0)
        ang_err = np.array([wrap_to_pi(a) for a in ang])
        front_mask = (np.abs(ang_err) <= half) & ~np.isnan(rng)
        front_dist = float(np.nanmin(rng[front_mask])) if np.any(front_mask) else float(np.nanmin(rng))
        self.pub_front.publish(Float32(data=front_dist))

        # NEAREST OBSTACLE
        valid_idx = np.where(~np.isnan(rng))[0]
        if len(valid_idx) > 0:
            i_min = int(valid_idx[np.nanargmin(rng[valid_idx])])
            d_min = float(rng[i_min])
            a_min = float(ang[i_min])
            nearest = Point(x=d_min * math.cos(a_min), y=d_min * math.sin(a_min), z=0.0)
            self.pub_nearest.publish(nearest)
        else:
            d_min = float('inf')
            a_min = 0.0
            nearest = Point(x=0.0, y=0.0, z=0.0)
            self.pub_nearest.publish(nearest)

        # ADAPTIVE SECTOR RESOLUTION
        sector_mins, sector_angles, nsec = self._adaptive_sector_resolution(ang, rng)
        
        arr = Float32MultiArray()
        arr.data = sector_mins
        self.pub_sectors.publish(arr)

        # GAP DETECTION
        gaps = self._detect_gaps(ang, rng)
        
        # Publish largest gap info
        if gaps:
            # Sort by gap clearance * width (prioritize both factors)
            gaps_sorted = sorted(gaps, key=lambda g: g[1] * g[2], reverse=True)
            best_gap = gaps_sorted[0]
            
            gap_msg = Float32MultiArray()
            gap_msg.data = [float(best_gap[0]), float(best_gap[1]), float(best_gap[2])]
            self.pub_gap_info.publish(gap_msg)
        else:
            gap_msg = Float32MultiArray()
            gap_msg.data = [0.0, 0.0, 0.0]
            self.pub_gap_info.publish(gap_msg)

        # STATUS STRING
        near_thr = float(self.get_parameter('min_obs_dist').value)
        num_near = int(np.nansum(rng < near_thr))
        num_gaps = len(gaps)
        
        status = (
            f"Min: {d_min:.2f}m @ {math.degrees(a_min):.1f}°, "
            f"Near(<{near_thr:.1f}m): {num_near}, "
            f"Gaps: {num_gaps}, "
            f"Sectors: {nsec}, "
            "Sect(m): " + ", ".join(f"{v:.1f}" for v in sector_mins[:8])  # Show first 8
        )
        self.pub_status.publish(String(data=status))
        self.get_logger().info(status)

        # RViz markers
        self.pub_markers.publish(self._make_markers(msg, sector_mins, sector_angles, 
                                                     nsec, i_min, d_min, a_min, gaps))


    def _make_markers(self, scan: LaserScan, sector_mins, sector_angles, nsec, 
                      i_min, d_min, a_min, gaps):
        ma = MarkerArray()

        mdel = Marker()
        mdel.action = Marker.DELETEALL
        ma.markers.append(mdel)

        # Sector edge markers
        sector_width = 2 * math.pi / nsec
        for i in range(nsec):
            d = sector_mins[i]
            if not math.isfinite(d):
                continue
                
            m = Marker()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = scan.header.frame_id
            m.ns = "sectors"
            m.id = i
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.02
            
            # Color code by distance: red=close, yellow=medium, green=far
            if d < 2.0:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
            elif d < 5.0:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
            m.color.a = 0.6

            a0 = sector_angles[i] - sector_width / 2
            a1 = sector_angles[i] + sector_width / 2
            
            m.points.append(self._pt(0.0, 0.0))
            m.points.append(self._pt(d*math.cos(a0), d*math.sin(a0)))
            m.points.append(self._pt(0.0, 0.0))
            m.points.append(self._pt(d*math.cos(a1), d*math.sin(a1)))
            ma.markers.append(m)

        # Nearest obstacle arrow
        if math.isfinite(d_min):
            arrow = Marker()
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.header.frame_id = scan.header.frame_id
            arrow.ns = "nearest"
            arrow.id = 999
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.05
            arrow.scale.y = 0.10
            arrow.scale.z = 0.10
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.2, 0.2, 1.0
            arrow.points.append(self._pt(0.0, 0.0))
            arrow.points.append(self._pt(d_min*math.cos(a_min), d_min*math.sin(a_min)))
            ma.markers.append(arrow)

        # Gap visualization
        for idx, (gap_angle, gap_width, gap_clearance) in enumerate(gaps):
            gap_marker = Marker()
            gap_marker.header.stamp = self.get_clock().now().to_msg()
            gap_marker.header.frame_id = scan.header.frame_id
            gap_marker.ns = "gaps"
            gap_marker.id = 1000 + idx
            gap_marker.type = Marker.LINE_STRIP
            gap_marker.action = Marker.ADD
            gap_marker.scale.x = 0.05
            gap_marker.color.r, gap_marker.color.g, gap_marker.color.b, gap_marker.color.a = 0.0, 1.0, 1.0, 0.8
            
            # Draw arc for gap
            a_start = gap_angle - gap_width / 2
            a_end = gap_angle + gap_width / 2
            num_arc_points = 20
            for j in range(num_arc_points + 1):
                t = j / num_arc_points
                a = a_start + t * gap_width
                gap_marker.points.append(self._pt(gap_clearance * math.cos(a), 
                                                   gap_clearance * math.sin(a)))
            ma.markers.append(gap_marker)
            
            # Gap center arrow
            gap_arrow = Marker()
            gap_arrow.header.stamp = self.get_clock().now().to_msg()
            gap_arrow.header.frame_id = scan.header.frame_id
            gap_arrow.ns = "gap_centers"
            gap_arrow.id = 2000 + idx
            gap_arrow.type = Marker.ARROW
            gap_arrow.action = Marker.ADD
            gap_arrow.scale.x = 0.08
            gap_arrow.scale.y = 0.15
            gap_arrow.scale.z = 0.15
            gap_arrow.color.r, gap_arrow.color.g, gap_arrow.color.b, gap_arrow.color.a = 0.0, 1.0, 1.0, 1.0
            gap_arrow.points.append(self._pt(0.0, 0.0))
            gap_arrow.points.append(self._pt(gap_clearance * math.cos(gap_angle),
                                              gap_clearance * math.sin(gap_angle)))
            ma.markers.append(gap_arrow)

        return ma


    @staticmethod
    def _pt(x, y):
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        return p


def main():
    rclpy.init()
    node = LidarPerception360()
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