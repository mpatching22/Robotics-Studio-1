#!/usr/bin/env python3

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
import math

def make_qos(reliable: bool):
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=QoSReliabilityPolicy.RELIABLE if reliable else QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

class PineTreeDetector(Node):
    def __init__(self):
        super().__init__('pine_tree_detector')
        
        # Subscribe to the front LiDAR topic
        # subscribe twice (only the matching one will actually receive data)
        self.sub_rel = self.create_subscription(LaserScan, '/scan_front', self.lidar_callback, make_qos(True))
        self.sub_be  = self.create_subscription(LaserScan, '/scan_front', self.lidar_callback, make_qos(False))
        
        # Publisher for detected tree location
        self.tree_publisher = self.create_publisher(Point, '/detected_tree_location', 10)
        
        # Tree detection parameters
        self.min_cluster_size = 5
        self.max_cluster_size = 50
        self.eps = 0.3  # maximum distance between points in a cluster (meters)
        
        # Pine tree characteristics (expected dimensions)
        self.expected_tree_width = 1.5  # meters (approximate pine tree trunk width)
        self.max_detection_range = 15.0  # meters
        
        # Known pine tree location in world coordinates (from SDF file)
        # Pine tree is at (3.0, 0.0, 0) in world coordinates
        # LiDAR is mounted at (0.20, 0.0, 0.25) on the robot base_link
        # So relative to LiDAR, the tree should be at approximately (2.8, 0.0)
        self.expected_tree_x = 2.8  # Expected x position relative to LiDAR
        self.expected_tree_y = 0.0  # Expected y position relative to LiDAR
        self.detection_tolerance = 0.5  # Tolerance in meters
        
        self.get_logger().info('Pine Tree Detector initialized')
    
    def lidar_callback(self, msg):
        """Process LiDAR scan data to detect pine tree"""

        self.seen_first = getattr(self, 'seen_first', False)
        if not self.seen_first:
            self.seen_first = True
            self.get_logger().info(
                f"First scan received: n={len(msg.ranges)}, "
                f"angles=[{msg.angle_min:.3f}, {msg.angle_max:.3f}], inc={msg.angle_increment:.4f}"
            )

        try:
            # Method 1: Simple directional detection (more reliable for known tree position)
            tree_location_simple = self.find_tree_in_direction(msg)
            
            # Method 2: Simple clustering-based detection (more general approach)
            points = self.laser_scan_to_cartesian(msg)
            if len(points) > 0:
                clusters = self.cluster_points(points)
                tree_location_cluster = self.identify_pine_tree(clusters)
            else:
                tree_location_cluster = None
            
            # Use the simple method if available, otherwise use clustering
            tree_location = tree_location_simple if tree_location_simple else tree_location_cluster
            
            if tree_location is not None:
                # Calculate distance from LiDAR
                distance = math.sqrt(tree_location.x**2 + tree_location.y**2)
                
                self.get_logger().info(
                    f'Pine tree detected at: x={tree_location.x:.2f}m, y={tree_location.y:.2f}m, '
                    f'distance={distance:.2f}m'
                )
                
                # Publish the detected tree location
                self.tree_publisher.publish(tree_location)
            else:
                self.get_logger().info('Pine tree not detected in current scan')
                
        except Exception as e:
            self.get_logger().error(f'Error in lidar_callback: {str(e)}')
    
    def find_tree_in_direction(self, msg):
        """Find the tree by looking in the expected direction (Method 1)"""
        expected_angle = math.atan2(self.expected_tree_y, self.expected_tree_x)
        target_index = int((expected_angle - msg.angle_min) / msg.angle_increment)
        target_index = max(0, min(target_index, len(msg.ranges) - 1))
        search_range = int(0.1 / msg.angle_increment)
        valid_readings = []
        for i in range(max(0, target_index - search_range), 
                      min(len(msg.ranges), target_index + search_range + 1)):
            range_val = msg.ranges[i]
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            distance_to_expected = math.sqrt((x - self.expected_tree_x)**2 + (y - self.expected_tree_y)**2)
            if distance_to_expected <= self.detection_tolerance:
                valid_readings.append((x, y))
        if valid_readings:
            best_reading = min(valid_readings, key=lambda r: (r[0] - self.expected_tree_x)**2 + (r[1] - self.expected_tree_y)**2)
            tree_point = Point()
            tree_point.x = best_reading[0]
            tree_point.y = best_reading[1]
            tree_point.z = 0.0
            return tree_point
        return None
    
    def laser_scan_to_cartesian(self, msg):
        points = []
        for i, range_val in enumerate(msg.ranges):
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
            if range_val > self.max_detection_range:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            points.append([x, y])
        return np.array(points)
    
    def cluster_points(self, points):
        if len(points) == 0:
            return {}
        clusters = []
        visited = [False] * len(points)
        for i in range(len(points)):
            if visited[i]:
                continue
            visited[i] = True
            cluster = [points[i]]
            queue = [i]
            while queue:
                idx = queue.pop(0)
                for j in range(len(points)):
                    if not visited[j] and np.linalg.norm(points[j] - points[idx]) <= self.eps:
                        visited[j] = True
                        cluster.append(points[j])
                        queue.append(j)
            if self.min_cluster_size <= len(cluster) <= self.max_cluster_size:
                clusters.append(np.array(cluster))
        return clusters
    
    def identify_pine_tree(self, clusters):
        if not clusters:
            return None
        best_candidate = None
        best_score = 0
        for cluster_points in clusters:
            centroid = np.mean(cluster_points, axis=0)
            distance_to_lidar = np.linalg.norm(centroid)
            min_x, max_x = np.min(cluster_points[:, 0]), np.max(cluster_points[:, 0])
            min_y, max_y = np.min(cluster_points[:, 1]), np.max(cluster_points[:, 1])
            width = max(max_x - min_x, max_y - min_y)
            score = self.calculate_tree_score(cluster_points, centroid, distance_to_lidar, width)
            if score > best_score:
                best_score = score
                best_candidate = centroid
        if best_candidate is not None and best_score > 0.5:
            tree_point = Point()
            tree_point.x = float(best_candidate[0])
            tree_point.y = float(best_candidate[1])
            tree_point.z = 0.0
            return tree_point
        return None
    
    def calculate_tree_score(self, cluster_points, centroid, distance, width):
        score = 0.0
        if 0.8 <= width <= 3.0:
            score += 0.4
        if 1.0 <= distance <= 10.0:
            score += 0.3
        point_density = len(cluster_points) / (width + 0.1)
        if 5 <= point_density <= 30:
            score += 0.2
        if self.is_roughly_circular(cluster_points):
            score += 0.1
        return score
    
    def is_roughly_circular(self, points):
        if len(points) < 5:
            return False
        centroid = np.mean(points, axis=0)
        distances = [np.linalg.norm(point - centroid) for point in points]
        mean_distance = np.mean(distances)
        std_distance = np.std(distances)
        return (std_distance / mean_distance) < 0.5

def main(args=None):
    rclpy.init(args=args)
    pine_tree_detector = PineTreeDetector()
    try:
        rclpy.spin(pine_tree_detector)
    except KeyboardInterrupt:
        pass
    finally:
        pine_tree_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
