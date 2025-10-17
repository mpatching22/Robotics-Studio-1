#!/usr/bin/env python3
import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def yaw_from_quat(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def quat_from_yaw(yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)  # x,y,z,w

class MinimalDroneNavigator(Node):
    def __init__(self):
        super().__init__('minimal_drone_navigator')
        qos_sub = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.last_pose: Optional[PoseStamped] = None
        self.nav_ready = False
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone/pose_1hz', self._pose_cb, qos_sub
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry', self._odom_cb, qos_sub
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('=== STEP 1: Waiting for pose data ===')
        while rclpy.ok() and self.last_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info('Still waiting for pose...')
        if self.last_pose:
            p = self.last_pose.pose.position
            self.get_logger().info(f'✓ Got initial pose: x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}')
            if p.z < 0.3:
                self.get_logger().warn(f'Drone altitude is {p.z:.2f}m - very low!')
                self.get_logger().info('Consider increasing spawn altitude in launch file to z=0.5 or higher')
        self.get_logger().info('=== STEP 2: Initializing Nav2 ===')
        try:
            self.nav = BasicNavigator()
            if self.last_pose:
                init_pose = PoseStamped()
                init_pose.header.frame_id = self.last_pose.header.frame_id
                init_pose.header.stamp = self.get_clock().now().to_msg()
                init_pose.pose = self.last_pose.pose
                self.nav.setInitialPose(init_pose)
                self.get_logger().info(f'✓ Set initial pose in frame: {init_pose.header.frame_id}')
            self.get_logger().info('Waiting for Nav2 to become active...')
            if self.nav.waitUntilNav2Active(localizer=None):
                self.nav_ready = True
                self.get_logger().info('✓ Nav2 is ready!')
            else:
                self.get_logger().warn('⚠ Nav2 not ready. Will try direct control.')
        except Exception as e:
            self.get_logger().error(f'Nav2 initialization failed: {e}')
            self.get_logger().info('Will try direct velocity control instead.')
        self.get_logger().info('=== STEP 3: Ready for commands ===')
        self.get_logger().info("Commands:")
        self.get_logger().info("  'go'    - Move forward 10m using Nav2")
        self.get_logger().info("  'drive' - Move forward 10m using direct velocity")
        self.get_logger().info("  'up'    - Climb to 1m altitude")
        self.get_logger().info("  'stop'  - Stop all motion")
        self.get_logger().info("  'q'     - Quit")
        self._stop = False
        self._input_thread = threading.Thread(target=self._cli_loop, daemon=True)
        self._input_thread.start()
        while rclpy.ok() and not self._stop:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Shutting down.')
        if hasattr(self, 'nav'):
            self.nav.destroyNode()

    def _pose_cb(self, msg: PoseStamped):
        self.last_pose = msg

    def _odom_cb(self, msg: Odometry):
        if self.last_pose is None:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose.pose
            self.last_pose = pose_msg

    def _cli_loop(self):
        try:
            while not self._stop:
                cmd = input('> ').strip().lower()
                if cmd in ('q', 'quit', 'exit'):
                    self._stop = True
                elif cmd in ('go', 'nav'):
                    self._nav2_forward_10m()
                elif cmd in ('drive', 'direct'):
                    self._direct_forward_10m()
                elif cmd == 'up':
                    self._climb_to_altitude()
                elif cmd == 'stop':
                    self._stop_motion()
                elif cmd == '':
                    continue
                else:
                    print("Commands: 'go' (Nav2), 'drive' (direct), 'up' (climb), 'stop', 'q' (quit)")
        except (EOFError, KeyboardInterrupt):
            self._stop = True

    def _nav2_forward_10m(self):
        if not self.nav_ready:
            self.get_logger().warn('Nav2 not ready. Try \"drive\" for direct control.')
            return
        if self.last_pose is None:
            self.get_logger().warn('No pose data available.')
            return
        p = self.last_pose.pose.position
        o = self.last_pose.pose.orientation
        yaw = yaw_from_quat(o.x, o.y, o.z, o.w)
        dx = 10.0 * math.cos(yaw)
        dy = 10.0 * math.sin(yaw)
        goal = PoseStamped()
        goal.header.frame_id = self.last_pose.header.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = p.x + dx
        goal.pose.position.y = p.y + dy
        goal.pose.position.z = max(p.z, 1.0)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self.get_logger().info(f'Nav2 Goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}, {goal.pose.position.z:.2f})')
        try:
            self.nav.goToPose(goal)
            while not self.nav.isTaskComplete():
                fb = self.nav.getFeedback()
                if fb and hasattr(fb, 'distance_remaining'):
                    if fb.distance_remaining is not None:
                        self.get_logger().info(f"Distance remaining: {fb.distance_remaining:.2f}m")
                rclpy.spin_once(self, timeout_sec=0.5)
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('✓ Nav2 navigation succeeded!')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Nav2 goal was canceled.')
            else:
                self.get_logger().warn(f'Nav2 navigation failed: {result}')
        except Exception as e:
            self.get_logger().error(f'Nav2 navigation error: {e}')

    def _direct_forward_10m(self):
        if self.last_pose is None:
            self.get_logger().warn('No pose data available.')
            return
        p = self.last_pose.pose.position
        o = self.last_pose.pose.orientation
        initial_yaw = yaw_from_quat(o.x, o.y, o.z, o.w)
        target_x = p.x + 10.0 * math.cos(initial_yaw)
        target_y = p.y + 10.0 * math.sin(initial_yaw)
        self.get_logger().info(f'Direct control: Moving from ({p.x:.2f}, {p.y:.2f}) to ({target_x:.2f}, {target_y:.2f})')
        cmd = Twist()
        cmd.linear.x = 1.0  # 1 m/s forward
        cmd.linear.z = 0.1 if p.z < 1.0 else 0.0
        rate = self.create_rate(10)
        start_time = self.get_clock().now()
        try:
            while rclpy.ok():
                current_time = self.get_clock().now()
                elapsed = (current_time - start_time).nanoseconds / 1e9
                if elapsed > 12.0:
                    self.get_logger().info('Direct control timeout - stopping')
                    break
                if self.last_pose:
                    current_p = self.last_pose.pose.position
                    distance = math.sqrt((target_x - current_p.x)**2 + (target_y - current_p.y)**2)
                    if distance < 0.5:
                        self.get_logger().info('✓ Reached target!')
                        break
                    if elapsed % 2.0 < 0.1:
                        self.get_logger().info(f'Distance to target: {distance:.2f}m')
                self.cmd_vel_pub.publish(cmd)
                rate.sleep()
                rclpy.spin_once(self, timeout_sec=0.01)
        except KeyboardInterrupt:
            self.get_logger().info('Direct control interrupted')
        self._stop_motion()

    def _climb_to_altitude(self):
        if self.last_pose is None:
            self.get_logger().warn('No pose data available.')
            return
        current_z = self.last_pose.pose.position.z
        target_z = 1.0
        self.get_logger().info(f'Climbing from {current_z:.2f}m to {target_z:.2f}m')
        cmd = Twist()
        cmd.linear.z = 0.5
        rate = self.create_rate(10)
        start_time = self.get_clock().now()
        try:
            while rclpy.ok():
                current_time = self.get_clock().now()
                elapsed = (current_time - start_time).nanoseconds / 1e9
                if elapsed > 10.0:
                    break
                if self.last_pose:
                    current_z = self.last_pose.pose.position.z
                    if current_z >= target_z - 0.1:
                        self.get_logger().info('✓ Reached target altitude!')
                        break
                    if elapsed % 1.0 < 0.1:
                        self.get_logger().info(f'Current altitude: {current_z:.2f}m')
                self.cmd_vel_pub.publish(cmd)
                rate.sleep()
                rclpy.spin_once(self, timeout_sec=0.01)
        except KeyboardInterrupt:
            pass
        self._stop_motion()

    def _stop_motion(self):
        cmd = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('✓ Stopped motion')

def main():
    rclpy.init()
    try:
        MinimalDroneNavigator()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()