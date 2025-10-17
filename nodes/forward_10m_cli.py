#!/usr/bin/env python3
import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Return yaw (rad) from quaternion (x,y,z,w)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float):
    """Return quaternion (x,y,z,w) for yaw (rad), zero roll/pitch."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


class Forward10mCLI(Node):
    """
    Type 'go' in this node's terminal to send a Nav2 goal 10 m forward (x–y plane)
    relative to the current yaw. Prints the final pose when done.
    """

    def __init__(self):
        super().__init__('forward_10m_cli')

        # --- Subscribe to your 1 Hz pose relay ---
        qos_sub = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.last_pose: Optional[PoseStamped] = None
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone/pose_1hz', self._pose_cb, qos_sub
        )

        # --- Nav2 Commander interface ---
        self.nav = BasicNavigator()

        # Wait for first pose so we can seed Nav2 initial pose
        self.get_logger().info('Waiting for /drone/pose_1hz...')
        while rclpy.ok() and self.last_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        init = PoseStamped()
        init.header.frame_id = self.last_pose.header.frame_id  # usually 'odom'
        init.header.stamp = self.get_clock().now().to_msg()
        init.pose = self.last_pose.pose
        self.nav.setInitialPose(init)

        # Start CLI thread BEFORE waiting for Nav2, so user sees the prompt
        self._stop = False
        self._input_thread = threading.Thread(target=self._cli_loop, daemon=True)
        self._input_thread.start()

        self.get_logger().info('Bringing Nav2 to active state (this can take a few seconds)...')
        self.nav.waitUntilNav2Active()  # no timeout kwarg on Humble
        self.get_logger().info("Ready. Type 'go' to move forward 10 m; 'q' to quit.")

        # Spin until the user quits
        while rclpy.ok() and not self._stop:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Shutting down.')
        self.nav.destroyNode()

    # -------------------- Callbacks & helpers --------------------

    def _pose_cb(self, msg: PoseStamped):
        self.last_pose = msg

    def _cli_loop(self):
        try:
            while not self._stop:
                cmd = input('> ').strip().lower()
                if cmd in ('q', 'quit', 'exit'):
                    self._stop = True
                elif cmd in ('go', 'g', 'forward'):
                    self._do_forward_10m()
                elif cmd == '':
                    continue
                else:
                    print("Type 'go' to move forward 10 m, or 'q' to quit.")
        except (EOFError, KeyboardInterrupt):
            self._stop = True

    def _do_forward_10m(self):
        # Ensure Nav2 is actually ready (defensive)
        try:
            _ = self.nav.getFeedback()
        except Exception:
            self.get_logger().warn('Nav2 not ready yet — try again in a few seconds.')
            return

        if self.last_pose is None:
            self.get_logger().warn('No pose yet; try again.')
            return

        # Current pose and yaw
        p = self.last_pose.pose.position
        o = self.last_pose.pose.orientation
        yaw = yaw_from_quat(o.x, o.y, o.z, o.w)

        # 10 m ahead in x–y plane (keep same z)
        dx = 10.0 * math.cos(yaw)
        dy = 10.0 * math.sin(yaw)

        goal = PoseStamped()
        goal.header.frame_id = self.last_pose.header.frame_id  # keep same frame as relay (e.g., 'odom')
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = p.x + dx
        goal.pose.position.y = p.y + dy
        goal.pose.position.z = p.z
        qx, qy, qz, qw = quat_from_yaw(yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        self.get_logger().info(
            f"Sending goal 10 m ahead: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}, {goal.pose.position.z:.2f}) in [{goal.header.frame_id}]"
        )

        # Send and monitor
        self.nav.goToPose(goal)
        while not self.nav.isTaskComplete():
            fb = self.nav.getFeedback()
            if fb and fb.distance_remaining is not None:
                self.get_logger().info(f"Distance remaining: {fb.distance_remaining:.2f} m")
            rclpy.spin_once(self, timeout_sec=0.2)

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            # Grab the latest pose and print it out
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.last_pose:
                lp = self.last_pose.pose
                lyaw = yaw_from_quat(lp.orientation.x, lp.orientation.y, lp.orientation.z, lp.orientation.w)
                self.get_logger().info(
                    f"Reached pose:\n"
                    f"  x={lp.position.x:.3f}, y={lp.position.y:.3f}, z={lp.position.z:.3f}, yaw={math.degrees(lyaw):.1f}°"
                )
            else:
                self.get_logger().info("Reached goal, but no latest pose available.")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Goal was canceled.')
        else:
            self.get_logger().warn('Goal failed or timed out.')


def main():
    rclpy.init()
    try:
        Forward10mCLI()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
