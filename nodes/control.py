#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

# -------------------- Minimal quaternion → yaw helper --------------------
def _quat_to_yaw(qx, qy, qz, qw) -> float:
    # yaw (Z-axis rotation) from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def _yaw_rot2x2(yaw: float):
    c = math.cos(yaw); s = math.sin(yaw)
    return np.array([[c, -s],
                     [s,  c]])

GUI_TO_CMD = {
    "HOVER": "hover",
    "MOVE TO GOAL": "move_to_goal",
    "LAND": "land",
    "TAKEOFF": "takeoff",
    "EMERGENCY LAND": "emergency_land",
}

class FlightControl(Node):
    """
    Z-only flight control that:
    - Stages 2D goals from the GUI (/cmd/goal), and only sends a Nav2 action goal
      on 'MOVE TO GOAL' (/cmd/control).
    - Listens to /cmd_vel from Nav2, injects linear.z, republishes to /cmd_vel_real.
    - Maintains the state machine and status publishing for UX continuity.
    """

    def __init__(self):
        super().__init__('flight_control')

        # --- Publishers
        self.pub_status     = self.create_publisher(String, '/movement/status', 10)
        self.pub_cmd_vel    = self.create_publisher(Twist,  '/cmd_vel_real', 10)  # final bus
        self.pub_goal_dist  = self.create_publisher(Float32, '/goal/distance', 10)
        self.pub_goal_time  = self.create_publisher(Float32, '/goal/time', 10)

        # --- Subscribers (GUI control, goal, height; pose/odom; sector mins)
        self.sub_cmd    = self.create_subscription(String,       '/cmd/control',    self.on_cmd,    10)
        self.sub_goal   = self.create_subscription(PointStamped, '/cmd/goal',       self.on_goal,   10)
        self.sub_height = self.create_subscription(Float32,      '/cmd/height',     self.on_height, 10)
        self.sub_pose_s = self.create_subscription(PoseStamped,  '/drone/pose_1hz', self.on_pose,   10)
        self.sub_odom   = self.create_subscription(Odometry,     '/odometry',       self.on_odom,   10)
        self.sub_sectors= self.create_subscription(Float32MultiArray, '/sector_mins', self.on_sectors, 10)

        # --- Nav2 /cmd_vel input (best-effort typical)
        qos_cmd = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.sub_nav2_cmd = self.create_subscription(Twist, '/cmd_vel', self._on_nav2_cmd, qos_cmd)
        self.last_nav2_twist = Twist()  # cache of last Nav2 twist

        # --- Action client to Nav2
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # --- Parameters (defaults preserved)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('max_xy_speed',   1.5)  # used only for clamping if needed
        self.declare_parameter('max_z_up',       2.0)
        self.declare_parameter('max_z_down',     1.0)
        self.declare_parameter('kp_z',           0.65)
        self.declare_parameter('ki_z',           0.20)
        self.declare_parameter('pos_tol_xy',     0.25)
        self.declare_parameter('pos_tol_z',      0.15)

        self.ctrl_hz      = float(self.get_parameter('control_rate_hz').value)
        self.max_xy_speed = float(self.get_parameter('max_xy_speed').value)
        self.max_z_up     = float(self.get_parameter('max_z_up').value)
        self.max_z_down   = float(self.get_parameter('max_z_down').value)
        self.kp_z         = float(self.get_parameter('kp_z').value)
        self.ki_z         = float(self.get_parameter('ki_z').value)
        self.pos_tol_xy   = float(self.get_parameter('pos_tol_xy').value)
        self.pos_tol_z    = float(self.get_parameter('pos_tol_z').value)

        # --- Internal state
        self.status        = "Pre Flight Checks"
        self._last_status  = None
        self.last_cmd      = None

        self.target_height = None  # operator-selected hover/target height
        self.goal_xyz      = None  # (x,y,z) for UI/meters used for dist/time UX
        self.hover_z       = None

        self.current_pose  = None
        self.currentX      = None
        self.currentY      = None
        self.currentZ      = None

        self.sector_mins   = None  # still subscribed for potential UI/UX; not used for XY

        # Staged XY goal (from GUI); only sent to Nav2 when MOVE TO GOAL pressed
        self.staged_goal_xy = None

        # Z controller memory
        self._int_z          = 0.0
        self._int_z_max      = 1.0
        self._last_ctrl_time = self.get_clock().now()

        # Control loop
        self.timer = self.create_timer(1.0 / max(1.0, self.ctrl_hz), self.main_loop)

        self.set_status(self.status)
        self.get_logger().info("FlightControl (Z-inject, Nav2 XY) ready.")

    # -------------------- Utilities --------------------
    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def set_status(self, s: str):
        if s != self._last_status:
            self._last_status = s
            self.status = s
            self.pub_status.publish(String(data=s))
            self.get_logger().info(f"[status] {s}")

    def zero_twist(self):
        tw = Twist()
        self.pub_cmd_vel.publish(tw)

    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        return _quat_to_yaw(q.x, q.y, q.z, q.w)

    # -------------------- Z control --------------------
    def _vz_hold(self, target_z: float) -> float:
        if self.currentZ is None:
            return 0.0
        now = self.get_clock().now()
        dt  = (now - self._last_ctrl_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.ctrl_hz
        self._last_ctrl_time = now

        e = float(target_z - self.currentZ)
        self._int_z += e * dt
        if self.ki_z > 0.0:
            max_i = self._int_z_max / self.ki_z
            self._int_z = self.clamp(self._int_z, -max_i, max_i)

        vz = self.kp_z * e + self.ki_z * self._int_z
        if vz >= 0.0:
            vz = self.clamp(vz, 0.0, self.max_z_up)
        else:
            vz = self.clamp(vz, -self.max_z_down, 0.0)
        return float(vz)

    # -------------------- Nav2 cmd_in → z inject → cmd_out --------------------
    def _on_nav2_cmd(self, msg: Twist):
        self.last_nav2_twist = msg  # cache

    def _publish_with_injected_vz(self, vz: float):
        out = Twist()
        # Pass-through XY + yaw from Nav2:
        out.linear.x  = self.last_nav2_twist.linear.x
        out.linear.y  = self.last_nav2_twist.linear.y
        out.angular.z = self.last_nav2_twist.angular.z
        # Inject altitude channel:
        out.linear.z  = float(vz)
        self.pub_cmd_vel.publish(out)

    # -------------------- State machine --------------------
    def main_loop(self):
        if self.status == 'Pre Flight Checks':
            self.pre_flight_checks()
        elif self.status == 'Landed':
            self.landed()
        elif self.status == 'Arrived at Goal':
            self.arrived_at_goal()
        elif self.status == 'Taking off':
            self.taking_off()
        elif self.status == 'Landing':
            self.landing()
        elif self.status == 'Hovering':
            self.hovering()
        elif self.status == 'Moving to Goal':
            self.moving_to_goal()
        elif self.status == 'Emergency Landing':
            self.emergency_landing()
        else:
            # Fallback: keep safe
            self.zero_twist()

    def pre_flight_checks(self):
        # Hold zero outputs but remain responsive to height/goal staging
        self.zero_twist()
        if self.last_cmd == 'takeoff':
            self.set_status('Taking off')

    def landed(self):
        self.zero_twist()
        # allow immediate transitions
        if self.last_cmd == 'takeoff':
            self.set_status('Taking off')
        elif self.last_cmd == 'move_to_goal':
            self.set_status('Taking off')
        elif self.last_cmd == 'emergency_land':
            self.set_status('Emergency Landing')

    def taking_off(self):
        tgt = self.target_height if self.target_height is not None else 2.0
        if self.currentZ is None:
            self.zero_twist()
            return
        # Once close enough, switch to Hover or Moving to Goal
        if abs(tgt - self.currentZ) <= self.pos_tol_z:
            if self.last_cmd == 'move_to_goal' and self.staged_goal_xy is not None:
                self.set_status('Moving to Goal')
            else:
                self.set_status('Hovering')
            self._publish_with_injected_vz(self._vz_hold(tgt))
            return
        self._publish_with_injected_vz(self._vz_hold(tgt))

    def hovering(self):
        # Remember the hover_z (first time)
        if self.hover_z is None and self.currentZ is not None:
            self.hover_z = float(self.currentZ)
        tgt_z = self.hover_z if self.hover_z is not None else (self.currentZ or 0.0)
        self._publish_with_injected_vz(self._vz_hold(tgt_z))

        if self.last_cmd == 'move_to_goal' and self.staged_goal_xy is not None:
            self.set_status('Moving to Goal')
        elif self.last_cmd == 'land':
            self.set_status('Landing')
        elif self.last_cmd == 'emergency_land':
            self.set_status('Emergency Landing')

    def moving_to_goal(self):
        # XY path is owned by Nav2. We only control Z to a sensible target.
        if self.currentX is None:
            self._publish_with_injected_vz(0.0)
            return

        # Choose Z to hold: goal z if specified, otherwise operator target height, else current
        if self.goal_xyz is not None:
            gz = self.goal_xyz[2]
        elif self.target_height is not None:
            gz = self.target_height
        elif self.currentZ is not None:
            gz = self.currentZ
        else:
            gz = 0.0

        # Consider arrival when horizontally close to staged goal (if any)
        if self.staged_goal_xy is not None:
            gx, gy = self.staged_goal_xy
            dx = gx - self.currentX
            dy = gy - self.currentY
            dist_xy = math.hypot(dx, dy)
            self.pub_goal_dist.publish(Float32(data=float(dist_xy)))
            # crude ETA not available without planner feedback; publish 0 for now
            self.pub_goal_time.publish(Float32(data=0.0))
            if dist_xy <= self.pos_tol_xy and abs((self.currentZ or gz) - gz) <= self.pos_tol_z:
                self.set_status('Arrived at Goal')

        self._publish_with_injected_vz(self._vz_hold(gz))

        if self.last_cmd == 'land':
            self.set_status('Landing')
        elif self.last_cmd == 'emergency_land':
            self.set_status('Emergency Landing')

    def arrived_at_goal(self):
        gz = self.goal_xyz[2] if self.goal_xyz is not None else (self.currentZ or 0.0)
        vz = self._vz_hold(gz)
        self._publish_with_injected_vz(vz)
        if self.last_cmd == 'land':
            self.set_status('Landing')

    def landing(self):
        # Simple vertical descent to ~0
        land_z = 0.10
        if self.currentZ is None:
            self.zero_twist()
            return
        if self.currentZ <= (land_z + self.pos_tol_z):
            self.zero_twist()
            self.set_status('Landed')
            return
        vz = self._vz_hold(land_z)
        self._publish_with_injected_vz(vz)

    def emergency_landing(self):
        if self.currentZ is None:
            self.zero_twist()
            return
        vz = self._vz_hold(-5.0)  # strong down target
        self._publish_with_injected_vz(vz)
        if self.currentZ <= 0.15:
            self.zero_twist()
            self.set_status('Landed')

    # -------------------- Callbacks --------------------
    def on_cmd(self, msg: String):
        text = msg.data.strip().upper()
        self.get_logger().info(f"/cmd/control: {text}")
        if text in GUI_TO_CMD:
            self.last_cmd = GUI_TO_CMD[text]
        else:
            self.last_cmd = text.lower()

        if self.last_cmd == 'takeoff':
            self.hover_z = None
            self.set_status('Taking off')

        elif self.last_cmd == 'move_to_goal':
            self.hover_z = None
            # if airborne, go straight to Moving; else via Taking off
            if self.currentZ is not None and self.currentZ > 0.3:
                self.set_status('Moving to Goal')
            else:
                self.set_status('Taking off')

            # Send Nav2 goal if we have one staged
            if self.staged_goal_xy is None:
                self.get_logger().warn("MOVE TO GOAL pressed but no staged goal set.")
            else:
                self._send_nav2_goal(self.staged_goal_xy[0], self.staged_goal_xy[1])

        elif self.last_cmd == 'hover':
            if self.currentZ is not None:
                self.hover_z = float(self.currentZ)
            self.set_status('Hovering')

        elif self.last_cmd == 'land':
            self.set_status('Landing')

        elif self.last_cmd == 'emergency_land':
            self.set_status('Emergency Landing')

    def _send_nav2_goal(self, gx: float, gy: float):
        if not self.nav2_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Nav2 action server not available yet.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(gx)
        goal_msg.pose.pose.position.y = float(gy)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending Nav2 goal → map: ({gx:.2f}, {gy:.2f})")
        send_future = self.nav2_client.send_goal_async(goal_msg)

        def _goal_response(fut):
            goal_handle = fut.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn("Nav2 goal rejected.")
                return
            self.get_logger().info("Nav2 goal accepted.")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_goal_result)

        def _goal_result(fut):
            try:
                _ = fut.result().result
                self.get_logger().info("Nav2 goal result received.")
            except Exception as e:
                self.get_logger().warn(f"Nav2 goal result error: {e}")

        send_future.add_done_callback(_goal_response)

    def on_goal(self, msg: PointStamped):
        # Stage X,Y; keep a Z for UX/metrics based on target height or currentZ
        gx = float(msg.point.x)
        gy = float(msg.point.y)
        gz = self.target_height if self.target_height is not None else (self.currentZ or 0.0)

        self.staged_goal_xy = (gx, gy)
        self.goal_xyz = (gx, gy, float(gz))
        self._update_goal_metrics()
        self.get_logger().info(f"/cmd/goal staged: (x={gx:.2f}, y={gy:.2f}, z={gz:.2f})")

    def on_height(self, msg: Float32):
        self.target_height = float(msg.data)
        self.get_logger().info(f"/cmd/height: {self.target_height:.2f} m")
        # Keep hover_z in sync if already hovering
        if self.status == 'Hovering':
            self.hover_z = self.target_height

    def on_pose(self, msg: PoseStamped):
        self.current_pose = msg.pose
        self.currentX = msg.pose.position.x
        self.currentY = msg.pose.position.y
        self.currentZ = msg.pose.position.z
        self._update_goal_metrics()

    def on_odom(self, msg: Odometry):
        # Keep pose continuously fresh (faster than the GUI pose topic)
        self.current_pose = msg.pose.pose
        self.currentX = msg.pose.pose.position.x
        self.currentY = msg.pose.pose.position.y
        self.currentZ = msg.pose.pose.position.z
        self._update_goal_metrics()

    def on_sectors(self, msg: Float32MultiArray):
        # Not used for XY anymore (Nav2 owns that), but keep for UI/UX scope
        self.sector_mins = np.array(msg.data, dtype=float) if len(msg.data) else None

    def _update_goal_metrics(self):
        # Publishes distance (horizontal) to goal for the GUI; ETA left as 0
        if self.currentX is None:
            return
        if self.goal_xyz is None:
            return
        gx, gy, gz = self.goal_xyz
        dx = gx - self.currentX
        dy = gy - self.currentY
        dist_xy = math.hypot(dx, dy)
        self.pub_goal_dist.publish(Float32(data=float(dist_xy)))
        # Without planner progress time, publish 0
        self.pub_goal_time.publish(Float32(data=0.0))

# -------------------- main --------------------
def main(args=None):
    rclpy.init(args=args)
    node = FlightControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.zero_twist()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
