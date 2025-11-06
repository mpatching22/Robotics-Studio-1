#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from std_srvs.srv import Trigger                 # + NEW
from action_msgs.msg import GoalStatusArray      # + NEW

from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo  # (needed to build request)

import os, csv, time

DEFAULT_HEIGHT_M = 0.75  # or keep your preferred default

def _desired_height(target_height: float | None) -> float:
    return target_height if target_height is not None else DEFAULT_HEIGHT_M


# -------------------- GUI command map --------------------
GUI_TO_CMD = {
    "HOVER": "hover",
    "MOVE TO GOAL": "move_to_goal",
    "LAND": "land",
    "TAKEOFF": "takeoff",
    "EMERGENCY LAND": "emergency_land",
    "START LOG": "start_log",
    "STOP LOG": "stop_log",
}

# -------------------- Defaults / Params --------------------
DEFAULT_TAKEOFF_HEIGHT = 0.75     # per your spec
DEFAULT_OUTPUT_CMD_TOPIC = '/cmd_vel_real'
NAV2_CMD_TOPIC = '/cmd_vel'
STATUS_TOPIC = '/movement/status'

# -------------------- Helpers --------------------
def _quat_to_yaw(qx, qy, qz, qw) -> float:
    # yaw (Z-axis rotation) from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class FlightControl(Node):
    """
    XY by Nav2, custom Z control:
      - Send goal to Nav2 on 'MOVE TO GOAL'
      - Subscribe to Nav2 /cmd_vel, inject linear.z, publish to /vel_cmd_real
      - No lidar_perception_360 dependency (Nav2 + SLAM own the XY)
      - State machine with per-state methods (clean structure kept)
      - CSV logger on demand
    """

    # -------------------- init --------------------
    def __init__(self):
        super().__init__('flight_control')

        # ---- publishers
        self.pub_status    = self.create_publisher(String, STATUS_TOPIC, 10)
        self.pub_cmd_real  = self.create_publisher(Twist, DEFAULT_OUTPUT_CMD_TOPIC, 10)
        self.pub_goal_dist = self.create_publisher(Float32, '/goal/distance', 10)
        self.pub_goal_time = self.create_publisher(Float32, '/goal/time', 10)

        # ---- subscribers (GUI + pose)
        self.sub_cmd    = self.create_subscription(String,       '/cmd/control',    self.on_cmd,    10)
        self.sub_goal   = self.create_subscription(PointStamped, '/cmd/goal',       self.on_goal,   10)
        self.sub_height = self.create_subscription(Float32,      '/cmd/height',     self.on_height, 10)
        self.sub_pose_s = self.create_subscription(PoseStamped,  '/drone/pose_1hz', self.on_pose,   10)
        self.sub_odom   = self.create_subscription(Odometry,     '/odometry',       self.on_odom,   10)

        # ---- Nav2 incoming velocity (BEST_EFFORT typical)
        qos_cmd = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        self.sub_nav2_cmd = self.create_subscription(Twist, NAV2_CMD_TOPIC, self._on_nav2_cmd, qos_cmd)
        self.last_nav2_twist = Twist()

        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # NEW: CancelGoal client for when we don't have a handle yet
        self.cancel_nav2_cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel')


        # --- Lifecycle "is_active" service clients (non-blocking)
        self.cli_nav_active = self.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        self.cli_loc_active = self.create_client(Trigger, '/lifecycle_manager_localization/is_active')  # ok if absent

        # Readiness flags flipped by async poller
        self._nav_ready = False
        self._loc_ready = True   # default to True so we don't block if there's no localization manager

        # Poll readiness once per second (non-blocking)
        self._readiness_timer = self.create_timer(1.0, self._poll_readiness)

        # Nav2 NavigateToPose status subscription (to detect SUCCEEDED=4)
        self.sub_nav_status = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._on_nav_status, 10
        )
        self._latest_nav_status = None

        # --- Terrain HAG topics (for precise terrain-following Z)
        self.sub_hag      = self.create_subscription(Float32, '/altitude/hag',         self.on_hag,        10)
        self.sub_hag_fwd  = self.create_subscription(Float32, '/altitude/hag_forward', self.on_hag_forward,10)

        # Store latest HAGs
        self.hag = None
        self.hag_forward = None

        # --- CSV trace state (original style)
        self.record_active   = False
        self.record_rows     = []     # list of (x, y, ground_z)
        self._last_record_t  = 0.0
        self._record_rate_hz = 5.0    # sample rate while Moving to Goal

        # ---- parameters
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('max_z_up',       2.0)
        self.declare_parameter('max_z_down',     1.0)
        self.declare_parameter('kp_z',           0.65)
        self.declare_parameter('ki_z',           0.20)
        self.declare_parameter('pos_tol_xy',     0.25)
        self.declare_parameter('pos_tol_z',      0.15)

        self.ctrl_hz    = float(self.get_parameter('control_rate_hz').value)
        self.max_z_up   = float(self.get_parameter('max_z_up').value)
        self.max_z_down = float(self.get_parameter('max_z_down').value)
        self.kp_z       = float(self.get_parameter('kp_z').value)
        self.ki_z       = float(self.get_parameter('ki_z').value)
        self.pos_tol_xy = float(self.get_parameter('pos_tol_xy').value)
        self.pos_tol_z  = float(self.get_parameter('pos_tol_z').value)

        # ---- state / memory
        self.status        = "Pre Flight Checks"
        self._last_status  = None
        self.last_cmd      = None

        self.target_height = None
        self.goal_xyz      = None  # (gx, gy, gz)
        self.staged_goal_xy= None  # (gx, gy)

        self.currentX = None
        self.currentY = None
        self.currentZ = None

        self.hover_z = None

        # PID memory for Z hold
        self._int_z = 0.0
        self._int_z_max = 1.0
        self._last_ctrl_time = self.get_clock().now()

        # Nav2 goal tracking
        self._nav2_goal_handle = None
        self._nav2_goal_active = False

        # CSV logging
        self.record_active = False
        self._record_rate_hz = 10.0
        self._last_record_t = 0.0
        self._csv_fp = None
        self._csv_writer = None
        self._log_dir = Path.home() / '.ros' / 'trailblazer_logs'
        self._log_dir.mkdir(parents=True, exist_ok=True)

        # loop + heartbeat
        self.timer = self.create_timer(1.0 / max(1.0, self.ctrl_hz), self.main_loop)
        self._status_heartbeat = self.create_timer(3.0, self._republish_status)

        self.set_status(self.status)
        self.get_logger().info("FlightControl ready (Nav2 XY + custom Z → /vel_cmd_real).")

    # -------------------- utilities --------------------
    def _republish_status(self):
        self.pub_status.publish(String(data=self.status))

    def set_status(self, s: str):
        prev = getattr(self, "_last_status", None)
        if s != prev:
            # leaving Moving to Goal → write CSV once
            if prev == 'Moving to Goal' and s != 'Moving to Goal':
                # consider 'Arrived at Goal' as reached=True, anything else still writes file
                reached = (s == 'Arrived at Goal')
                self._save_trace_csv(reached=reached)

            self._last_status = s
            self.status = s
            self.pub_status.publish(String(data=s))
            self.get_logger().info(f"[status] {s}")


    def zero_twist(self):
        self.pub_cmd_real.publish(Twist())

    def _desired_abs_z(self) -> float:
        """
        Desired absolute altitude using (ground_z_ahead + target_height).
        Fallbacks:
        - if no forward HAG: use current ground_z (z - hag)
        - if no HAG at all:  use absolute target_height (legacy)
        """
        tgt_hag = _desired_height(self.target_height)

        if self.currentZ is None:
            return tgt_hag  # can't do better yet

        # Prefer forward preview (terrain look-ahead)
        if self.hag_forward is not None and self.hag_forward == self.hag_forward:  # NaN-safe
            ground_z_ahead = self.currentZ - float(self.hag_forward)
            return ground_z_ahead + tgt_hag

        # Else current ground
        if self.hag is not None and self.hag == self.hag:
            ground_z_now = self.currentZ - float(self.hag)
            return ground_z_now + tgt_hag

        # No HAG at all → legacy absolute height
        return tgt_hag


    def _vz_hold(self, target_z: float) -> float:
        """PI-like altitude hold (safe clamped vz)."""
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
            self._int_z = clamp(self._int_z, -max_i, max_i)

        vz = self.kp_z * e + self.ki_z * self._int_z
        if vz >= 0.0:
            vz = clamp(vz, 0.0, self.max_z_up)
        else:
            vz = clamp(vz, -self.max_z_down, 0.0)
        return float(vz)

    def _on_nav2_cmd(self, msg: Twist):
        self.last_nav2_twist = msg

    def _publish_with_injected_vz(self, vz: float):
        out = Twist()
        out.linear.x  = self.last_nav2_twist.linear.x
        out.linear.y  = self.last_nav2_twist.linear.y
        out.angular.z = self.last_nav2_twist.angular.z
        out.linear.z  = float(vz)
        self.pub_cmd_real.publish(out)

    def _publish_manual(self, vx: float, vy: float, vz: float, wz: float):
        out = Twist()
        out.linear.x = float(vx)
        out.linear.y = float(vy)
        out.linear.z = float(vz)
        out.angular.z = float(wz)
        self.pub_cmd_real.publish(out)

    def _cancel_nav2_goal(self):
        """Robust cancel:
        - If we have a goal handle, cancel that specific goal.
        - Otherwise, call the CancelGoal service with an empty GoalInfo to cancel any goal.
        """
        # Case A: have handle → cancel via action handle
        if self._nav2_goal_handle is not None:
            self.get_logger().info("Cancelling Nav2 goal via goal handle…")
            cancel_future = self._nav2_goal_handle.cancel_goal_async()

            def _canceled(_):
                self.get_logger().info("Nav2 goal cancel completed (handle).")
                self._nav2_goal_active = False
                self._nav2_goal_handle = None
                self.last_nav2_twist = Twist()
            cancel_future.add_done_callback(_canceled)
            return

        # Case B: no handle yet → cancel via CancelGoal service (cancel any goal)
        if self.cancel_nav2_cli.service_is_ready():
            self.get_logger().info("Cancelling Nav2 goal via CancelGoal service (no handle)…")
            req = CancelGoal.Request()
            # Empty GoalInfo => cancel all goals for this action server
            req.goal_info = GoalInfo()  # default zeros (uuid all zeros, stamp 0) cancels any
            fut = self.cancel_nav2_cli.call_async(req)

            def _canceled_service(_):
                self.get_logger().info("Nav2 CancelGoal service responded.")
                self._nav2_goal_active = False
                self._nav2_goal_handle = None
                self.last_nav2_twist = Twist()
            fut.add_done_callback(_canceled_service)
        else:
            self.get_logger().warn("CancelGoal service not ready; deferring cancel.")


    # + NEW
    def _on_nav_status(self, msg: GoalStatusArray):
        """Track the most recent goal status from Nav2. Values:
        0 UNKNOWN, 1 ACCEPTED, 2 EXECUTING, 3 CANCELING, 4 SUCCEEDED, 5 CANCELED, 6 ABORTED
        """
        if not msg.status_list:
            return
        last = msg.status_list[-1]
        self._latest_nav_status = int(last.status)
        # Treat EXECUTING as 'active', SUCCEEDED as 'reached'
        self._have_active_goal = (self._latest_nav_status == 2)
        if self._latest_nav_status == 4:
            # SUCCEEDED
            self.get_logger().info("Nav2 reports goal SUCCEEDED via status stream.")
            # We'll let moving_to_goal() transition state, to keep logic centralized.

    def _poll_readiness(self):
        # Query Nav2 is_active without blocking the executor
        if self.cli_nav_active.service_is_ready():
            fut = self.cli_nav_active.call_async(Trigger.Request())
            fut.add_done_callback(self._on_nav_ready)

        # Optional localization manager: only if present; otherwise we keep _loc_ready True
        if self.cli_loc_active.service_is_ready():
            fut2 = self.cli_loc_active.call_async(Trigger.Request())
            fut2.add_done_callback(self._on_loc_ready)

    def _on_nav_ready(self, fut):
        try:
            res = fut.result()
            self._nav_ready = bool(res.success)
        except Exception:
            self._nav_ready = False

    def _on_loc_ready(self, fut):
        try:
            res = fut.result()
            self._loc_ready = bool(res.success)
        except Exception:
            self._loc_ready = True  # don't block if manager is flaky/missing

    def _on_nav_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        self._latest_nav_status = int(msg.status_list[-1].status)  # 4 == SUCCEEDED
    # -------------------- state machine --------------------
    def main_loop(self):
        if   self.status == 'Pre Flight Checks':  self.pre_flight_checks()
        elif self.status == 'Taking off':         self.taking_off()
        elif self.status == 'Moving to Goal':     self.moving_to_goal()
        elif self.status == 'Hovering':           self.hovering()
        elif self.status == 'Landing':            self.landing()
        elif self.status == 'Emergency Landing':  self.emergency_landing()
        elif self.status == 'Arrived at Goal':    self.arrived_at_goal()
        elif self.status == 'Landed':             self.landed()
        else:
            self.zero_twist()

        # lightweight CSV recorder (time-based)
        self._maybe_record_row()

    def pre_flight_checks(self):
        # Leave pre-flight only when Nav2 (and, if present, localization) are active
        if self._nav_ready and self._loc_ready:
            self.set_status("Hovering")
        else:
            self.zero_twist()  # stay idle; async poller keeps updating flags



    def landed(self):
        self.zero_twist()
        if self.last_cmd == 'takeoff':
            self.set_status('Taking off')
        elif self.last_cmd == 'move_to_goal':
            self.set_status('Taking off')
        elif self.last_cmd == 'emergency_land':
            self.set_status('Emergency Landing')

    def taking_off(self):
        # Terrain-following target (uses hag_forward > hag > absolute height)
        gz = self._desired_abs_z()
        vz = self._vz_hold(gz)

        # Close enough? transition to goal or hover
        if self.currentZ is not None and abs(gz - self.currentZ) <= self.pos_tol_z:
            if self.last_cmd == 'move_to_goal' and self.staged_goal_xy is not None:
                if self.nav2_client.wait_for_server(timeout_sec=0.5):
                    self._send_nav2_goal(*self.staged_goal_xy)
                    self.set_status('Moving to Goal')
                else:
                    self.get_logger().warn("NavigateToPose server not ready yet")
            else:
                self.set_status('Hovering')

        # Keep climbing/holding using injected Z
        self._publish_with_injected_vz(vz)


    def hovering(self):
        # Hold terrain-following target while idle
        gz = self._desired_abs_z()
        vz = self._vz_hold(gz)
        self._publish_with_injected_vz(vz)

        # Command-driven transitions
        if self.last_cmd == 'move_to_goal' and self.staged_goal_xy is not None:
            if self.nav2_client.wait_for_server(timeout_sec=0.5):
                self._send_nav2_goal(*self.staged_goal_xy)
                self.set_status('Moving to Goal')
            else:
                self.get_logger().warn("NavigateToPose server not ready yet.")
        elif self.last_cmd == 'land':
            self.set_status('Landing')
        elif self.last_cmd == 'emergency_land':
            self.set_status('Emergency Landing')


    def moving_to_goal(self):
        # Start/continue recording during this state
        self.record_active = True

        # --- Terrain-following target Z ---
        gz = self._desired_abs_z()
        vz = self._vz_hold(gz)

        # --- Append CSV sample at fixed rate ---
        now = self.get_clock().now().nanoseconds / 1e9
        if (self.record_active
            and self.currentX is not None
            and self.currentY is not None
            and self.currentZ is not None):
            if (now - self._last_record_t) >= (1.0 / self._record_rate_hz):
                # ground_z = z - HAG (prefer forward HAG)
                if self.hag_forward is not None and self.hag_forward == self.hag_forward:
                    ground_z = self.currentZ - float(self.hag_forward)
                elif self.hag is not None and self.hag == self.hag:
                    ground_z = self.currentZ - float(self.hag)
                else:
                    ground_z = float('nan')
                self.record_rows.append((float(self.currentX), float(self.currentY), float(ground_z)))
                self._last_record_t = now

        # --- Nav2-driven arrival: only when SUCCEEDED ---
        if getattr(self, "_latest_nav_status", None) == 4:
            self._cancel_nav2_goal()
            self.set_status('Arrived at Goal')
            # hold altitude while state flips next tick
            self._publish_with_injected_vz(self._vz_hold(gz))
            return

        # --- Keep flying: inject Z into Nav2 XY ---
        self._publish_with_injected_vz(vz)

        # --- Command overrides while en route ---
        if self.last_cmd == 'hover':
            self._cancel_nav2_goal(); self.set_status('Hovering')
        elif self.last_cmd == 'land':
            self._cancel_nav2_goal(); self.set_status('Landing')
        elif self.last_cmd == 'emergency_land':
            self._cancel_nav2_goal(); self.set_status('Emergency Landing')



    def arrived_at_goal(self):
        # Hover at terrain-following target at the goal
        gz = self._desired_abs_z()
        vz = self._vz_hold(gz)
        self._publish_with_injected_vz(vz)

        # Commands from here
        if self.last_cmd == 'land':
            self.set_status('Landing')
        elif self.last_cmd == 'hover':
            self.set_status('Hovering')


    def landing(self):
        land_z = 0.10
        if self.currentZ is None:
            self.zero_twist()
            return
        if self.currentZ <= (land_z + self.pos_tol_z):
            self.zero_twist()
            self.set_status('Landed')
            return
        self._publish_with_injected_vz(self._vz_hold(land_z))

    def emergency_landing(self):
        # aggressive down target (controller clamps to max_z_down)
        if self.currentZ is None:
            self.zero_twist()
            return
        self._publish_with_injected_vz(self._vz_hold(-5.0))
        if self.currentZ <= 0.15:
            self.zero_twist()
            self.set_status('Landed')

    # -------------------- callbacks --------------------
    def on_cmd(self, msg: String):
        text = msg.data.strip().upper()
        self.get_logger().info(f"/cmd/control: {text}")
        self.last_cmd = GUI_TO_CMD.get(text, text.lower())

        if self.last_cmd == 'takeoff':
            self.hover_z = None
            self.set_status('Taking off')

        elif self.last_cmd == 'move_to_goal':
            self.hover_z = None
            if self.currentZ is not None and self.currentZ > 0.3:
                self.set_status('Moving to Goal')
            else:
                self.set_status('Taking off')
            if self.staged_goal_xy is None:
                self.get_logger().warn("MOVE TO GOAL pressed but no staged goal set.")
            else:
                if self.nav2_client.wait_for_server(timeout_sec=0.5):
                    self._send_nav2_goal(*self.staged_goal_xy)
                else:
                    self.get_logger().warn("NavigateToPose server not ready yet.")

        elif self.last_cmd == 'hover':
            if self.status == 'Moving to Goal' and self._nav2_goal_active:
                self._cancel_nav2_goal()
            self.set_status('Hovering')

        elif self.last_cmd == 'land':
            if self.status == 'Moving to Goal' and self._nav2_goal_active:
                self._cancel_nav2_goal()
            self.set_status('Landing')

        elif self.last_cmd == 'emergency_land':
            if self._nav2_goal_active:
                self._cancel_nav2_goal()
            self.set_status('Emergency Landing')

        elif self.last_cmd == 'start_log':
            self._start_logging()

        elif self.last_cmd == 'stop_log':
            self._stop_logging()

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
                self._nav2_goal_active = False
                self._nav2_goal_handle = None
                return
            self.get_logger().info("Nav2 goal accepted.")
            self._nav2_goal_handle = goal_handle
            self._nav2_goal_active = True
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_goal_result)

        def _goal_result(fut):
            try:
                _ = fut.result().result
                self.get_logger().info("Nav2 goal result received.")
            except Exception as e:
                self.get_logger().warn(f"Nav2 goal result error: {e}")
            self._nav2_goal_active = False
            self._nav2_goal_handle = None

        send_future.add_done_callback(_goal_response)

    def on_goal(self, msg: PointStamped):
        gx = float(msg.point.x)
        gy = float(msg.point.y)
        gz = self.target_height if self.target_height is not None else DEFAULT_TAKEOFF_HEIGHT

        self.staged_goal_xy = (gx, gy)
        self.goal_xyz = (gx, gy, float(gz))
        self._publish_goal_metrics()
        self.get_logger().info(f"/cmd/goal staged: (x={gx:.2f}, y={gy:.2f}, z={gz:.2f})")

    def on_height(self, msg: Float32):
        self.target_height = float(msg.data)
        self.get_logger().info(f"/cmd/height: {self.target_height:.2f} m")
        if self.status == 'Hovering':
            self.hover_z = self.target_height  # live adjust

    def on_pose(self, msg: PoseStamped):
        self.currentX = msg.pose.position.x
        self.currentY = msg.pose.position.y
        self.currentZ = msg.pose.position.z
        self._publish_goal_metrics()

    def on_odom(self, msg: Odometry):
        self.currentX = msg.pose.pose.position.x
        self.currentY = msg.pose.pose.position.y
        self.currentZ = msg.pose.pose.position.z
        self._publish_goal_metrics()

    def on_hag(self, msg: Float32):
        self.hag = float(msg.data)

    def on_hag_forward(self, msg: Float32):
        self.hag_forward = float(msg.data)


    def _publish_goal_metrics(self):
        if self.currentX is None or self.goal_xyz is None:
            return
        gx, gy, _ = self.goal_xyz
        dx = gx - self.currentX
        dy = gy - self.currentY
        dist_xy = math.hypot(dx, dy)
        # simple placeholder ETA ~= distance
        self.pub_goal_dist.publish(Float32(data=float(dist_xy)))
        self.pub_goal_time.publish(Float32(data=float(max(0.0, dist_xy))))

    # -------------------- CSV logging --------------------
    def _start_logging(self):
        if self.record_active:
            self.get_logger().info("Logging already active.")
            return
        ts = time.strftime("%Y%m%d-%H%M%S")
        path = self._log_dir / f"log_{ts}.csv"
        self._csv_fp = open(path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_fp)
        self._csv_writer.writerow(["time_s","x","y","z","vx","vy","wz","state","status"])
        self._last_record_t = 0.0
        self.record_active = True
        self.get_logger().info(f"Logging → {path}")

    def _stop_logging(self):
        if not self.record_active:
            self.get_logger().info("Logging already stopped.")
            return
        try:
            self._csv_fp.flush()
            self._csv_fp.close()
        except Exception:
            pass
        self._csv_fp = None
        self._csv_writer = None
        self.record_active = False
        self.get_logger().info("Logging stopped.")

    def _maybe_record_row(self):
        if not self.record_active or self._csv_writer is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_record_t) < (1.0 / self._record_rate_hz):
            return
        self._last_record_t = now

        x = float(self.currentX) if self.currentX is not None else float('nan')
        y = float(self.currentY) if self.currentY is not None else float('nan')
        z = float(self.currentZ) if self.currentZ is not None else float('nan')
        vx = float(self.last_nav2_twist.linear.x)
        vy = float(self.last_nav2_twist.linear.y)
        wz = float(self.last_nav2_twist.angular.z)
        self._csv_writer.writerow([f"{now:.3f}", x, y, z, vx, vy, wz, self.status, self._last_status or ""])
        # avoid excessive disk IO
        try:
            self._csv_fp.flush()
        except Exception:
            pass
    
    def _save_trace_csv(self, reached: bool):
        try:
            self.record_active = False
            if not self.record_rows:
                self.get_logger().info("[trace] no samples recorded; skipping CSV write")
                return

            outdir = os.path.join(os.getcwd(), 'data')
            os.makedirs(outdir, exist_ok=True)

            # Name by goal (or last sample if goal unknown)
            if self.goal_xyz is not None:
                gx, gy, _ = self.goal_xyz
            else:
                gx, gy, _gz = self.record_rows[-1]

            fname = f"x{round(gx,2)}y{round(gy,2)}.csv"
            path = os.path.join(outdir, fname.replace(' ', ''))

            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['x', 'y', 'height'])  # height = ground_z
                for x, y, ground_z in self.record_rows:
                    # retain your 3-decimal formatting from the sample code
                    w.writerow([f"{x:.3f}", f"{y:.3f}", f"{ground_z:.3f}"])

            self.get_logger().info(f"[trace] wrote {len(self.record_rows)} samples to {path}")
        except Exception as e:
            self.get_logger().error(f"[trace] failed to write CSV: {e}")
        finally:
            self.record_rows = []

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
        node._stop_logging()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
