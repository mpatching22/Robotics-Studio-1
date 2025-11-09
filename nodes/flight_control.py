#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file flight_control.py
@brief Trailblazer flight controller: Nav2 for XY, custom Z (terrain-following), CSV logging.

@details
This node fuses Nav2/SLAM XY control with a custom altitude controller that follows terrain
using down-facing LiDAR (HAG) and an optional forward-looking HAG preview window. The node:

- Accepts high-level commands from the GUI (e.g., TAKEOFF, MOVE TO GOAL, HOVER, LAND).
- Sends goals to Nav2 (NavigateToPose), listens to Nav2 `/cmd_vel` and injects `linear.z`.
- Publishes the final velocity command on `/cmd_vel_real`.
- Tracks status with a simple state machine (Pre-flight → Landed → Taking off → …).
- Records CSV logs and also writes a compact “trace CSV” (x, y, ground_z) after each run.
- Optionally clears the Nav2 costmaps if rapid altitude changes suggest costmap “ghosts”.
- Publishes UI-friendly telemetry: status text, distance/time-to-goal.

@par Publications
- `/movement/status` (`std_msgs/String`)         : Human-readable status.
- `/cmd_vel_real`    (`geometry_msgs/Twist`)     : Mixed command (Nav2 XY + injected Z).
- `/goal/distance`   (`std_msgs/Float32`)        : Remaining distance in metres.
- `/goal/time`       (`std_msgs/Float32`)        : Simple ETA proxy (seconds).

@par Subscriptions
- `/cmd/control`     (`std_msgs/String`)         : GUI verbs ("TAKEOFF", "MOVE TO GOAL", etc.).
- `/cmd/goal`        (`geometry_msgs/PointStamped`) : Staged goal in map frame (Z ignored).
- `/cmd/height`      (`std_msgs/Float32`)        : Target height-above-ground (HAG).
- `/drone/pose_1hz`  (`geometry_msgs/PoseStamped`): Down-sampled pose (UI relay).
- `/odometry`        (`nav_msgs/Odometry`)       : Full-rate odometry (redundant, but supported).
- `/cmd_vel`         (`geometry_msgs/Twist`)     : Nav2 controller output (XY + yaw rate).
- `/altitude/hag`    (`std_msgs/Float32`)        : Instant HAG (vertical below).
- `/altitude/hag_forward` (`std_msgs/Float32`)   : Forward look-ahead HAG.

@par Actions
- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) : Send XY goals to Nav2.

@par Services (optional / best effort)
- `/local_costmap/clear_entirely_local_costmap`  : Clear local costmap (type depends on distro).
- `/global_costmap/clear_entirely_global_costmap`: Clear global costmap (optional).
- `/lifecycle_manager_navigation/is_active`      : `std_srvs/Trigger` (Nav2 lifecycle manager).
- `/lifecycle_manager_localization/is_active`    : `std_srvs/Trigger` (SLAM lifecycle manager).

@par Parameters
- `control_rate_hz` (double, default: 35.0)   : Main control loop rate.
- `max_z_up`        (double, default: 3.0)    : Upward max climb speed (m/s).
- `max_z_down`      (double, default: 1.0)    : Downward max descent speed (m/s).
- `kp_z`            (double, default: 0.65)   : Z controller proportional gain.
- `ki_z`            (double, default: 0.20)   : Z controller integral gain.
- `pos_tol_xy`      (double, default: 0.25)   : XY tolerance for “close enough” checks (m).
- `pos_tol_z`       (double, default: 0.15)   : Z tolerance for “close enough” checks (m).
- `z_clear_thresh_m`(double, default: 0.4)    : Δz threshold to trigger local costmap clear (m).
- `z_clear_window_sec` (double, default: 0.75): Window to measure Δz (s).
- `z_clear_cooldown_sec`(double,default: 2.0) : Minimum time between clears (s).
- `z_clear_global_enable` (bool, default: false): Also clear global costmap if Δz is large.
- `z_clear_global_mult` (double, default: 2.0): Multiplier for global clear Δz threshold.

@note
- The node assumes Nav2 handles XY path planning from 360° LiDAR; this node only manages Z.
- If lifecycle managers are absent, the node assumes “active enough” for pre-flight progression.
"""

from __future__ import annotations

import os
import csv
import math
import time
from pathlib import Path
from collections import deque
from typing import Optional, Deque, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo

# Optional service type for clearing costmaps (variant across distros)
try:
    from nav2_msgs.srv import ClearEntireCostmap
    CLEAR_SRV = ClearEntireCostmap
except Exception:  # fallback
    from std_srvs import srv as _std_srvs
    CLEAR_SRV = _std_srvs.Empty  # type: ignore

# ---------------------------------------------------------------------------
# Constants / defaults
# ---------------------------------------------------------------------------
DEFAULT_HEIGHT_M: float = 0.75           # Used when no target height is set
DEFAULT_TAKEOFF_HEIGHT: float = 1.0      # Safety takeoff target if none provided

DEFAULT_OUTPUT_CMD_TOPIC = '/cmd_vel_real'
NAV2_CMD_TOPIC = '/cmd_vel'
STATUS_TOPIC = '/movement/status'

GUI_TO_CMD = {
    "HOVER": "hover",
    "MOVE TO GOAL": "move_to_goal",
    "LAND": "land",
    "TAKEOFF": "takeoff",
    "EMERGENCY LAND": "emergency_land",
    "START LOG": "start_log",
    "STOP LOG": "stop_log",
}


def _desired_height(target_height: Optional[float]) -> float:
    """
    @brief Resolve the desired HAG, with fallback to default.
    @param target_height Target HAG if provided.
    @return Desired HAG in metres.
    """
    return float(target_height) if target_height is not None else DEFAULT_HEIGHT_M


def clamp(v: float, lo: float, hi: float) -> float:
    """
    @brief Clamp a value into [lo, hi].
    @param v  Value to clamp.
    @param lo Lower bound.
    @param hi Upper bound.
    @return Clamped value.
    """
    return max(lo, min(hi, v))


class FlightControl(Node):
    """
    @class FlightControl
    @brief Nav2 + custom Z controller with a simple state machine and logging.

    @details
    - Listens to GUI commands and acts as a high-level flight mode manager.
    - Sends goals to Nav2, injects `linear.z` into Nav2 velocity for altitude control.
    - Records CSV, emits goal metrics for the GUI, and writes a compact trace CSV on exit.
    """

    # -----------------------------------------------------------------------
    # Init
    # -----------------------------------------------------------------------
    def __init__(self) -> None:
        """@brief Construct all publishers, subscribers, actions, services and timers."""
        super().__init__('flight_control')

        # --- Publishers
        self.pub_status    = self.create_publisher(String, STATUS_TOPIC, 10)
        self.pub_cmd_real  = self.create_publisher(Twist, DEFAULT_OUTPUT_CMD_TOPIC, 10)
        self.pub_goal_dist = self.create_publisher(Float32, '/goal/distance', 10)
        self.pub_goal_time = self.create_publisher(Float32, '/goal/time', 10)

        # --- Subscribers (GUI + pose)
        self.sub_cmd    = self.create_subscription(String,       '/cmd/control',    self.on_cmd,    10)
        self.sub_goal   = self.create_subscription(PointStamped, '/cmd/goal',       self.on_goal,   10)
        self.sub_height = self.create_subscription(Float32,      '/cmd/height',     self.on_height, 10)
        self.sub_pose_s = self.create_subscription(PoseStamped,  '/drone/pose_1hz', self.on_pose,   10)
        self.sub_odom   = self.create_subscription(Odometry,     '/odometry',       self.on_odom,   10)

        # --- Nav2 incoming velocity (BEST_EFFORT typical)
        qos_cmd = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.sub_nav2_cmd = self.create_subscription(Twist, NAV2_CMD_TOPIC, self._on_nav2_cmd, qos_cmd)
        self.last_nav2_twist: Twist = Twist()

        # --- Nav2 NavigateToPose action + cancel service
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.cancel_nav2_cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel')

        # --- Lifecycle "is_active" service clients (non-blocking)
        self.cli_nav_active = self.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        self.cli_loc_active = self.create_client(Trigger, '/lifecycle_manager_localization/is_active')

        # Readiness flags (updated asynchronously)
        self._nav_ready: bool = False
        self._loc_ready: bool = True   # if manager is absent, don't block

        # Poll readiness once per second (non-blocking)
        self._readiness_timer = self.create_timer(1.0, self._poll_readiness)

        # --- Nav2 status stream (GoalStatusArray) to detect SUCCEEDED
        self.sub_nav_status = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._on_nav_status, 10
        )
        self._latest_nav_status: Optional[int] = None

        # --- Terrain HAG topics
        self.sub_hag      = self.create_subscription(Float32, '/altitude/hag',         self.on_hag,         10)
        self.sub_hag_fwd  = self.create_subscription(Float32, '/altitude/hag_forward', self.on_hag_forward, 10)

        # --- HAG values
        self.hag: Optional[float] = None
        self.hag_forward: Optional[float] = None

        # --- Parameters -----------------------------------------------------
        # These control timing, vertical control behaviour, positional tolerances,
        # and the Δz costmap-clear watchdog thresholds.

        # Control loop rate (Hz). Higher = faster altitude updates but more CPU load.
        self.declare_parameter('control_rate_hz', 35.0)

        # Maximum upward climb speed (m/s) that the Z-controller will command.
        self.declare_parameter('max_z_up', 3.0)

        # Maximum downward descent speed (m/s) that the Z-controller will command.
        self.declare_parameter('max_z_down', 1.0)

        # Proportional gain for altitude (Z) control. Larger = faster response, more oscillation.
        self.declare_parameter('kp_z', 0.65)

        # Integral gain for altitude (Z) control. Corrects steady-state error; too high causes drift.
        self.declare_parameter('ki_z', 0.20)

        # XY position tolerance (metres) used to judge when a goal has been reached laterally.
        self.declare_parameter('pos_tol_xy', 0.25)

        # Z-axis (vertical) tolerance (metres) for altitude “close-enough” checks.
        self.declare_parameter('pos_tol_z', 0.15)

        # Δz threshold (metres) that triggers a *local* costmap clear when exceeded within
        # the configured window. Helps remove stale obstacles after large height changes.
        self.declare_parameter('z_clear_thresh_m', 0.4)

        # Time window (seconds) over which Δz is evaluated by the watchdog.
        self.declare_parameter('z_clear_window_sec', 0.75)

        # Minimum cooldown time (seconds) between consecutive costmap clears.
        self.declare_parameter('z_clear_cooldown_sec', 2.0)

        # Whether to also clear the *global* costmap if a very large Δz is detected.
        self.declare_parameter('z_clear_global_enable', False)

        # Multiplier applied to the local Δz threshold when deciding to clear the global costmap.
        # e.g. with threshold=0.4 and mult=2.0 → global clear when |Δz| > 0.8 m.
        self.declare_parameter('z_clear_global_mult', 2.0)


        # Read params
        self.ctrl_hz               = float(self.get_parameter('control_rate_hz').value)
        self.max_z_up              = float(self.get_parameter('max_z_up').value)
        self.max_z_down            = float(self.get_parameter('max_z_down').value)
        self.kp_z                  = float(self.get_parameter('kp_z').value)
        self.ki_z                  = float(self.get_parameter('ki_z').value)
        self.pos_tol_xy            = float(self.get_parameter('pos_tol_xy').value)
        self.pos_tol_z             = float(self.get_parameter('pos_tol_z').value)

        self.z_clear_thresh_m      = float(self.get_parameter('z_clear_thresh_m').value)
        self.z_clear_window_sec    = float(self.get_parameter('z_clear_window_sec').value)
        self.z_clear_cooldown_sec  = float(self.get_parameter('z_clear_cooldown_sec').value)
        self.z_clear_global_enable = bool(self.get_parameter('z_clear_global_enable').value)
        self.z_clear_global_mult   = float(self.get_parameter('z_clear_global_mult').value)

        # --- State / memory
        self.status: str = "Pre Flight Checks"
        self._last_status: Optional[str] = None
        self.last_cmd: Optional[str] = None

        self.target_height: Optional[float] = None
        self.goal_xyz: Optional[Tuple[float, float, float]] = None
        self.staged_goal_xy: Optional[Tuple[float, float]] = None

        self.currentX: Optional[float] = None
        self.currentY: Optional[float] = None
        self.currentZ: Optional[float] = None

        self.hover_z: Optional[float] = None

        # --- Δz watchdog
        self._z_hist: Deque[Tuple[float, float]] = deque()   # (t_sec, z)
        self._last_clear_t: float = 0.0

        # --- Costmap clear service clients
        self._clear_local_cli  = self.create_client(CLEAR_SRV, '/local_costmap/clear_entirely_local_costmap')
        self._clear_global_cli = self.create_client(CLEAR_SRV, '/global_costmap/clear_entirely_global_costmap')

        # --- Nav2 goal tracking
        self._is_navigating: bool = False
        self._nav2_goal_handle = None
        self._nav2_goal_active: bool = False

        # --- Z controller integral memory
        self._int_z: float = 0.0
        self._int_z_max: float = 1.0
        self._last_ctrl_time = self.get_clock().now()

        # --- Lightweight trace buffer (x,y,ground_z) during Moving to Goal
        self.record_rows: List[Tuple[float, float, float]] = []
        self._record_rate_hz: float = 10.0
        self._last_record_t: float = 0.0
        self.record_active: bool = False

        # --- CSV runtime logger (extended log)
        self._csv_fp = None
        self._csv_writer = None
        self._log_dir = Path.home() / '.ros' / 'trailblazer_logs'
        self._log_dir.mkdir(parents=True, exist_ok=True)

        # --- Timers
        self.timer = self.create_timer(1.0 / max(1.0, self.ctrl_hz), self.main_loop)
        self._status_heartbeat = self.create_timer(3.0, self._republish_status)

        self.set_status(self.status)
        self.get_logger().info("FlightControl ready (Nav2 XY + custom Z → /cmd_vel_real).")

    # -----------------------------------------------------------------------
    # Utilities / common
    # -----------------------------------------------------------------------
    def _republish_status(self) -> None:
        """@brief Periodically re-emit status for late-joining GUIs."""
        self.pub_status.publish(String(data=self.status))

    def set_status(self, s: str) -> None:
        """
        @brief Update status and emit to GUI. Triggers trace CSV write when leaving Moving to Goal.
        @param s New status text.
        """
        prev = getattr(self, "_last_status", None)
        if s != prev:
            if prev == 'Moving to Goal' and s != 'Moving to Goal':
                reached = (s == 'Arrived at Goal')
                self._save_trace_csv(reached=reached)
            self._last_status = s
            self.status = s
            self.pub_status.publish(String(data=s))
            self.get_logger().info(f"[status] {s}")

    def zero_twist(self) -> None:
        """@brief Publish a zeroed Twist on `/cmd_vel_real`."""
        self.pub_cmd_real.publish(Twist())

    def _desired_abs_z(self) -> float:
        """
        @brief Compute desired absolute altitude (z) from HAG and forward preview.

        @details
        - Prefer `hag_forward` (look-ahead) if finite: ground_ahead = currentZ - hag_forward.
        - Else use `hag` (underneath): ground_now = currentZ - hag.
        - If no HAG available: fall back to absolute HAG target (legacy).

        @return Desired absolute z (metres).
        """
        tgt_hag = _desired_height(self.target_height)

        if self.currentZ is None:
            return tgt_hag  # no better estimate yet

        if self.hag_forward is not None and self.hag_forward == self.hag_forward:  # NaN-safe
            ground_z_ahead = self.currentZ - float(self.hag_forward)
            return ground_z_ahead + tgt_hag

        if self.hag is not None and self.hag == self.hag:
            ground_z_now = self.currentZ - float(self.hag)
            return ground_z_now + tgt_hag

        return tgt_hag  # legacy absolute

    def _vz_hold(self, target_z: float) -> float:
        """
        @brief PI-like altitude hold controller (produces safe vz).

        @param target_z Desired absolute z (metres).
        @return Vertical velocity command (m/s), clamped to up/down limits.
        """
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

    def _on_nav2_cmd(self, msg: Twist) -> None:
        """
        @brief Cache latest Nav2 controller twist (XY + yaw rate).
        @param msg Incoming `/cmd_vel` message from Nav2.
        """
        self.last_nav2_twist = msg

    def _publish_with_injected_vz(self, vz: float) -> None:
        """
        @brief Publish Twist with Nav2 XY/yaw and injected Z.
        @param vz Vertical velocity command (m/s).
        """
        out = Twist()
        out.linear.x  = self.last_nav2_twist.linear.x
        out.linear.y  = self.last_nav2_twist.linear.y
        out.angular.z = self.last_nav2_twist.angular.z
        out.linear.z  = float(vz)
        self.pub_cmd_real.publish(out)

    def _publish_manual(self, vx: float, vy: float, vz: float, wz: float) -> None:
        """
        @brief Publish a fully manual Twist (bypasses Nav2 XY). Kept for completeness.
        """
        out = Twist()
        out.linear.x = float(vx)
        out.linear.y = float(vy)
        out.linear.z = float(vz)
        out.angular.z = float(wz)
        self.pub_cmd_real.publish(out)

    def _cancel_nav2_goal(self) -> None:
        """
        @brief Robust Nav2 cancel:
        - If we have a goal handle, cancel that specific goal.
        - Otherwise, use CancelGoal service to cancel any active goal.
        """
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

        if self.cancel_nav2_cli.service_is_ready():
            self.get_logger().info("Cancelling Nav2 goal via CancelGoal service (no handle)…")
            req = CancelGoal.Request()
            req.goal_info = GoalInfo()  # empty → cancel any goal
            fut = self.cancel_nav2_cli.call_async(req)

            def _canceled_service(_):
                self.get_logger().info("Nav2 CancelGoal service responded.")
                self._nav2_goal_active = False
                self._nav2_goal_handle = None
                self.last_nav2_twist = Twist()

            fut.add_done_callback(_canceled_service)
        else:
            self.get_logger().warn("CancelGoal service not ready; deferring cancel.")

    # -----------------------------------------------------------------------
    # Lifecycle + Nav2 status + Δz watchdog
    # -----------------------------------------------------------------------
    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        """
        @brief Track the latest NavigateToPose status (0..6). 4 == SUCCEEDED.
        @param msg Action status array.
        """
        if not msg.status_list:
            return
        self._latest_nav_status = int(msg.status_list[-1].status)

    def _poll_readiness(self) -> None:
        """
        @brief Poll lifecycle managers (if present) to guard pre-flight progression.
        """
        if self.cli_nav_active.service_is_ready():
            fut = self.cli_nav_active.call_async(Trigger.Request())
            fut.add_done_callback(self._on_nav_ready)

        if self.cli_loc_active.service_is_ready():
            fut2 = self.cli_loc_active.call_async(Trigger.Request())
            fut2.add_done_callback(self._on_loc_ready)

    def _on_nav_ready(self, fut) -> None:
        """@brief Async callback: mark Nav2 active/inactive."""
        try:
            res = fut.result()
            self._nav_ready = bool(res.success)
        except Exception:
            self._nav_ready = False

    def _on_loc_ready(self, fut) -> None:
        """@brief Async callback: mark localization active/inactive (assume active if error)."""
        try:
            res = fut.result()
            self._loc_ready = bool(res.success)
        except Exception:
            self._loc_ready = True  # do not block if missing/unstable

    def _dz_watchdog_and_clear(self, now_s: float) -> None:
        """
        @brief Track altitude changes and clear costmaps if |Δz| is large.

        @details
        Maintains a short history of (t,z). If |z(t)-z(t0)| exceeds a threshold over
        the configured window and cooldown has elapsed, we clear the local costmap,
        and (optionally) the global costmap if Δz is very large.
        """
        z = self.currentZ
        if z is None:
            return

        # Maintain window
        self._z_hist.append((now_s, float(z)))
        while self._z_hist and (now_s - self._z_hist[0][0]) > self.z_clear_window_sec:
            self._z_hist.popleft()

        if len(self._z_hist) < 2:
            return

        z_old = self._z_hist[0][1]
        dz = abs(float(z) - z_old)
        since_last = now_s - self._last_clear_t

        want_local = (dz > self.z_clear_thresh_m) and (since_last > self.z_clear_cooldown_sec)
        want_global = (
            self.z_clear_global_enable and
            (dz > (self.z_clear_thresh_m * self.z_clear_global_mult)) and
            (since_last > self.z_clear_cooldown_sec)
        )

        if not (want_local or want_global):
            return

        # Fire clears (non-blocking)
        if want_local and self._clear_local_cli.service_is_ready():
            try:
                self.get_logger().info(f"Δz watchdog: Δz={dz:.2f}m → clearing LOCAL costmap")
                self._clear_local_cli.call_async(CLEAR_SRV.Request())
                self._last_clear_t = now_s
            except Exception as e:
                self.get_logger().warn(f"Δz watchdog: local clear failed: {e}")

        if want_global and self._clear_global_cli.service_is_ready():
            try:
                self.get_logger().info(f"Δz watchdog: large Δz={dz:.2f}m → clearing GLOBAL costmap")
                self._clear_global_cli.call_async(CLEAR_SRV.Request())
                self._last_clear_t = now_s
            except Exception as e:
                self.get_logger().warn(f"Δz watchdog: global clear failed: {e}")

    # -----------------------------------------------------------------------
    # State machine
    # -----------------------------------------------------------------------
    def main_loop(self) -> None:
        """@brief Main periodic tick; dispatch by current status + record trace rows."""
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

        # Lightweight recorder for the extended CSV log
        self._maybe_record_row()

    def pre_flight_checks(self) -> None:
        """@brief Wait until Nav2 (and localization, if managed) are active."""
        if self._nav_ready and self._loc_ready:
            self.set_status("Landed")
        else:
            self.zero_twist()

    def landed(self) -> None:
        """@brief Disarmed/idle on ground; respond to TAKEOFF/MOVE TO GOAL/emergency."""
        self.zero_twist()
        if self.last_cmd in ('takeoff', 'move_to_goal'):
            self.set_status('Taking off')
        elif self.last_cmd == 'emergency_land':
            self.set_status('Emergency Landing')

    def taking_off(self) -> None:
        """@brief Climb to a safe HAG and transition to goal/hover."""
        gz = self._desired_abs_z()
        vz = self._vz_hold(gz)

        # Transition when within Z tolerance
        if self.currentZ is not None and abs(gz - self.currentZ) <= self.pos_tol_z:
            if self.last_cmd == 'move_to_goal' and self.staged_goal_xy is not None:
                if self.nav2_client.wait_for_server(timeout_sec=0.5):
                    self._send_nav2_goal(*self.staged_goal_xy)
                    self.set_status('Moving to Goal')
                else:
                    self.get_logger().warn("NavigateToPose server not ready yet")
            else:
                self.set_status('Hovering')

        self._publish_with_injected_vz(vz)

    def hovering(self) -> None:
        """@brief Hold altitude; accept MOVE TO GOAL / LAND / EMERGENCY LAND verbs."""
        gz = self._desired_abs_z()
        self._publish_with_injected_vz(self._vz_hold(gz))

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

    def moving_to_goal(self) -> None:
        """@brief Inject Z while Nav2 drives XY; detect arrival; allow overrides."""
        self.record_active = True

        # Z target + injected vz
        gz = self._desired_abs_z()
        vz = self._vz_hold(gz)

        # Trace buffer row at fixed rate
        now = self.get_clock().now().nanoseconds / 1e9
        if self.currentX is not None and self.currentY is not None and self.currentZ is not None:
            if (now - self._last_record_t) >= (1.0 / self._record_rate_hz):
                if self.hag_forward is not None and self.hag_forward == self.hag_forward:
                    ground_z = self.currentZ - float(self.hag_forward)
                elif self.hag is not None and self.hag == self.hag:
                    ground_z = self.currentZ - float(self.hag)
                else:
                    ground_z = float('nan')
                self.record_rows.append((float(self.currentX), float(self.currentY), float(ground_z)))
                self._last_record_t = now

        # Arrival: wait for SUCCEEDED from Nav2 stream
        if self._latest_nav_status == 4:
            self._is_navigating = False
            self._cancel_nav2_goal()
            self.set_status('Arrived at Goal')
            self._publish_with_injected_vz(self._vz_hold(gz))
            return

        # Keep flying
        self._publish_with_injected_vz(vz)

        # Δz watchdog (only meaningful when moving)
        self._dz_watchdog_and_clear(now)

        # Overrides en route
        if self.last_cmd == 'hover':
            self._is_navigating = False
            self._cancel_nav2_goal()
            self.set_status('Hovering')
        elif self.last_cmd == 'land':
            self._is_navigating = False
            self._cancel_nav2_goal()
            self.set_status('Landing')
        elif self.last_cmd == 'emergency_land':
            self._is_navigating = False
            self._cancel_nav2_goal()
            self.set_status('Emergency Landing')

    def arrived_at_goal(self) -> None:
        """@brief Hold at goal; accept LAND/HOVER."""
        gz = self._desired_abs_z()
        self._publish_with_injected_vz(self._vz_hold(gz))

        if self.last_cmd == 'land':
            self.set_status('Landing')
        elif self.last_cmd == 'hover':
            self.set_status('Hovering')

    def landing(self) -> None:
        """@brief Controlled descent to ~0.10 m, then Landed."""
        land_z = 0.10
        if self.currentZ is None:
            self.zero_twist()
            return
        if self.currentZ <= (land_z + self.pos_tol_z):
            self.zero_twist()
            self.set_status('Landed')
            return
        self._publish_with_injected_vz(self._vz_hold(land_z))

    def emergency_landing(self) -> None:
        """@brief Aggressive descent (limited by max_z_down) until near ground."""
        if self.currentZ is None:
            self.zero_twist()
            return
        self._publish_with_injected_vz(self._vz_hold(-5.0))  # controller clamps to max_z_down
        if self.currentZ <= 0.15:
            self.zero_twist()
            self.set_status('Landed')

    # -----------------------------------------------------------------------
    # Callbacks (GUI + telemetry)
    # -----------------------------------------------------------------------
    def on_cmd(self, msg: String) -> None:
        """
        @brief Handle GUI command verbs; map to internal actions and state transitions.
        @param msg GUI command string.
        """
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

    def _send_nav2_goal(self, gx: float, gy: float) -> None:
        """
        @brief Send an XY goal to Nav2 in the `map` frame.
        @param gx Goal X (m)
        @param gy Goal Y (m)
        """
        self._is_navigating = True
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

    def on_goal(self, msg: PointStamped) -> None:
        """
        @brief Stage an incoming goal (from GUI) and publish goal metrics.
        @param msg Incoming PointStamped (map frame assumed).
        """
        gx = float(msg.point.x)
        gy = float(msg.point.y)
        gz = self.target_height if self.target_height is not None else DEFAULT_TAKEOFF_HEIGHT

        self.staged_goal_xy = (gx, gy)
        self.goal_xyz = (gx, gy, float(gz))
        self._publish_goal_metrics()
        self.get_logger().info(f"/cmd/goal staged: (x={gx:.2f}, y={gy:.2f}, z={gz:.2f})")

    def on_height(self, msg: Float32) -> None:
        """
        @brief Update target HAG setpoint.
        @param msg Desired HAG in metres.
        """
        self.target_height = float(msg.data)
        self.get_logger().info(f"/cmd/height: {self.target_height:.2f} m")
        if self.status == 'Hovering':
            self.hover_z = self.target_height  # live adjust

    def on_pose(self, msg: PoseStamped) -> None:
        """
        @brief Update current pose from PoseStamped (UI relay).
        @param msg PoseStamped
        """
        self.currentX = msg.pose.position.x
        self.currentY = msg.pose.position.y
        self.currentZ = msg.pose.position.z
        self._publish_goal_metrics()

    def on_odom(self, msg: Odometry) -> None:
        """
        @brief Update current pose from Odometry (redundant but supported).
        @param msg Odometry
        """
        self.currentX = msg.pose.pose.position.x
        self.currentY = msg.pose.pose.position.y
        self.currentZ = msg.pose.pose.position.z
        self._publish_goal_metrics()

    def on_hag(self, msg: Float32) -> None:
        """@brief Update current HAG (below)."""
        self.hag = float(msg.data)

    def on_hag_forward(self, msg: Float32) -> None:
        """@brief Update forward look-ahead HAG."""
        self.hag_forward = float(msg.data)

    def _publish_goal_metrics(self) -> None:
        """
        @brief Publish remaining distance and a simple ETA proxy for the GUI.
        @note ETA is a placeholder equal to distance (s) until speed-based estimate is added.
        """
        if self.currentX is None or self.goal_xyz is None:
            return
        gx, gy, _ = self.goal_xyz
        dx = gx - self.currentX
        dy = gy - self.currentY
        dist_xy = math.hypot(dx, dy)
        self.pub_goal_dist.publish(Float32(data=float(dist_xy)))
        self.pub_goal_time.publish(Float32(data=float(max(0.0, dist_xy))))

    # -----------------------------------------------------------------------
    # CSV logging (runtime log) + trace CSV (path heatmap input)
    # -----------------------------------------------------------------------
    def _start_logging(self) -> None:
        """@brief Begin writing an extended CSV runtime log to ~/.ros/trailblazer_logs."""
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

    def _stop_logging(self) -> None:
        """@brief Stop writing the extended CSV runtime log."""
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

    def _maybe_record_row(self) -> None:
        """@brief Periodically append a row to the extended CSV log."""
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
        try:
            self._csv_fp.flush()
        except Exception:
            pass

    def _save_trace_csv(self, reached: bool) -> None:
        """
        @brief Write the compact trace CSV (x,y,ground_z) for map/graph generation.
        @param reached Whether the goal was reported as reached (for future metadata).
        """
        try:
            if not self.record_rows:
                self.get_logger().info("[trace] no samples recorded; skipping CSV write")
                return

            outdir = os.path.join(os.getcwd(), 'data')
            os.makedirs(outdir, exist_ok=True)

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
                    w.writerow([f"{x:.3f}", f"{y:.3f}", f"{ground_z:.3f}"])

            self.get_logger().info(f"[trace] wrote {len(self.record_rows)} samples to {path}")
        except Exception as e:
            self.get_logger().error(f"[trace] failed to write CSV: {e}")
        finally:
            # Reset trace buffer after write or error
            self.record_rows = []
            self.record_active = False

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    """@brief ROS 2 entry point."""
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
