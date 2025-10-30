#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

import numpy as np
import tf_transformations as tft  # For quaternion to rotation matrix

GUI_TO_CMD = {
    "HOVER": "hover",
    "MOVE TO GOAL": "move_to_goal",
    "LAND": "land",
    "TAKEOFF": "takeoff",
    "EMERGENCY LAND": "emergency_land",
}

STATUSES = [
    "Pre Flight Checks",
    "Landed",
    "Hovering",
    "Arrived at Goal",

    "Taking off",
    "Landing",
    "Hovering",
    "Moving to Goal",
    "Emergency Landing",
]

class FlightControl(Node):
    def __init__(self):
        super().__init__('flight_control')
        self.get_logger().info("Flight Control Node Started")

        # --- Publishers ---
        self.pub_status  = self.create_publisher(String, '/movement/status', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_goal_dist = self.create_publisher(Float32, '/goal/distance', 10)
        self.pub_goal_time = self.create_publisher(Float32, '/goal/time', 10)


        # --- Parameters (defaults; overridable from launch) ---
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('max_xy_speed',   1.5)
        self.declare_parameter('max_z_up',       2.0)
        self.declare_parameter('max_z_down',     1.0)
        self.declare_parameter('kp_xy',          0.8)
        self.declare_parameter('kp_z',           0.65)
        self.declare_parameter('ki_z',           0.20)
        self.declare_parameter('pos_tol_xy',     0.25)
        self.declare_parameter('pos_tol_z',      0.15)
        self.declare_parameter('rep_threshold', 3.0)  # Max distance for repulsion (m); matches your min_obs_dist
        self.declare_parameter('k_rep', 2.0)          # Repulsive gain; higher = stronger avoidance

        # Read params
        self.ctrl_hz      = float(self.get_parameter('control_rate_hz').value)
        self.max_xy_speed = float(self.get_parameter('max_xy_speed').value)
        self.max_z_up     = float(self.get_parameter('max_z_up').value)
        self.max_z_down   = float(self.get_parameter('max_z_down').value)
        self.kp_xy        = float(self.get_parameter('kp_xy').value)
        self.kp_z         = float(self.get_parameter('kp_z').value)
        self.ki_z         = float(self.get_parameter('ki_z').value)
        self.pos_tol_xy   = float(self.get_parameter('pos_tol_xy').value)
        self.pos_tol_z    = float(self.get_parameter('pos_tol_z').value)
        self.rep_threshold = float(self.get_parameter('rep_threshold').value)
        self.k_rep = float(self.get_parameter('k_rep').value)

        # --- Subscribers ---
        self.sub_cmd      = self.create_subscription(String,       '/cmd/control',   self.on_cmd,   10)
        self.sub_goal     = self.create_subscription(PointStamped, '/cmd/goal',      self.on_goal,  10)
        self.sub_height_s = self.create_subscription(Float32,      '/cmd/height',    self.on_height,10)
        self.sub_pose_s   = self.create_subscription(PoseStamped,  '/drone/pose_1hz',self.on_pose,  10)

        self.velX = self.velY = self.velZ = None  # measured (or filtered) velocities
        self.k_brake_xy      = 1.25               # how hard to counter-brake (cmd vel per m/s)
        self.stop_speed_xy   = 0.05               # when |v| < this, command 0 precisely
        self.max_brake_speed = 1.5                # clamp for safety

        # Subscribe to filtered odometry (preferred). If you don’t have this, use /odometry.
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)

        self.sub_sectors = self.create_subscription(Float32MultiArray, '/sector_mins', self.on_sectors, 10)
        self.sector_mins = None  # List of min distances per sector


        # --- Internal state ---
        self.status          = "Pre Flight Checks"
        self._last_status    = None
        self.last_cmd        = None

        self.target_height   = None
        self.goal_xyz        = None
        self.hover_z = None  # Z setpoint captured when HOVER is pressed

        self.current_pose    = None
        self.currentX        = None
        self.currentY        = None
        self.currentZ        = None

        # Z PI controller state
        self._int_z          = 0.0
        self._int_z_max      = 1.0  # limit on integral *contribution* (anti-windup)
        self._last_ctrl_time = self.get_clock().now()

        # Timer (main loop)
        self.timer = self.create_timer(1.0 / max(1.0, self.ctrl_hz), self.main_loop)

        # Publish initial status so GUI leaves "CONNECTING..."
        self.set_status(self.status)

    # ----------------- Utilities -----------------
    def on_sectors(self, msg: Float32MultiArray):
        self.sector_mins = msg.data
        
    def clamp(self, x, lo, hi):
        return lo if x < lo else hi if x > hi else x
        

    def set_status(self, s: str):
        if s != self._last_status:
            self._last_status = s
            self.status = s
            msg = String()
            msg.data = s
            self.pub_status.publish(msg)
            self.get_logger().info(f"Status: {s}")

    def zero_twist(self):
        self.pub_cmd_vel.publish(Twist())

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
            # switching intent; drop any hover target
            self.hover_z = None
            if self.currentZ is not None and self.currentZ > 0.3:
                self.set_status('Moving to Goal')
            else:
                self.set_status('Taking off')

        elif self.last_cmd == 'land':
            self.hover_z = None
            self.set_status('Landing')

        elif self.last_cmd == 'hover':
            # Override any previously set height:
            self.target_height = None
            # Snapshot the current altitude as the hover setpoint (or capture later on first pose)
            self.hover_z = float(self.currentZ) if self.currentZ is not None else None
            self.set_status('Hovering')

        elif self.last_cmd == 'emergency_land':
            self.hover_z = None
            self.set_status('Emergency Landing')


    def on_goal(self, msg: PointStamped):
        self.goal_xyz = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        self._update_goal_metrics()
        self.get_logger().info(f"/cmd/goal: {self.goal_xyz}")

    def on_height(self, msg: Float32):
        self.target_height = float(msg.data)
        self.get_logger().info(f"/cmd/height: {self.target_height:.2f} m")

    def on_pose(self, msg: PoseStamped):
        self.current_pose = msg.pose
        self.currentX = float(msg.pose.position.x)
        self.currentY = float(msg.pose.position.y)
        self.currentZ = float(msg.pose.position.z)

        # If HOVER was requested but we hadn't captured Z yet, snapshot once now.
        if self.last_cmd == 'hover' and self.hover_z is None and self.currentZ is not None:
            self.hover_z = float(self.currentZ)


    # ----------------- Core controllers -----------------
    def _vz_hold(self, target_z: float) -> float:
        """
        PI controller for altitude hold. Returns a vz command.
        """
        if self.currentZ is None:
            return 0.0

        now = self.get_clock().now()
        dt = (now - self._last_ctrl_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 1.0:  # guard dt if clocks jump
            dt = 1.0 / self.ctrl_hz
        self._last_ctrl_time = now

        e = float(target_z - self.currentZ)

        # Integral with anti-windup via clamped contribution
        self._int_z += e * dt
        if self.ki_z > 0.0:
            max_i = self._int_z_max / self.ki_z
            self._int_z = self.clamp(self._int_z, -max_i, max_i)

        vz = self.kp_z * e + self.ki_z * self._int_z

        # Ascent/descent limits (gentler down)
        if vz >= 0.0:
            vz = self.clamp(vz, 0.0, self.max_z_up)
        else:
            vz = self.clamp(vz, -self.max_z_down, 0.0)
        return float(vz)

    def _cmd_vel(self, vx: float, vy: float, vz: float):
        tw = Twist()
        tw.linear.x = float(self.clamp(vx, -self.max_xy_speed, self.max_xy_speed))
        tw.linear.y = float(self.clamp(vy, -self.max_xy_speed, self.max_xy_speed))
        # vz already limited in _vz_hold
        tw.linear.z = float(vz)
        self.pub_cmd_vel.publish(tw)

    # ----------------- States -----------------
    def main_loop(self):
        # Ensure we always publish something sensible for Z each tick to prevent sag.
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
            self.set_status("Pre Flight Checks" )

    def pre_flight_checks(self):
        # Minimal pre-flight for now
        self.set_status("Pre Flight Checks")
        # Drop straight to Landed once we have pose
        if self.currentZ is not None:
            self.set_status("Landed")

    def landed(self):
        # Motors stopped; wait for commands
        self.zero_twist()
        if self.last_cmd == 'takeoff':
            self.set_status('Taking off')
        elif self.last_cmd == 'move_to_goal':
            self.set_status('Taking off')


    def arrived_at_goal(self):
        # Park at goal height (if set) or current height
        gz = self.goal_xyz[2] if self.goal_xyz is not None else (self.currentZ or 0.0)
        vz = self._vz_hold(gz)
        self._cmd_vel(0.0, 0.0, vz)

        if self.last_cmd == 'land':
            self.set_status('Landing')

    def taking_off(self):
        # Pick a sensible default if GUI hasn't sent a height
        tgt = self.target_height if self.target_height is not None else 2.0

        if self.currentZ is None:
            # no pose yet; keep publishing nothing aggressive
            self.zero_twist()
            return

        # Close enough? park and decide next state
        if abs(tgt - self.currentZ) <= self.pos_tol_z:
            if self.last_cmd == 'move_to_goal' and self.goal_xyz is not None:
                self.set_status('Moving to Goal')
            else:
                self.set_status('Hovering')
            # keep holding altitude no matter what
            vz = self._vz_hold(tgt)
            self._cmd_vel(0.0, 0.0, vz)
            return

        # Otherwise, rise/descend toward target and KEEP thrust applied
        vz = self._vz_hold(tgt)
        self._cmd_vel(0.0, 0.0, vz)

    def moving_to_goal(self):
        if self.currentX is None or self.goal_xyz is None:
            # Nothing to do yet; hold current/target height to avoid sag
            tgt = self.target_height if self.target_height is not None else (self.currentZ or 0.0)
            vz = self._vz_hold(tgt)
            self._cmd_vel(0.0, 0.0, vz)
            return

        gx, gy, gz = self.goal_xyz
        ex = gx - self.currentX
        ey = gy - self.currentY
        ez = gz - (self.currentZ if self.currentZ is not None else gz)

        # Arrival check (unchanged)
        if abs(ex) <= self.pos_tol_xy and abs(ey) <= self.pos_tol_xy and abs(ez) <= self.pos_tol_z:
            self.set_status('Arrived at Goal')
            self.pub_goal_dist.publish(Float32(data=0.0))
            self.pub_goal_time.publish(Float32(data=0.0))
            vz = self._vz_hold(gz)
            self._cmd_vel(0.0, 0.0, vz)
            return

        # Z via PI hold to goal.z (unchanged)
        vz = self._vz_hold(gz)

        # XY with potential fields
        att = np.array([self.kp_xy * ex, self.kp_xy * ey])  # Attractive vector
        att_mag = np.linalg.norm(att)
        if att_mag > self.max_xy_speed:
            att = (att / att_mag) * self.max_xy_speed

        rep = np.zeros(2)  # Repulsive vector (in world frame)

        if self.sector_mins is not None and self.current_pose is not None and len(self.sector_mins) > 0:
            nsec = len(self.sector_mins)
            sector_width = 2 * math.pi / nsec
            sector_angles = [(i + 0.5) * sector_width for i in range(nsec)]  # Centers (rad), assuming angle_min=0

            # Get 2x2 rotation matrix from current orientation (to transform body-frame obstacle points to world)
            q = [self.current_pose.orientation.x, self.current_pose.orientation.y,
                self.current_pose.orientation.z, self.current_pose.orientation.w]
            rot = tft.quaternion_matrix(q)[0:2, 0:2]  # XY rotation only

            for i in range(nsec):
                d = self.sector_mins[i]
                if d >= self.rep_threshold or not np.isfinite(d):
                    continue  # Ignore far/invalid sectors

                a = sector_angles[i]  # Sector center angle (body frame)
                px_b = d * math.cos(a)
                py_b = d * math.sin(a)
                p_b = np.array([px_b, py_b])

                # Transform to world frame
                p_w = np.dot(rot, p_b) + np.array([self.currentX, self.currentY])

                # Repulsive direction: from obstacle to drone (push away)
                dir_vec = np.array([self.currentX, self.currentY]) - p_w
                dist = np.linalg.norm(dir_vec)
                if dist > 0.01:  # Avoid div/0
                    rep_mag = self.k_rep * ((1.0 / dist) - (1.0 / self.rep_threshold)) ** 2
                    rep_dir = dir_vec / dist
                    rep += rep_mag * rep_dir

        # Total desired velocity vector
        total = att + rep
        total_mag = np.linalg.norm(total)
        if total_mag > self.max_xy_speed:
            total = (total / total_mag) * self.max_xy_speed

        vx = total[0]
        vy = total[1]

        self._cmd_vel(vx, vy, vz)
        self._update_goal_metrics()

    def landing(self):
        # Descend toward a small near-ground height (e.g., 0.10 m)
        land_z = 0.10

        if self.currentZ is None:
            self.zero_twist()
            return

        if self.currentZ <= (land_z + self.pos_tol_z):
            # Touchdown
            self.zero_twist()
            self.set_status('Landed')
            return

        # Command a gentle descent (negative vz limited within _vz_hold)
        vz = self._vz_hold(land_z)
        self._cmd_vel(0.0, 0.0, vz)

    def hovering(self):
        self.set_status('Hovering')

        # Ensure we have a *constant* Z setpoint during hover
        if self.hover_z is None and self.currentZ is not None:
            self.hover_z = float(self.currentZ)

        # Hold the frozen hover_z; if still None, fall back to currentZ
        tgt_z = self.hover_z if self.hover_z is not None else (self.currentZ or 0.0)
        vz = self._vz_hold(tgt_z)

        # Zero XY while hovering (or swap in quick-stop XY if you want snappier braking)
        self._cmd_vel(0.0, 0.0, vz)

        # Allowed intent changes only
        if self.last_cmd == 'move_to_goal' and self.goal_xyz is not None:
            self.set_status('Moving to Goal')
        elif self.last_cmd == 'land':
            self.set_status('Landing')

    def emergency_landing(self):
        # Aggressive, controlled descent straight down (no XY)
        if self.currentZ is None:
            self.zero_twist()
            return
        # Force target well below ground so PI drives fast down; limit by max_z_down internally
        vz = self._vz_hold(-5.0)
        self._cmd_vel(0.0, 0.0, vz)
        if self.currentZ <= 0.15:
            self.zero_twist()
            self.set_status('Landed')

    def _update_goal_metrics(self):
        """Publish straight-line distance to goal and a naive ETA = distance * 2."""
        if self.goal_xyz is None:
            return
        if self.currentX is None or self.currentY is None or self.currentZ is None:
            return

        gx, gy, gz = self.goal_xyz
        ex = gx - self.currentX
        ey = gy - self.currentY
        ez = gz - self.currentZ

        distance = math.sqrt(ex*ex + ey*ey + ez*ez)

        # Publish distance (meters)
        self.pub_goal_dist.publish(Float32(data=float(distance)))

        # Publish time as simply distance * 2 (seconds)
        self.pub_goal_time.publish(Float32(data=float(distance * 2.0)))

    def on_odom(self, msg: Odometry):
        t = msg.twist.twist
        self.velX = float(t.linear.x)
        self.velY = float(t.linear.y)
        self.velZ = float(t.linear.z)

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