#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

# --- Minimal quaternion helpers (avoid external tf_transformations dependency) ---
def _quat_to_yaw(qx, qy, qz, qw) -> float:
    # yaw (Z-axis rotation) from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def _yaw_rot2x2(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s],
                     [s,  c]])
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

        # Publishers
        self.pub_status  = self.create_publisher(String, '/movement/status', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.pub_goal_dist = self.create_publisher(Float32, '/goal/distance', 10)
        self.pub_goal_time = self.create_publisher(Float32, '/goal/time', 10)

        # Parameters (default values)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('max_xy_speed',   1.5)
        self.declare_parameter('max_z_up',       2.0)
        self.declare_parameter('max_z_down',     1.0)
        self.declare_parameter('kp_xy',          0.8)
        self.declare_parameter('kp_z',           0.65)
        self.declare_parameter('ki_z',           0.20)
        self.declare_parameter('pos_tol_xy',     0.25)
        self.declare_parameter('pos_tol_z',      0.15)
        self.declare_parameter('rep_threshold', 3.0)  
        self.declare_parameter('k_rep', 2.0)          

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

        # Subscribers
        self.sub_cmd      = self.create_subscription(String,       '/cmd/control',   self.on_cmd,   10)
        self.sub_goal     = self.create_subscription(PointStamped, '/cmd/goal',      self.on_goal,  10)
        self.sub_height_s = self.create_subscription(Float32,      '/cmd/height',    self.on_height,10)
        self.sub_pose_s   = self.create_subscription(PoseStamped,  '/drone/pose_1hz',self.on_pose,  10)

        self.sub_odom = self.create_subscription(Odometry, '/odometry', self.on_odom, 10)
        self.sub_sectors = self.create_subscription(Float32MultiArray, '/sector_mins', self.on_sectors, 10)
        self.sector_mins = None 

        # Internal state
        self.status          = "Pre Flight Checks"
        self._last_status    = None
        self.last_cmd        = None

        self.target_height   = None
        self.goal_xyz        = None
        self.hover_z = None

        self.current_pose    = None
        self.currentX        = None
        self.currentY        = None
        self.currentZ        = None

        self._int_z          = 0.0
        self._int_z_max      = 1.0  
        self._last_ctrl_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / max(1.0, self.ctrl_hz), self.main_loop)
        self.set_status(self.status)

    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        return _quat_to_yaw(q.x, q.y, q.z, q.w)

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
            self.hover_z = None
            if self.currentZ is not None and self.currentZ > 0.3:
                self.set_status('Moving to Goal')
            else:
                self.set_status('Taking off')
        elif self.last_cmd == 'land':
            self.hover_z = None
            self.set_status('Landing')
        elif self.last_cmd == 'hover':
            self.target_height = None
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

        if self.last_cmd == 'hover' and self.hover_z is None and self.currentZ is not None:
            self.hover_z = float(self.currentZ)

    def _vz_hold(self, target_z: float) -> float:
        if self.currentZ is None:
            return 0.0
        now = self.get_clock().now()
        dt = (now - self._last_ctrl_time).nanoseconds / 1e9
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

    def _cmd_vel(self, vx: float, vy: float, vz: float):
        tw = Twist()
        tw.linear.x = float(self.clamp(vx, -self.max_xy_speed, self.max_xy_speed))
        tw.linear.y = float(self.clamp(vy, -self.max_xy_speed, self.max_xy_speed))
        tw.linear.z = float(vz)
        self.pub_cmd_vel.publish(tw)

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
            self.set_status("Pre Flight Checks")

    def pre_flight_checks(self):
        self.set_status("Pre Flight Checks")
        if self.currentZ is not None:
            self.set_status("Landed")

    def landed(self):
        self.zero_twist()
        if self.last_cmd == 'takeoff':
            self.set_status('Taking off')
        elif self.last_cmd == 'move_to_goal':
            self.set_status('Taking off')

    def arrived_at_goal(self):
        gz = self.goal_xyz[2] if self.goal_xyz is not None else (self.currentZ or 0.0)
        vz = self._vz_hold(gz)
        self._cmd_vel(0.0, 0.0, vz)
        if self.last_cmd == 'land':
            self.set_status('Landing')

    def taking_off(self):
        tgt = self.target_height if self.target_height is not None else 2.0
        if self.currentZ is None:
            self.zero_twist()
            return
        if abs(tgt - self.currentZ) <= self.pos_tol_z:
            if self.last_cmd == 'move_to_goal' and self.goal_xyz is not None:
                self.set_status('Moving to Goal')
            else:
                self.set_status('Hovering')
            vz = self._vz_hold(tgt)
            self._cmd_vel(0.0, 0.0, vz)
            return
        vz = self._vz_hold(tgt)
        self._cmd_vel(0.0, 0.0, vz)

    def moving_to_goal(self):
        if self.currentX is None or self.goal_xyz is None:
            tgt = self.target_height if self.target_height is not None else (self.currentZ or 0.0)
            vz = self._vz_hold(tgt)
            self._cmd_vel(0.0, 0.0, vz)
            return

        gx, gy, gz = self.goal_xyz
        ex = gx - self.currentX
        ey = gy - self.currentY
        ez = gz - (self.currentZ if self.currentZ is not None else gz)

        if abs(ex) <= self.pos_tol_xy and abs(ey) <= self.pos_tol_xy and abs(ez) <= self.pos_tol_z:
            self.set_status('Arrived at Goal')
            self.pub_goal_dist.publish(Float32(data=0.0))
            self.pub_goal_time.publish(Float32(data=0.0))
            vz = self._vz_hold(gz)
            self._cmd_vel(0.0, 0.0, vz)
            return

        vz = self._vz_hold(gz)

        att = np.array([self.kp_xy * ex, self.kp_xy * ey])
        att_mag = np.linalg.norm(att)
        if att_mag > self.max_xy_speed:
            att = (att / att_mag) * self.max_xy_speed

        rep = np.zeros(2)

        if self.sector_mins is not None and self.current_pose is not None and len(self.sector_mins) > 0:
            nsec = len(self.sector_mins)
            sector_width = 2 * math.pi / nsec
            sector_angles = [(i + 0.5) * sector_width for i in range(nsec)]

            q = [self.current_pose.orientation.x, self.current_pose.orientation.y,
                self.current_pose.orientation.z, self.current_pose.orientation.w]
            yaw = self.get_yaw_from_pose(self.current_pose)
            rot = _yaw_rot2x2(yaw)


            for i in range(nsec):
                d = self.sector_mins[i]
                if d >= self.rep_threshold or not np.isfinite(d):
                    continue
                a = sector_angles[i]
                px_b = d * math.cos(a)
                py_b = d * math.sin(a)
                p_b = np.array([px_b, py_b])
                p_w = np.dot(rot, p_b) + np.array([self.currentX, self.currentY])
                dir_vec = np.array([self.currentX, self.currentY]) - p_w
                dist = np.linalg.norm(dir_vec)
                if dist > 0.01:
                    rep_mag = self.k_rep * ((1.0 / dist) - (1.0 / self.rep_threshold)) ** 2
                    rep_dir = dir_vec / dist
                    rep += rep_mag * rep_dir

        total = att + rep
        total_mag = np.linalg.norm(total)
        if total_mag > self.max_xy_speed:
            total = (total / total_mag) * self.max_xy_speed

        current_yaw = self.get_yaw_from_pose(self.current_pose)
        vx_global = total[0]
        vy_global = total[1]
        desired_yaw = math.atan2(vy_global, vx_global)
        yaw_error = desired_yaw - current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        k_yaw = 1.5
        max_yaw_rate = math.radians(45)
        yaw_rate_cmd = k_yaw * yaw_error
        yaw_rate_cmd = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate_cmd))

        c = math.cos(-current_yaw)
        s = math.sin(-current_yaw)
        vx_body = c * vx_global - s * vy_global
        vy_body = s * vx_global + c * vy_global

        tw = Twist()
        tw.linear.x = float(self.clamp(vx_body, -self.max_xy_speed, self.max_xy_speed))
        tw.linear.y = 0.0  
        tw.linear.z = float(vz)
        tw.angular.z = float(yaw_rate_cmd)
        self.pub_cmd_vel.publish(tw)

        self._update_goal_metrics()

    def landing(self):
        land_z = 0.10

        if self.currentZ is None:
            self.zero_twist()
            return

        if self.currentZ <= (land_z + self.pos_tol_z):
            self.zero_twist()
            self.set_status('Landed')
            return

        vz = self._vz_hold(land_z)
        self._cmd_vel(0.0, 0.0, vz)

    def hovering(self):
        self.set_status('Hovering')

        if self.hover_z is None and self.currentZ is not None:
            self.hover_z = float(self.currentZ)

        tgt_z = self.hover_z if self.hover_z is not None else (self.currentZ or 0.0)
        vz = self._vz_hold(tgt_z)

        self._cmd_vel(0.0, 0.0, vz)

        if self.last_cmd == 'move_to_goal' and self.goal_xyz is not None:
            self.set_status('Moving to Goal')
        elif self.last_cmd == 'land':
            self.set_status('Landing')

    def emergency_landing(self):
        if self.currentZ is None:
            self.zero_twist()
            return
        vz = self._vz_hold(-5.0)
        self._cmd_vel(0.0, 0.0, vz)
        if self.currentZ <= 0.15:
            self.zero_twist()
            self.set_status('Landed')

    def _update_goal_metrics(self):
        if self.goal_xyz is None:
            return
        if self.currentX is None or self.currentY is None or self.currentZ is None:
            return

        gx, gy, gz = self.goal_xyz
        ex = gx - self.currentX
        ey = gy - self.currentY
        ez = gz - self.currentZ

        distance = math.sqrt(ex*ex + ey*ey + ez*ez)

        self.pub_goal_dist.publish(Float32(data=float(distance)))
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
