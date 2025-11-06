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

# --- Minimal quaternion helpers ---
def _quat_to_yaw(qx, qy, qz, qw) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def _yaw_rot2x2(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s],
                     [s,  c]])

def wrap_to_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi

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
    "Moving to Goal",
    "Emergency Landing",
]

DEFAULT_HEIGHT_M = 1.0

def _desired_height(target_height: float | None) -> float:
    return target_height if target_height is not None else DEFAULT_HEIGHT_M
    
    
class FlightControl(Node):
    def __init__(self):
        super().__init__('flight_control')
        self.get_logger().info("Flight Control Node Started")

        # Publishers
        self.pub_status  = self.create_publisher(String, '/movement/status', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_goal_dist = self.create_publisher(Float32, '/goal/distance', 10)
        self.pub_goal_time = self.create_publisher(Float32, '/goal/time', 10)
        self.pub_debug = self.create_publisher(String, '/debug/flight_control', 10)

        # Parameters
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('max_xy_speed', 1.5)
        self.declare_parameter('max_z_up', 2.0)
        self.declare_parameter('max_z_down', 1.0)
        self.declare_parameter('kp_xy', 0.8)
        self.declare_parameter('kp_z', 0.65)
        self.declare_parameter('ki_z', 0.20)
        self.declare_parameter('kp_attitude', 2.0)
        self.declare_parameter('pos_tol_xy', 0.25)
        self.declare_parameter('pos_tol_z', 0.15)
        self.declare_parameter('rep_threshold', 2.5)
        self.declare_parameter('k_rep', 3.0)
        self.declare_parameter('k_yaw', 1.5)
        self.declare_parameter('max_yaw_rate_deg', 45.0)
        self.declare_parameter('safe_stopping_dist', 1.5)
        self.declare_parameter('use_gap_navigation', True)
        self.declare_parameter('gap_heading_window_deg', 30.0)

        # Read params
        self.ctrl_hz = float(self.get_parameter('control_rate_hz').value)
        self.max_xy_speed = float(self.get_parameter('max_xy_speed').value)
        self.max_z_up = float(self.get_parameter('max_z_up').value)
        self.max_z_down = float(self.get_parameter('max_z_down').value)
        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kp_attitude = float(self.get_parameter('kp_attitude').value)
        self.pos_tol_xy = float(self.get_parameter('pos_tol_xy').value)
        self.pos_tol_z = float(self.get_parameter('pos_tol_z').value)
        self.rep_threshold = float(self.get_parameter('rep_threshold').value)
        self.k_rep = float(self.get_parameter('k_rep').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.max_yaw_rate = math.radians(float(self.get_parameter('max_yaw_rate_deg').value))
        self.safe_stopping_dist = float(self.get_parameter('safe_stopping_dist').value)
        self.use_gap_nav = bool(self.get_parameter('use_gap_navigation').value)
        self.gap_heading_window = math.radians(float(self.get_parameter('gap_heading_window_deg').value))

        # Subscribers
        self.sub_cmd = self.create_subscription(String, '/cmd/control', self.on_cmd, 10)
        self.sub_goal = self.create_subscription(PointStamped, '/cmd/goal', self.on_goal, 10)
        self.sub_height_s = self.create_subscription(Float32, '/cmd/height', self.on_height, 10)
        self.sub_pose_s = self.create_subscription(PoseStamped, '/drone/pose_1hz', self.on_pose, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odometry', self.on_odom, 10)
        self.sub_sectors = self.create_subscription(Float32MultiArray, '/sector_mins', self.on_sectors, 10)
        self.sub_gap_info = self.create_subscription(Float32MultiArray, '/gap_info', self.on_gap_info, 10)

        # Internal state
        self.status = "Pre Flight Checks"
        self._last_status = None
        self.last_cmd = None

        self.target_height = DEFAULT_HEIGHT_M
        self.goal_xyz = None
        self.hover_z = None

        self.current_pose = None
        self.currentX = None
        self.currentY = None
        self.currentZ = None

        self._int_z = 0.0
        self._int_z_max = 1.0
        self._last_ctrl_time = self.get_clock().now()

        self.sector_mins = None
        self.sector_angles = None
        self.num_sectors = 8
        self.gap_info = None

        self.velX = 0.0
        self.velY = 0.0
        self.velZ = 0.0

        self.stuck_counter = 0

        self.timer = self.create_timer(1.0 / max(1.0, self.ctrl_hz), self.main_loop)
        self.set_status(self.status)
        self.get_logger().info(f'Control Rate: {self.ctrl_hz} Hz, Gap Navigation: {self.use_gap_nav}')

    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        return _quat_to_yaw(q.x, q.y, q.z, q.w)

    def on_sectors(self, msg: Float32MultiArray):
        """Updated sector callback with angle calculation"""
        self.sector_mins = np.array(msg.data)
        self.num_sectors = len(self.sector_mins)
        
        # Calculate sector angles (center of each sector)
        sector_width = 2 * math.pi / self.num_sectors
        self.sector_angles = np.array([(i + 0.5) * sector_width - math.pi for i in range(self.num_sectors)])

    def on_gap_info(self, msg: Float32MultiArray):
        """Receive best gap information from enhanced perception"""
        if len(msg.data) >= 3:
            self.gap_info = {
                'angle': float(msg.data[0]),
                'width': float(msg.data[1]),
                'clearance': float(msg.data[2])
            }

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
        self.goal_xyz = (float(msg.point.x), float(msg.point.y), 0.0)
        self._update_goal_metrics()
        self.get_logger().info(f"/cmd/goal: (x={self.goal_xyz[0]:.2f}, y={self.goal_xyz[1]:.2f})  [z ignored]")

    def on_height(self, msg: Float32):
        self.target_height = float(msg.data)
        self.get_logger().info(f"Set height updated to {self.target_height:.2f} m")

    def on_pose(self, msg: PoseStamped):
        self.current_pose = msg.pose
        self.currentX = float(msg.pose.position.x)
        self.currentY = float(msg.pose.position.y)
        self.currentZ = float(msg.pose.position.z)

        if self.last_cmd == 'hover' and self.hover_z is None and self.currentZ is not None:
            self.hover_z = float(self.currentZ)

    def _vz_hold(self, target_z: float) -> float:
        """Z-height hold with PI control"""
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

    def _cmd_vel(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Publish cmd_vel with clamping"""
        tw = Twist()
        tw.linear.x = float(self.clamp(vx, -self.max_xy_speed, self.max_xy_speed))
        tw.linear.y = float(self.clamp(vy, -self.max_xy_speed, self.max_xy_speed))
        tw.linear.z = float(vz)
        tw.angular.z = float(self.clamp(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate))
        self.pub_cmd_vel.publish(tw)

    def _get_min_obstacle_distance(self) -> float:
        """Get minimum obstacle distance across all sectors"""
        if self.sector_mins is None or len(self.sector_mins) == 0:
            return float('inf')
        return float(np.nanmin(self.sector_mins))

    def _get_obstacle_repulsion(self) -> np.ndarray:
        """
        Compute repulsive force from obstacles in world frame
        FIXED: Properly transform from sensor frame to world frame
        """
        rep = np.zeros(2)

        if self.sector_mins is None or self.current_pose is None or len(self.sector_mins) == 0:
            return rep

        current_yaw = self.get_yaw_from_pose(self.current_pose)
        rot = _yaw_rot2x2(current_yaw)

        for i in range(len(self.sector_mins)):
            d = self.sector_mins[i]
            if d >= self.rep_threshold or not np.isfinite(d):
                continue

            # Get angle of this sector in LIDAR frame
            a_lidar = self.sector_angles[i] if self.sector_angles is not None else (i + 0.5) * (2 * math.pi / len(self.sector_mins)) - math.pi

            # Transform obstacle position to world frame
            # obstacle in sensor frame
            px_sensor = d * math.cos(a_lidar)
            py_sensor = d * math.sin(a_lidar)

            # Transform to world frame
            p_sensor = np.array([px_sensor, py_sensor])
            p_world = np.dot(rot, p_sensor) + np.array([self.currentX, self.currentY])

            # Direction from obstacle to drone
            dir_vec = np.array([self.currentX, self.currentY]) - p_world
            dist = np.linalg.norm(dir_vec)

            if dist > 0.01:
                rep_mag = self.k_rep * ((1.0 / d) - (1.0 / self.rep_threshold)) ** 2
                rep_dir = dir_vec / dist
                rep += rep_mag * rep_dir

        return rep

    def _compute_desired_heading(self, goal_error: np.ndarray, 
                                 obstacles_nearby: bool) -> tuple:
        """
        Compute desired heading and speed considering goal and obstacles
        Returns: (desired_yaw, speed_scale, vy_body)
        """
        # Attractive force toward goal
        att = self.kp_xy * goal_error
        att_mag = np.linalg.norm(att)
        if att_mag > 0:
            att = (att / att_mag) * min(att_mag, self.max_xy_speed)
        else:
            att = np.zeros(2)

        # Repulsive force from obstacles
        rep = self._get_obstacle_repulsion()

        current_yaw = self.get_yaw_from_pose(self.current_pose)
        # Reduce speed if obstacles are close
        speed_scale = 1.0
        min_dist = self._get_min_obstacle_distance()
        
        if min_dist < self.safe_stopping_dist:
            if min_dist < 0.5:
                speed_scale = 0.1  # Critical: nearly stop
            elif min_dist < 1.0:
                speed_scale = 0.3  # Warning: slow down significantly
            else:
                speed_scale = 0.6  # Caution: moderate slowdown
        elif min_dist < self.rep_threshold:
            speed_scale = 0.8  # Light slowdown

        if self.use_gap_nav and obstacles_nearby and self.gap_info is not None and self.gap_info['clearance'] > 0.5 and self.gap_info['width'] > self.gap_heading_window:
            # Use gap navigation when nearby obstacles
            gap_angle = self.gap_info['angle']
            desired_yaw = wrap_to_pi(current_yaw + gap_angle)
            speed_scale = max(0.3, min(1.0, self.gap_info['clearance'] / self.rep_threshold))  # Scale speed with gap clearance
            vy_body = 0.0
        else:
            # Use potential field
            total = att + rep
            total_mag = np.linalg.norm(total)
            if total_mag > self.max_xy_speed:
                total = (total / total_mag) * self.max_xy_speed
            desired_yaw = math.atan2(total[1], total[0]) if total_mag > 0 else current_yaw
            vy_body = 0.0

            # Stuck detection
            if total_mag < 0.1 and min_dist < self.safe_stopping_dist:
                self.stuck_counter += 1
                if self.stuck_counter > 5:  # Increased threshold
                    # Find the side with the farther average distance (more open space)
                    if self.sector_angles is not None and self.sector_mins is not None:
                        left_mask = self.sector_angles > 0
                        right_mask = self.sector_angles < 0
                        left_mins = self.sector_mins[left_mask]
                        right_mins = self.sector_mins[right_mask]
                        left_mins = left_mins[np.isfinite(left_mins)]
                        right_mins = right_mins[np.isfinite(right_mins)]
                        left_avg = np.mean(left_mins) if len(left_mins) > 0 else 0
                        right_avg = np.mean(right_mins) if len(right_mins) > 0 else 0
                        # Choose the side with the larger average distance
                        # if left_avg > right_avg:
                        #     turn_angle = math.pi / 3  # Turn left 90 degrees
                        #     # vy_body = 0.8  # Move left
                        # else:
                        #     turn_angle = -math.pi / 3  # Turn right 90 degrees
                        #     # vy_body = -0.8  # Move right
                        vy_body = np.random.choice([-1, 1]) * 1.0       # Stronger random push
                        desired_yaw += np.random.uniform(-math.pi/6, math.pi/6) # Small random turn
                        # desired_yaw = wrap_to_pi(current_yaw + turn_angle)
                        speed_scale = 0.5
            else:
                self.stuck_counter = 0

        return desired_yaw, speed_scale, vy_body

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
        if self.currentZ is not None and self.currentZ < 0.5:
            self.set_status("Landed")

    def landed(self):
        self.zero_twist()
        if self.last_cmd == 'takeoff':
            self.set_status('Taking off')
        elif self.last_cmd == 'move_to_goal':
            self.set_status('Taking off')

    def arrived_at_goal(self):
        gz = _desired_height(self.target_height)
        vz = self._vz_hold(gz)
        self._cmd_vel(0.0, 0.0, vz)
        if self.last_cmd == 'land':
            self.set_status('Landing')

    def taking_off(self):
        tgt = _desired_height(self.target_height)
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
            vz = self._vz_hold(self.currentZ if self.currentZ is not None else 0.0)
            self._cmd_vel(0.0, 0.0, vz)
            return

        gx, gy, _ = self.goal_xyz  # ignore goal z
        ex = gx - self.currentX
        ey = gy - self.currentY

        # Use set/desired height for vertical control
        gz = _desired_height(self.target_height)
        ez = gz - (self.currentZ if self.currentZ is not None else gz)
        ...
        # When publishing vertical velocity, use gz from desired height
        vz = self._vz_hold(gz)

        # Check if goal reached
        if abs(ex) <= self.pos_tol_xy and abs(ey) <= self.pos_tol_xy and abs(ez) <= self.pos_tol_z:
            self.set_status('Arrived at Goal')
            self.pub_goal_dist.publish(Float32(data=0.0))
            self.pub_goal_time.publish(Float32(data=0.0))
            vz = self._vz_hold(gz)
            self._cmd_vel(0.0, 0.0, vz)
            return

        vz = self._vz_hold(gz)

        # Check for nearby obstacles
        min_dist = self._get_min_obstacle_distance()
        obstacles_nearby = min_dist < self.rep_threshold

        # Compute desired heading and speed scale
        goal_error = np.array([ex, ey])
        desired_yaw, speed_scale, vy_body = self._compute_desired_heading(goal_error, obstacles_nearby)

        # Get current yaw
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        yaw_error = wrap_to_pi(desired_yaw - current_yaw)
        yaw_rate_cmd = self.k_yaw * yaw_error

        # Compute body frame velocity
        v_cmd = np.linalg.norm(goal_error)
        v_cmd = min(v_cmd * self.kp_xy, self.max_xy_speed) * speed_scale

        vx_body = v_cmd if abs(yaw_error) < math.radians(10) else v_cmd * 0.7

        self._cmd_vel(vx_body, vy_body, vz, yaw_rate_cmd)
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
        if self.goal_xyz is None or self.currentX is None:
            return
        gx, gy, _ = self.goal_xyz
        ex = gx - self.currentX
        ey = gy - self.currentY
        gz = _desired_height(self.target_height)
        ez = (self.currentZ if self.currentZ is not None else gz) - gz
        distance = math.sqrt(ex*ex + ey*ey + ez*ez)
        self.pub_goal_dist.publish(Float32(data=float(distance)))
        self.pub_goal_time.publish(Float32(data=float(distance / (self.max_xy_speed * 0.7) if self.max_xy_speed > 0 else 0.0)))

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