#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
import tf_transformations as tft
import heapq
from scipy.ndimage import binary_dilation

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
        self.declare_parameter('k_yaw',          1.5)  # Yaw gain
        self.declare_parameter('max_yaw_rate',   0.785)  # rad/s (~45 deg/s)
        self.declare_parameter('grid_res', 0.2)  # meters per cell
        self.declare_parameter('grid_offset', 20.0)  # offset for negative coords
        self.declare_parameter('grid_size', 200)  # cells, cover -20 to 20 m
        self.declare_parameter('drone_radius', 0.5)  # for dilation
        self.declare_parameter('local_max_dist', 10.0)  # max for local goal if global far

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
        self.k_yaw        = float(self.get_parameter('k_yaw').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.grid_res     = float(self.get_parameter('grid_res').value)
        self.grid_offset  = float(self.get_parameter('grid_offset').value)
        self.grid_size    = int(self.get_parameter('grid_size').value)
        self.drone_radius = float(self.get_parameter('drone_radius').value)
        self.local_max_dist = float(self.get_parameter('local_max_dist').value)

        # --- Subscribers ---
        self.sub_cmd      = self.create_subscription(String,       '/cmd/control',   self.on_cmd,   10)
        self.sub_goal     = self.create_subscription(PointStamped, '/cmd/goal',      self.on_goal,  10)
        self.sub_height_s = self.create_subscription(Float32,      '/cmd/height',    self.on_height,10)
        self.sub_pose_s   = self.create_subscription(PoseStamped,  '/drone/pose_1hz',self.on_pose,  10)
        self.sub_odom     = self.create_subscription(Odometry,     '/odometry/filtered', self.on_odom, 10)
        self.sub_scan     = self.create_subscription(LaserScan,    '/scan',          self.on_scan,  10)

        self.velX = self.velY = self.velZ = None
        self.k_brake_xy      = 1.25
        self.stop_speed_xy   = 0.05
        self.max_brake_speed = 1.5

        self.scan = None  # Latest LaserScan for A*

        # Occupancy grid (-1 unknown, 0 free, 1 occupied)
        self.occ_grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        radius = int(self.drone_radius / self.grid_res)
        self.structure = np.ones((2 * radius + 1, 2 * radius + 1), dtype=bool)

        # Internal state
        self.status        = "Pre Flight Checks"
        self._last_status  = None
        self.last_cmd      = None
        self.target_height = None
        self.goal_xyz      = None
        self.hover_z       = None
        self.current_pose  = None
        self.currentX      = None
        self.currentY      = None
        self.currentZ      = None
        self._int_z        = 0.0
        self._int_z_max    = 1.0
        self._last_ctrl_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / max(1.0, self.ctrl_hz), self.main_loop)
        self.set_status(self.status)

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

    def on_odom(self, msg: Odometry):
        t = msg.twist.twist
        self.velX = float(t.linear.x)
        self.velY = float(t.linear.y)
        self.velZ = float(t.linear.z)

    def on_scan(self, msg: LaserScan):
        self.scan = msg

        if self.currentX is None or self.currentY is None or self.current_pose is None:
            return

        q = [self.current_pose.orientation.x, self.current_pose.orientation.y,
             self.current_pose.orientation.z, self.current_pose.orientation.w]
        current_yaw = tft.euler_from_quaternion(q)[2]

        for i in range(len(msg.ranges)):
            r = msg.ranges[i]
            if not np.isfinite(r):
                r = msg.range_max

            if r < msg.range_min:
                continue

            a = msg.angle_min + i * msg.angle_increment + current_yaw

            cap_r = min(r, msg.range_max)
            end_x = self.currentX + cap_r * math.cos(a)
            end_y = self.currentY + cap_r * math.sin(a)

            start_ix = int((self.currentX + self.grid_offset) / self.grid_res)
            start_iy = int((self.currentY + self.grid_offset) / self.grid_res)
            end_ix = int((end_x + self.grid_offset) / self.grid_res)
            end_iy = int((end_y + self.grid_offset) / self.grid_res)

            line = self.bresenham(start_iy, start_ix, end_iy, end_ix)

            for iy, ix in line:
                if 0 <= ix < self.grid_size and 0 <= iy < self.grid_size:
                    self.occ_grid[iy, ix] = 0  # free

            if r < msg.range_max:
                if 0 <= end_ix < self.grid_size and 0 <= end_iy < self.grid_size:
                    self.occ_grid[end_iy, end_ix] = 1  # occupied

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

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

    def _cmd_vel(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        tw = Twist()
        tw.linear.x = float(self.clamp(vx, -self.max_xy_speed, self.max_xy_speed))
        tw.linear.y = float(self.clamp(vy, -self.max_xy_speed, self.max_xy_speed))
        tw.linear.z = float(vz)
        tw.angular.z = float(self.clamp(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate))
        self.pub_cmd_vel.publish(tw)

    def _a_star(self, start, goal, dilated):
        heap = []
        heapq.heappush(heap, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: math.hypot(goal[0] - start[0], goal[1] - start[1])}
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while heap:
            _, current = heapq.heappop(heap)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dy, dx in directions:
                neighbor = (current[0] + dy, current[1] + dx)
                if 0 <= neighbor[0] < self.grid_size and 0 <= neighbor[1] < self.grid_size and not dilated[neighbor[0], neighbor[1]]:
                    cost = 1.4 if abs(dy) + abs(dx) == 2 else 1.0
                    tent_g = g_score[current] + cost
                    if neighbor not in g_score or tent_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tent_g
                        f_score[neighbor] = tent_g + math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                        heapq.heappush(heap, (f_score[neighbor], neighbor))

        return None

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
        if self.currentX is None or self.goal_xyz is None or self.current_pose is None:
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

        q = [self.current_pose.orientation.x, self.current_pose.orientation.y,
             self.current_pose.orientation.z, self.current_pose.orientation.w]
        current_yaw = tft.euler_from_quaternion(q)[2]

        dist_to_goal = math.sqrt(ex**2 + ey**2)
        gx_local = gx
        gy_local = gy
        if dist_to_goal > self.local_max_dist:
            angle_to_goal = math.atan2(ey, ex)
            gx_local = self.currentX + self.local_max_dist * math.cos(angle_to_goal)
            gy_local = self.currentY + self.local_max_dist * math.sin(angle_to_goal)

        start_iy = int((self.currentY + self.grid_offset) / self.grid_res)
        start_ix = int((self.currentX + self.grid_offset) / self.grid_res)
        goal_iy = int((gy_local + self.grid_offset) / self.grid_res)
        goal_ix = int((gx_local + self.grid_offset) / self.grid_res)

        dilated = binary_dilation(self.occ_grid == 1, structure=self.structure)

        path = self._a_star((start_iy, start_ix), (goal_iy, goal_ix), dilated)

        if path is None or len(path) < 2:
            self.get_logger().warn("No path found, rotating to explore")
            self._cmd_vel(0.0, 0.0, vz, self.max_yaw_rate * 0.3)
        else:
            self.get_logger().info(f"Path found, length {len(path)}")
            look_idx = min(3, len(path) - 1)
            next_iy, next_ix = path[look_idx]
            next_y = next_iy * self.grid_res - self.grid_offset
            next_x = next_ix * self.grid_res - self.grid_offset

            ex_next = next_x - self.currentX
            ey_next = next_y - self.currentY
            dist_next = math.sqrt(ex_next**2 + ey_next**2)
            angle_to_next = math.atan2(ey_next, ex_next)
            yaw_error = angle_to_next - current_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            yaw_rate_cmd = self.k_yaw * yaw_error

            vx_body = self.kp_xy * dist_next
            vx_body = self.clamp(vx_body, 0.1, self.max_xy_speed)

            ex_body_next = math.cos(current_yaw) * ex_next + math.sin(current_yaw) * ey_next
            ey_body_next = -math.sin(current_yaw) * ex_next + math.cos(current_yaw) * ey_next
            desired_angle_body = math.atan2(ey_body_next, ex_body_next)
            yaw_error_body = math.atan2(math.sin(desired_angle_body), math.cos(desired_angle_body))
            yaw_rate_cmd = self.k_yaw * yaw_error_body

            self._cmd_vel(vx_body, 0.0, vz, yaw_rate_cmd)

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