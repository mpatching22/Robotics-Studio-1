#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class AltitudeMixer(Node):
    """
    Merges Nav2 XY/yaw velocity with a smooth PD altitude controller.
    Input:
      - /nav/cmd_vel        (Twist)  ← from Nav2 controller_server
      - /odometry           (Odometry) for z & ż (preferred)
         or /drone/pose_1hz (PoseStamped) if odom unavailable
      - /cmd/height         (Float32) Z setpoint (m)  ← set by GUI/FC
    Output:
      - /cmd_vel            (Twist) final command to Gazebo/bridge
    """

    def __init__(self):
        super().__init__('altitude_mixer')

        # ---- params ----
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('kp_z', 1.2)            # position gain
        self.declare_parameter('kd_z', 0.8)            # damping gain
        self.declare_parameter('max_up', 1.5)          # m/s up
        self.declare_parameter('max_down', 1.0)        # m/s down (positive number)
        self.declare_parameter('deadband', 0.05)       # m
        self.declare_parameter('use_odom', True)

        self.rate = float(self.get_parameter('rate_hz').value)
        self.kp = float(self.get_parameter('kp_z').value)
        self.kd = float(self.get_parameter('kd_z').value)
        self.max_up = float(self.get_parameter('max_up').value)
        self.max_dn = float(self.get_parameter('max_down').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.use_odom = bool(self.get_parameter('use_odom').value)

        # ---- state ----
        self.nav_cmd = Twist()         # latest Nav2 velocity
        self.nav_cmd_time = 0.0
        self.z_target = 2.0
        self.z = None
        self.z_dot = 0.0
        self.last_z = None
        self.last_t = None

        # ---- pubs/subs ----
        qos = 10
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', qos)
        self.sub_nav = self.create_subscription(Twist, '/nav/cmd_vel', self._on_nav_cmd, qos)

        if self.use_odom:
            self.sub_odom = self.create_subscription(Odometry, '/odometry', self._on_odom, qos)
        else:
            self.sub_pose = self.create_subscription(PoseStamped, '/drone/pose_1hz', self._on_pose, qos)

        self.sub_height = self.create_subscription(Float32, '/cmd/height', self._on_height, qos)

        self.timer = self.create_timer(max(0.01, 1.0/self.rate), self._tick)
        self.get_logger().info("AltitudeMixer ready (Nav2 on /nav/cmd_vel → out /cmd_vel).")

    def _on_height(self, msg: Float32):
        self.z_target = float(msg.data)
        self.get_logger().info(f"Altitude setpoint: {self.z_target:.2f} m")

    def _on_nav_cmd(self, msg: Twist):
        self.nav_cmd = msg
        self.nav_cmd_time = time.time()

    def _on_odom(self, msg: Odometry):
        z_now = float(msg.pose.pose.position.z)
        t_now = self.get_clock().now().nanoseconds * 1e-9
        self.z = z_now
        # Use odom twist if sane, else finite-diff
        zd_meas = float(msg.twist.twist.linear.z)
        if math.isfinite(zd_meas) and abs(zd_meas) < 50.0:
            self.z_dot = zd_meas
        else:
            if self.last_z is not None and self.last_t is not None:
                dt = max(1e-3, t_now - self.last_t)
                # simple low-pass derivative
                alpha = 0.3
                self.z_dot = (1-alpha)*self.z_dot + alpha*((z_now - self.last_z)/dt)
        self.last_z, self.last_t = z_now, t_now

    def _on_pose(self, msg: PoseStamped):
        # fallback if no odom
        z_now = float(msg.pose.position.z)
        t_now = self.get_clock().now().nanoseconds * 1e-9
        self.z = z_now
        if self.last_z is not None and self.last_t is not None:
            dt = max(1e-3, t_now - self.last_t)
            alpha = 0.3
            self.z_dot = (1-alpha)*self.z_dot + alpha*((z_now - self.last_z)/dt)
        self.last_z, self.last_t = z_now, t_now

    def _pd_altitude(self, e, edot):
        # Smooth approach to reduce overshoot: limit P via tanh(), then add damping
        v_p = self.max_up * math.tanh(self.kp * e / max(1e-6, self.max_up))
        v_d = - self.kd * edot
        v = v_p + v_d

        # Deadband around target to stop hunting
        if abs(e) < self.deadband and abs(edot) < 0.02:
            return 0.0

        # Clamp asymmetric (slower down than up)
        v = min(self.max_up, max(-self.max_dn, v))
        return v

    def _tick(self):
        out = Twist()

        # Use stale-check so XY go to zero if Nav2 is not commanding
        nav_stale = (time.time() - self.nav_cmd_time) > 0.5
        if not nav_stale:
            out.linear.x = self.nav_cmd.linear.x
            out.linear.y = self.nav_cmd.linear.y
            out.angular.z = self.nav_cmd.angular.z

        # Altitude loop
        if self.z is not None:
            e = self.z_target - self.z
            vz = self._pd_altitude(e, self.z_dot)
            out.linear.z = vz
        else:
            # No altitude estimate yet → hold Z = 0 velocity
            out.linear.z = 0.0

        self.pub_cmd.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
