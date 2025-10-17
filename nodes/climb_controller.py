#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import math

class DroneClimbController(Node):
    def __init__(self):
        super().__init__('drone_climb_controller')

        # Params
        self.declare_parameter('pose_topic', '/drone/pose_1hz')  
        self.declare_parameter('msg_type', 'pose')               
        self.declare_parameter('climb_speed_mps', 0.1)           # Desired climb rate
        self.declare_parameter('publish_rate_hz', 20.0)          # Higher rate for stability
        self.declare_parameter('target_z', 15.0)                 
        self.declare_parameter('timeout_sec', 2.0)               
        
        # Drone physical parameters
        self.declare_parameter('drone_mass_kg', 0.503)           # From URDF
        self.declare_parameter('gravity_mps2', 9.81)             # From world file
        self.declare_parameter('hover_thrust_scale', 1.0)        # Tuning parameter
        self.declare_parameter('climb_thrust_scale', 1.2)        # Extra thrust for climbing

        self.pose_topic = self.get_parameter('pose_topic').value
        self.msg_type   = self.get_parameter('msg_type').value
        self.climb_speed = float(self.get_parameter('climb_speed_mps').value)
        self.pub_hz     = float(self.get_parameter('publish_rate_hz').value)
        self.target_z   = float(self.get_parameter('target_z').value)
        self.timeout_s  = float(self.get_parameter('timeout_sec').value)
        
        # Physical parameters
        self.mass = float(self.get_parameter('drone_mass_kg').value)
        self.gravity = float(self.get_parameter('gravity_mps2').value)
        self.hover_scale = float(self.get_parameter('hover_thrust_scale').value)
        self.climb_scale = float(self.get_parameter('climb_thrust_scale').value)
        
        # Calculate required thrust to counteract gravity
        self.gravity_compensation = self.gravity * self.hover_scale
        self.climb_thrust = self.gravity * self.climb_scale

        # QoS
        qos_sub = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        qos_pub = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)

        # State
        self.current_z = None
        self.previous_z = None
        self.current_velocity_z = 0.0
        self.last_pose_time = 0.0
        self.message_count = 0
        self.command_count = 0
        self.last_z_time = 0.0

        # Control state
        self.control_mode = "WAITING"  # WAITING, HOVERING, CLIMBING, TARGET_REACHED

        # Sub
        if self.msg_type.lower().startswith('odom'):
            self.subscriber = self.create_subscription(Odometry, self.pose_topic, self._odom_cb, qos_sub)
        else:
            self.subscriber = self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, qos_sub)

        # Pub
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_pub)

        # Timer
        self.timer = self.create_timer(1.0 / max(self.pub_hz, 1e-3), self._tick)
        
        # Status timer for debugging
        self.status_timer = self.create_timer(3.0, self._status_update)

        self.get_logger().info(
            f'DroneClimbController: mass={self.mass:.3f}kg, gravity={self.gravity:.2f}m/s²'
        )
        self.get_logger().info(
            f'Thrust: hover={self.gravity_compensation:.2f}m/s², climb={self.climb_thrust:.2f}m/s²'
        )
        self.get_logger().info(
            f'Target: climb_speed={self.climb_speed:.2f}m/s, target_z={self.target_z:.2f}m, rate={self.pub_hz:.1f}Hz'
        )

    def _pose_cb(self, msg: PoseStamped):
        current_time = time.time()
        
        # Calculate velocity if we have previous data
        if self.current_z is not None and self.last_z_time > 0:
            dt = current_time - self.last_z_time
            if dt > 0:
                self.current_velocity_z = (msg.pose.position.z - self.current_z) / dt
        
        self.previous_z = self.current_z
        self.current_z = msg.pose.position.z
        self.last_pose_time = current_time
        self.last_z_time = current_time
        self.message_count += 1
        
        # Debug first few messages
        if self.message_count <= 5:
            self.get_logger().info(
                f'Pose #{self.message_count}: z={self.current_z:.3f}m, vz={self.current_velocity_z:.3f}m/s'
            )

    def _odom_cb(self, msg: Odometry):
        # Extract pose from odometry
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self._pose_cb(pose_msg)
        
        # Also get velocity directly from odometry if available
        if hasattr(msg.twist.twist.linear, 'z'):
            self.current_velocity_z = msg.twist.twist.linear.z

    def _tick(self):
        now = time.time()
        twist = Twist()
        
        # Check if pose data is available and fresh
        pose_stale = (now - self.last_pose_time) > self.timeout_s
        
        if self.current_z is None:
            self.control_mode = "WAITING"
            if self.command_count % 60 == 0:  # Log every 3 seconds at 20Hz
                self.get_logger().warn('No pose data received yet')
            self.cmd_pub.publish(twist)  # Publish zero - drone will fall
            self.command_count += 1
            return

        if pose_stale:
            self.control_mode = "WAITING"
            if self.command_count % 60 == 0:
                self.get_logger().warn(f'Pose data is stale ({now - self.last_pose_time:.1f}s old)')
            self.cmd_pub.publish(twist)  # Publish zero - drone will fall
            self.command_count += 1
            return

        # Determine control mode and command
        z_error = self.target_z - self.current_z
        
        if abs(z_error) < 0.5:  # Within 50cm of target
            self.control_mode = "TARGET_REACHED"
            # Just hover - counteract gravity
            twist.linear.z = self.gravity_compensation
            
        elif z_error > 0:  # Need to climb
            self.control_mode = "CLIMBING"
            
            # Use more aggressive thrust for climbing
            # Base thrust to counter gravity + extra for climbing
            if self.current_velocity_z < self.climb_speed:
                # Need more thrust to reach climb speed
                thrust_command = self.climb_thrust
            else:
                # Maintaining climb speed, use hover + small margin
                thrust_command = self.gravity_compensation + 1.0
            
            twist.linear.z = thrust_command
            
        else:  # Need to descend (z_error < 0)
            self.control_mode = "DESCENDING" 
            # Reduce thrust to descend, but don't let drone drop too fast
            twist.linear.z = max(0.0, self.gravity_compensation - 2.0)

        # Safety limits
        MAX_THRUST = 20.0  # Maximum upward acceleration
        MIN_THRUST = 0.0   # Minimum thrust (let gravity work)
        
        twist.linear.z = max(MIN_THRUST, min(MAX_THRUST, twist.linear.z))

        # Publish command
        self.cmd_pub.publish(twist)
        self.command_count += 1
        
        # Debug first few commands
        if self.command_count <= 10 or self.command_count % 100 == 0:
            self.get_logger().info(
                f'Mode: {self.control_mode}, z={self.current_z:.3f}m, '
                f'target={self.target_z:.3f}m, vz={self.current_velocity_z:.3f}m/s, '
                f'thrust={twist.linear.z:.3f}m/s²'
            )

    def _status_update(self):
        """Periodic status report"""
        if self.current_z is not None:
            z_error = self.target_z - self.current_z
            self.get_logger().info(
                f'Status: {self.control_mode}, z={self.current_z:.3f}m (error={z_error:.3f}m), '
                f'vz={self.current_velocity_z:.3f}m/s, msgs={self.message_count}, cmds={self.command_count}'
            )
        else:
            self.get_logger().warn(
                f'Status: No pose data, listening on {self.pose_topic} ({self.msg_type})'
            )

def main():
    rclpy.init()
    node = DroneClimbController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()