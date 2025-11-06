#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry


class PoseRelay(Node):
    def __init__(self):
        super().__init__('pose_relay')

        # Parameters
        self.declare_parameter('source_topic', '/odometry')   # or '/odometry'
        self.declare_parameter('output_topic', '/drone/pose_1hz')
        self.declare_parameter('point_topic',  '/drone/position')
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('source_type', 'pose')           # 'pose' or 'odom'
        self.declare_parameter('frame_id_override', '')

        src_topic      = self.get_parameter('source_topic').value
        out_topic      = self.get_parameter('output_topic').value
        point_topic    = self.get_parameter('point_topic').value
        self.rate      = float(self.get_parameter('rate_hz').value)
        self.source_type   = str(self.get_parameter('source_type').value)
        self.frame_override = str(self.get_parameter('frame_id_override').value)

        # QoS: tolerant subscriber, easy-to-consume publishers
        qos_sub = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_pub = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscribe to pose or odom
        self.last_pose = None
        self.message_count = 0
        self.last_publish_time = 0.0

        if self.source_type.lower().startswith('odom'):
            self.subscriber = self.create_subscription(Odometry, src_topic, self._odom_cb, qos_sub)
        else:
            self.subscriber = self.create_subscription(PoseStamped, src_topic, self._pose_cb, qos_sub)

        # Publishers
        self.pub_pose  = self.create_publisher(PoseStamped, out_topic, qos_pub)
        self.pub_point = self.create_publisher(PointStamped, point_topic, qos_pub)

        # Timers
        timer_rate = max(self.rate, 0.1)
        self.timer = self.create_timer(1.0 / timer_rate, self._tick)
        # self.status_timer = self.create_timer(5.0, self._status_update)

        # self.get_logger().info(
        #     f'Relaying [{self.source_type}] from "{src_topic}" -> "{out_topic}" at {self.rate:.2f} Hz'
        # )
        # self.get_logger().info(f'Timer period: {1.0 / timer_rate:.2f} s')
        # self.get_logger().info('Waiting for pose data...')

    def _pose_cb(self, msg: PoseStamped):
        self.last_pose = msg
        self.message_count += 1
        if self.message_count <= 3:
            p = msg.pose.position
            # self.get_logger().info(f'Received pose #{self.message_count}: pos=({p.x:.2f},{p.y:.2f},{p.z:.2f}) frame={msg.header.frame_id}')

    def _odom_cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._pose_cb(ps)

    def _tick(self):
        if self.last_pose is None:
            # log occasionally to avoid spam
            return
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_override or self.last_pose.header.frame_id
        out.pose = self.last_pose.pose

        pt = PointStamped()
        pt.header = out.header
        pt.point = out.pose.position

        self.pub_pose.publish(out)
        self.pub_point.publish(pt)

    # def _status_update(self):
        # self.get_logger().info(
        #     f'Status: Received {self.message_count} input msgs; last_pose: {"Available" if self.last_pose else "None"}'
        # )


def main():
    rclpy.init()
    node = PoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# Call ros2 topic echo /drone/pose_1hz to see pose (in another sourced terminal)