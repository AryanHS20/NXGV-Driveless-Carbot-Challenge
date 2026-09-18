"""Read-only ROS monitor for the staged Carbot V4 port.

The node subscribes to sensor feeds and publishes JSON diagnostics only. It has
no Twist publisher and cannot command the vehicle.
"""

import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .shadow_core import evaluate_inputs


STATUS_TOPIC = '/v4_experimental/status'


class ShadowMonitor(Node):
    def __init__(self) -> None:
        super().__init__('v4_shadow_monitor')
        self.declare_parameter('enabled', False)
        self.declare_parameter('primary_camera_topic', '/camera/color/image_raw')
        self.declare_parameter('secondary_camera_topic', '/camera/second/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('uwb_topic', '/uwb_fix')
        self.declare_parameter('camera_timeout_sec', 0.45)
        self.declare_parameter('scan_timeout_sec', 0.50)
        self.declare_parameter('odom_timeout_sec', 0.25)
        self.declare_parameter('uwb_timeout_sec', 1.00)
        self.declare_parameter('require_secondary_camera', True)
        self.declare_parameter('require_uwb', False)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._last_seen = {
            'primary_camera': None,
            'secondary_camera': None,
            'scan': None,
            'odom': None,
            'uwb': None,
        }
        self._timeouts = {
            'primary_camera': float(self.get_parameter('camera_timeout_sec').value),
            'secondary_camera': float(self.get_parameter('camera_timeout_sec').value),
            'scan': float(self.get_parameter('scan_timeout_sec').value),
            'odom': float(self.get_parameter('odom_timeout_sec').value),
            'uwb': float(self.get_parameter('uwb_timeout_sec').value),
        }

        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(
            Image,
            str(self.get_parameter('primary_camera_topic').value),
            lambda _msg: self._mark('primary_camera'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('secondary_camera_topic').value),
            lambda _msg: self._mark('secondary_camera'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            lambda _msg: self._mark('scan'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            lambda _msg: self._mark('odom'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('uwb_topic').value),
            self._uwb_callback,
            10,
        )
        self.create_timer(0.2, self._publish_status)

        state = 'enabled' if self._enabled else 'disabled'
        self.get_logger().warning(
            f'V4 experimental shadow monitor is {state}; '
            'motion authority is permanently false'
        )

    def _mark(self, name: str) -> None:
        self._last_seen[name] = time.monotonic()

    def _uwb_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict) and payload.get('valid') is True:
            self._mark('uwb')

    def _publish_status(self) -> None:
        required = ['primary_camera', 'scan', 'odom']
        if bool(self.get_parameter('require_secondary_camera').value):
            required.append('secondary_camera')
        if bool(self.get_parameter('require_uwb').value):
            required.append('uwb')

        report = evaluate_inputs(
            time.monotonic(), self._last_seen, self._timeouts, required
        )
        report.update({
            'enabled': self._enabled,
            'mode': 'shadow',
            'algorithm_stage': 0,
            'can_publish_motion': False,
        })
        if not self._enabled:
            report['ready_for_shadow_evaluation'] = False
        msg = String()
        msg.data = json.dumps(report, separators=(',', ':'), sort_keys=True)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShadowMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
