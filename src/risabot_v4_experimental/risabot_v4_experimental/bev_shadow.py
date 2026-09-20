"""Two-camera calibration and BEV shadow node.

Outputs are debug images and JSON diagnostics under /v4_experimental only.
This node imports no motion message and owns no command publisher.
"""

import json
import time
from typing import Dict

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .bev_core import CalibrationError, load_profiles, profile_report, warp_to_bev


STATUS_TOPIC = '/v4_experimental/bev/status'
PRIMARY_BEV_TOPIC = '/v4_experimental/bev/primary/image'
PRIMARY_COVERAGE_TOPIC = '/v4_experimental/bev/primary/coverage'
SECONDARY_BEV_TOPIC = '/v4_experimental/bev/secondary/image'
SECONDARY_COVERAGE_TOPIC = '/v4_experimental/bev/secondary/coverage'


class BevShadow(Node):
    def __init__(self) -> None:
        super().__init__('v4_bev_shadow')
        self.declare_parameter('enabled', False)
        self.declare_parameter('profile_path', '')
        self.declare_parameter('primary_camera_topic', '/camera/color/image_raw')
        self.declare_parameter('secondary_camera_topic', '/camera/second/image_raw')
        self.declare_parameter('process_secondary', True)
        self.declare_parameter('max_hz', 10.0)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._active_names = (
            ('primary', 'secondary')
            if bool(self.get_parameter('process_secondary').value)
            else ('primary',)
        )
        self._max_hz = max(0.1, float(self.get_parameter('max_hz').value))
        self._bridge = CvBridge()
        self._profiles = {}
        self._profile_error = ''
        self._last_output: Dict[str, float] = {name: 0.0 for name in self._active_names}
        self._frames: Dict[str, int] = {name: 0 for name in self._active_names}
        self._coverage: Dict[str, float] = {name: 0.0 for name in self._active_names}
        self._last_error: Dict[str, str] = {name: '' for name in self._active_names}

        profile_path = str(self.get_parameter('profile_path').value)
        try:
            self._profiles = load_profiles(profile_path)
        except (CalibrationError, OSError, ValueError) as exc:
            self._profile_error = str(exc)
            self.get_logger().error(f'V4 calibration profiles rejected: {exc}')

        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._image_pubs = {
            name: self.create_publisher(
                Image,
                PRIMARY_BEV_TOPIC if name == 'primary' else SECONDARY_BEV_TOPIC,
                2,
            )
            for name in self._active_names
        }
        self._coverage_pubs = {
            name: self.create_publisher(
                Image,
                PRIMARY_COVERAGE_TOPIC if name == 'primary' else SECONDARY_COVERAGE_TOPIC,
                2,
            )
            for name in self._active_names
        }
        for name in self._active_names:
            self.create_subscription(
                Image,
                str(self.get_parameter(f'{name}_camera_topic').value),
                lambda msg, camera=name: self._image_callback(camera, msg),
                qos_profile_sensor_data,
            )
        self.create_timer(1.0, self._publish_status)

        state = 'enabled' if self._enabled else 'disabled'
        self.get_logger().warning(
            f'V4 BEV shadow is {state}; outputs are diagnostics only and '
            'motion authority is permanently false'
        )

    def _image_callback(self, name: str, msg: Image) -> None:
        if not self._enabled or name not in self._profiles:
            return
        profile = self._profiles[name]
        if not profile.calibrated:
            self._last_error[name] = 'profile is explicitly uncalibrated'
            return
        now = time.monotonic()
        if now - self._last_output[name] < (1.0 / self._max_hz):
            return
        try:
            source = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            bev, coverage = warp_to_bev(source, profile)
            bev_msg = self._bridge.cv2_to_imgmsg(bev, encoding='bgr8')
            coverage_msg = self._bridge.cv2_to_imgmsg(coverage, encoding='mono8')
        except (CalibrationError, CvBridgeError, ValueError) as exc:
            self._last_error[name] = str(exc)
            return

        bev_msg.header = msg.header
        coverage_msg.header = msg.header
        bev_msg.header.frame_id = f'v4_bev_{name}'
        coverage_msg.header.frame_id = f'v4_bev_{name}'
        self._image_pubs[name].publish(bev_msg)
        self._coverage_pubs[name].publish(coverage_msg)
        self._last_output[name] = now
        self._frames[name] += 1
        self._coverage[name] = float((coverage > 0).sum()) / float(coverage.size)
        self._last_error[name] = ''

    def _publish_status(self) -> None:
        reports = {
            name: profile_report(profile)
            for name, profile in self._profiles.items()
            if name in self._active_names
        }
        calibrated = bool(reports) and all(
            item['calibrated'] for item in reports.values()
        )
        payload = {
            'algorithm_stage': 1,
            'mode': 'shadow',
            'enabled': self._enabled,
            'motion_authority': False,
            'can_publish_motion': False,
            'profiles_valid': not self._profile_error,
            'calibration_complete': calibrated,
            'active_profiles': list(self._active_names),
            'profile_error': self._profile_error,
            'profiles': reports,
            'frames_published': self._frames,
            'coverage_fraction': self._coverage,
            'last_error': self._last_error,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BevShadow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
