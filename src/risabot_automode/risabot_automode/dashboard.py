#!/usr/bin/env python3
"""
Robot Dashboard — Web-based status monitor
Subscribes to all key ROS topics and serves a live web dashboard
at http://<robot_ip>:8080

Run standalone:  python3 dashboard.py
Or via launch:   included in competition.launch.py
"""

import http.server
import json
import math
import re
import socketserver
import threading
import time
import os
import yaml
import numpy as np
from .parameter_values import coerce_parameter_value

import cv2
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as RosParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, Joy, LaserScan
from std_msgs.msg import Bool, Float32, String

from .topics import (
    AUTO_CMD_VEL_TOPIC,
    AUTO_MODE_TOPIC,
    BOOM_GATE_TOPIC,
    CMD_SAFETY_STATUS_TOPIC,
    CAMERA_DEBUG_LINE_TOPIC,
    CAMERA_DEBUG_OBS_TOPIC,
    CAMERA_DEBUG_TL_TOPIC,
    CAMERA_IMAGE_TOPIC,
    MIPI_SECONDARY_TOPIC,
    MIPI_TERTIARY_TOPIC,
    SIDE_CAMERA_REQUEST_TOPIC,
    CMD_VEL_TOPIC,
    DASH_CTRL_TOPIC,
    DASH_STATE_TOPIC,
    IMU_DATA_TOPIC,
    IMU_CALIBRATE_TOPIC,
    JOY_TOPIC,
    LANE_ERROR_TOPIC,
    OBSTACLE_CAMERA_TOPIC,
    OBSTACLE_FUSED_TOPIC,
    OBSTACLE_LIDAR_TOPIC,
    ODOM_TOPIC,
    OBSTRUCTION_ACTIVE_TOPIC,
    LOOP_STATS_TOPIC,
    PARKING_COMPLETE_TOPIC,
    HEALTH_STATUS_TOPIC,
    SET_CHALLENGE_TOPIC,
    SIGNAGE_DEBUG_TOPIC,
    TRAFFIC_LIGHT_TOPIC,
    TUNNEL_DETECTED_TOPIC,
    PARKING_SIGN_TOPIC,
    RECORD_PLAYBACK_STATE_TOPIC,
    RECORD_PLAYBACK_CMD_TOPIC,
)

try:
    from cv_bridge import CvBridge, CvBridgeError
except ImportError:
    CvBridge = None
    CvBridgeError = Exception

from .dashboard_panels import registry

# ======================== HTML Dashboard ========================

from .dashboard_templates import DASHBOARD_HTML, TEACH_HTML

_DEFAULT_PARAMS = {}
_PARAMS_SOURCE_PATH = ''  # Legacy compatibility: automode source params path.
_PARAMS_SOURCE_PATHS = []

_PARAM_CONFIGS = (
    ('risabot_automode', 'params.yaml', os.path.join('..', 'config', 'params.yaml')),
    ('risabot_v4_experimental', 'v4_experimental.yaml',
     os.path.join('..', '..', 'risabot_v4_experimental', 'config', 'v4_experimental.yaml')),
    ('risabot_v4_control', 'v4_control.yaml',
     os.path.join('..', '..', 'risabot_v4_control', 'config', 'v4_control.yaml')),
)

_V4_DASHBOARD_SAFE_PARAMS = {
    'v4_bev_shadow': {'max_hz'},
    'v4_road_mask_shadow': {
        'value_min', 'value_max', 'saturation_max', 'morph_open_px',
        'morph_close_px', 'min_component_px', 'seed_radius_m',
        'min_corridor_width_m', 'max_corridor_width_m',
        'corridor_row_step_px', 'memory_planning_age_sec',
        'memory_planning_distance_m', 'memory_reset_jump_m',
        'memory_reset_yaw_rad',
    },
    'v4_trajectory_shadow': {
        'horizon_m', 'step_m', 'lookahead_m', 'rollout_speed_mps',
        'steering_lag_sec', 'steering_rate_rad_sec',
        'footprint_sample_spacing_m', 'minimum_road_support',
        'obstacle_margin_m',
    },
    'v4_motion_executor': {
        'forward_speed_mps', 'path_forward_speed_mps',
        'path_reverse_speed_mps', 'minimum_speed_scale',
        'hill_max_speed_mps',
    },
}

_V4_COMPOSITE_TILES = (
    ('bev', 'BEV'),
    ('coverage', 'COVERAGE'),
    ('candidate', 'CANDIDATE'),
    ('connected', 'CONNECTED'),
    ('fused', 'FUSED'),
)
_V4_COMPOSITE_LABELS = dict(_V4_COMPOSITE_TILES)
_V4_COMPOSITE_NAMES = frozenset(_V4_COMPOSITE_LABELS)
_V4_COMPOSITE_TILE_SIZE = (320, 240)
_V4_COMPOSITE_FOOTER_HEIGHT = 32
_V4_COMPOSITE_SYNC_WAIT_SEC = 0.15
_V4_COMPOSITE_STALE_SEC = 1.0
_V4_COMPOSITE_MAX_STAMP_SETS = 3
_DASHBOARD_DIAGNOSTIC_INTERVAL_SEC = 5.0

def load_default_params():
    global _DEFAULT_PARAMS, _PARAMS_SOURCE_PATH, _PARAMS_SOURCE_PATHS
    _DEFAULT_PARAMS = {}
    _PARAMS_SOURCE_PATH = ''
    _PARAMS_SOURCE_PATHS = []
    try:
        from ament_index_python.packages import get_package_share_directory
        this_dir = os.path.dirname(os.path.abspath(__file__))
        for package, filename, source_relative in _PARAM_CONFIGS:
            source_params = os.path.abspath(os.path.join(this_dir, source_relative))
            try:
                share_dir = get_package_share_directory(package)
                installed_params = os.path.join(share_dir, 'config', filename)
            except Exception:
                installed_params = ''
            params_file = source_params if os.path.exists(source_params) else installed_params
            if not params_file or not os.path.exists(params_file):
                print(f'Parameter config not found for {package}: {filename}')
                continue
            write_path = source_params if os.path.exists(source_params) else os.path.realpath(params_file)
            _PARAMS_SOURCE_PATHS.append(write_path)
            if package == 'risabot_automode':
                _PARAMS_SOURCE_PATH = write_path
            with open(params_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if data:
                    for node_name, node_data in data.items():
                        if isinstance(node_data, dict) and 'ros__parameters' in node_data:
                            if node_name not in _DEFAULT_PARAMS:
                                _DEFAULT_PARAMS[node_name] = {}
                            _DEFAULT_PARAMS[node_name].update(node_data['ros__parameters'])
            print(f'Loaded default params from {params_file}')
            print(f'Parameter save path: {write_path}')
    except Exception as e:
        print(f"Failed to load default params: {e}")

# ======================== ROS2 Dashboard Node ========================

class DashboardNode(Node):
    """ROS 2 node providing data and camera streams to the web dashboard."""
    def __init__(self):
        super().__init__('dashboard')
        self.get_logger().info('Dashboard node starting on http://0.0.0.0:8080')

        self.declare_parameter('use_hw_odom', False)
        self.declare_parameter('freshness_stale_sec', 2.0)
        self.declare_parameter('sim_odom_scale', 1.55)
        self.declare_parameter('hw_odom_scale', 1.0)
        self.declare_parameter('hw_odom_yaw_scale', 1.0)
        self.declare_parameter('cam_encode_max_hz', 5.0)  # MJPEG encode cap (CPU saver)
        self.declare_parameter('dashboard_port', 8080)  # HTTP port (8081 when the carbot GUI owns 8080)

        # CV Bridge for camera
        self.bridge = CvBridge() if CvBridge else None
        self.latest_jpeg = None
        self.jpeg_condition = threading.Condition()
        self.frame_id = 0
        self._encode_min_interval = 1.0 / max(1.0, float(self.get_parameter('cam_encode_max_hz').value))
        self._last_encode_mono = 0.0
        self._last_camera_source_mono = 0.0
        self.active_camera_view = 'raw'
        self.active_camera_source = 'forward'  # forward | second | third
        self._camera_selection_generation = 0
        self._init_v4_composite_state()
        self._v4telemetry = None
        self._v4telemetry_mono = 0.0
        self.initial_joy_axes = None

        # LiDAR scan storage for 2D visualization
        self.lidar_points = []  # [{x, y}]
        self.lidar_lock = threading.Lock()
        self.lidar_angle_offset = 3.1416  # default, same as tunnel node

        # Tunnel debug info for LiDAR overlay
        self.tunnel_debug = ''  # "left,right,error,angular_z"
        self.tunnel_debug_lock = threading.Lock()
        
        # Client tracking for performance
        self.num_camera_clients = 0
        self.camera_clients_lock = threading.Lock()

        # Shared state (read by HTTP handler)
        self.data = {
            'state': 'LANE_FOLLOW',
            'lap': 1,
            'auto_mode': False,
            'state_time': 0,
            'state_dist': '0.00',
            'traffic_light': 'unknown',
            'lidar_obstacle': None,
            'camera_obstacle': None,
            'fused_obstacle': None,
            'boom_gate': None,
            'tunnel_detected': None,
            'obstruction_active': None,
            'parking_complete': None,
            'stop_reason': '',
            'lane_error': 0.0,
            'cmd_lin_x': 0.0,
            'cmd_ang_z': 0.0,
            'distance': 0.0,
            'speed': 0.0,
            'odom_x': 0.0,
            'odom_y': 0.0,
            'odom_yaw': 0.0,
            'buttons': [],
            'axes': [],
            'speed_pct': 25,
            'ctrl_state_index': 0,
            'ctrl_state_name': 'LANE_FOLLOW',
            'health_ok': None,
            'health_summary': '',
            'health_stale': [],
            'cmd_safety_estop': False,
            'cmd_safety_timeout_count': 0,
            'cmd_safety_autonomy_source': 'unknown',
            'v4_status': {},
            'loop_stats': {},
            'rp_state': 'IDLE',
            'rp_buffer_size': 0,
            'rp_playback_index': 0,
            'rp_recording_name': '',
            'rp_active_parking': '',
            'rp_saved_recordings': [],
            'parking_sign_detected': None,
            # IMU
            'imu_roll':  0.0,
            'imu_pitch': 0.0,
            'imu_yaw':   0.0,
        }
        self.topic_last_update = {
            'auto_mode': 0.0,
            'obstacle_front': 0.0,
            'obstacle_camera': 0.0,
            'obstacle_fused': 0.0,
            'traffic_light': 0.0,
            'boom_gate': 0.0,
            'tunnel_detected': 0.0,
            'obstruction_active': 0.0,
            'parking_complete': 0.0,
            'lane_error': 0.0,
            'cmd_vel': 0.0,
            'odom': 0.0,
            'odom_sim': 0.0,
            'joy': 0.0,
            'dashboard_state': 0.0,
            'dashboard_ctrl': 0.0,
            'set_challenge': 0.0,
            'health_status': 0.0,
            'cmd_safety_status': 0.0,
            'loop_stats': 0.0,
            'record_playback_state': 0.0,
            'parking_sign': 0.0,
            'imu_rpy': 0.0,
        }
        self.data_lock = threading.Lock()

        # Record & Playback command publisher
        self.rp_cmd_pub = self.create_publisher(String, RECORD_PLAYBACK_CMD_TOPIC, 10)
        self.challenge_pub = self.create_publisher(String, SET_CHALLENGE_TOPIC, 10)
        self.imu_cal_pub = self.create_publisher(String, IMU_CALIBRATE_TOPIC, 10)
        self.side_camera_request_pub = self.create_publisher(
            String, SIDE_CAMERA_REQUEST_TOPIC, 10)

        # State tracking
        self._state_entry_time = time.time()

        # === Subscriptions ===
        qos = QoSPresetProfiles.SENSOR_DATA.value

        self.create_subscription(Bool, AUTO_MODE_TOPIC, self._auto_mode_cb, 10)
        self.create_subscription(Bool, OBSTACLE_LIDAR_TOPIC, self._lidar_cb, qos)
        self.create_subscription(Bool, OBSTACLE_CAMERA_TOPIC, self._cam_cb, qos)
        self.create_subscription(Bool, OBSTACLE_FUSED_TOPIC, self._fused_cb, 10)
        self.create_subscription(String, TRAFFIC_LIGHT_TOPIC, self._tl_cb, 10)
        self.create_subscription(Bool, BOOM_GATE_TOPIC, self._gate_cb, 10)
        self.create_subscription(Bool, TUNNEL_DETECTED_TOPIC, self._tunnel_cb, 10)
        self.create_subscription(Bool, OBSTRUCTION_ACTIVE_TOPIC, self._obst_cb, 10)
        self.create_subscription(Bool, PARKING_COMPLETE_TOPIC, self._park_cb, 10)
        self.create_subscription(String, HEALTH_STATUS_TOPIC, self._health_cb, 10)
        self.create_subscription(String, CMD_SAFETY_STATUS_TOPIC, self._cmd_safety_cb, 10)
        self.create_subscription(String, LOOP_STATS_TOPIC, self._loop_stats_cb, 10)
        for component, topic in (
            ('bev', '/v4_experimental/bev/status'),
            ('road', '/v4_experimental/road/status'),
            ('pose', '/v4_experimental/pose/status'),
            ('uwb', '/v4_experimental/uwb/status'),
            ('trajectory', '/v4_experimental/trajectory/status'),
            ('parking', '/v4_experimental/parking/status'),
            ('recovery', '/v4_experimental/recovery/status'),
            ('arbitration', '/v4_experimental/arbitration/status'),
            ('control', '/v4_control/status'),
        ):
            self.create_subscription(
                String, topic,
                lambda msg, name=component: self._v4_status_cb(name, msg), 10)
        self.create_subscription(Float32, LANE_ERROR_TOPIC, self._lane_cb, qos)
        self.create_subscription(Twist, CMD_VEL_TOPIC, self._cmd_cb, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)
        self.create_subscription(Joy, JOY_TOPIC, self._joy_cb, 10)
        self.create_subscription(String, DASH_STATE_TOPIC, self._dash_state_cb, 10)
        self.create_subscription(String, DASH_CTRL_TOPIC, self._dash_ctrl_cb, 10)
        self.create_subscription(String, SET_CHALLENGE_TOPIC, self._set_challenge_cb, 10)
        # Also listen to auto commands for display
        self.create_subscription(Twist, AUTO_CMD_VEL_TOPIC, self._cmd_cb, 10)

        # Camera subscriptions (SENSOR_DATA QoS to match camera publisher)
        self.create_subscription(Image, CAMERA_IMAGE_TOPIC, lambda msg: self._image_cb(msg, 'raw'), qos)
        self.create_subscription(Image, CAMERA_DEBUG_LINE_TOPIC, lambda msg: self._image_cb(msg, 'line_follower'), qos)
        # Single subscription covers both 'signage' and 'traffic_light' dashboard views
        self.create_subscription(Image, SIGNAGE_DEBUG_TOPIC, lambda msg: self._image_cb(msg, 'signage'), qos)
        self.create_subscription(Image, CAMERA_DEBUG_OBS_TOPIC, lambda msg: self._image_cb(msg, 'obstacle'), qos)
        self.create_subscription(Image, MIPI_SECONDARY_TOPIC, lambda msg: self._image_cb(msg, 'second'), qos)
        self.create_subscription(Image, MIPI_TERTIARY_TOPIC, lambda msg: self._image_cb(msg, 'third'), qos)

        # V4 composite view. Frames are grouped by their ROS source stamp in
        # _v4_comp_cb; never combine the latest frame from unrelated cycles.
        self.create_subscription(Image, '/v4_experimental/bev/primary/image', lambda msg: self._v4_comp_cb(msg, 'bev'), qos)
        self.create_subscription(Image, '/v4_experimental/bev/primary/coverage', lambda msg: self._v4_comp_cb(msg, 'coverage'), qos)
        self.create_subscription(Image, '/v4_experimental/road/primary/candidate', lambda msg: self._v4_comp_cb(msg, 'candidate'), qos)
        self.create_subscription(Image, '/v4_experimental/road/primary/connected', lambda msg: self._v4_comp_cb(msg, 'connected'), qos)
        self.create_subscription(Image, '/v4_experimental/road/primary/fused', lambda msg: self._v4_comp_cb(msg, 'fused'), qos)

        # Parking signboard detection flag
        self.create_subscription(Bool, PARKING_SIGN_TOPIC, self._parking_sign_cb, 10)

        # IMU full RPY data
        self.create_subscription(String, IMU_DATA_TOPIC, self._imu_cb, 10)

        # LiDAR scan for 2D visualization
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)

        # Tunnel debug for LiDAR overlay
        self.create_subscription(String, '/tunnel_debug', self._tunnel_debug_cb, 10)

        # Record & Playback state from servo_controller
        self.create_subscription(String, RECORD_PLAYBACK_STATE_TOPIC, self._rp_state_cb, 10)

        # V4 telemetry bridge output for the track map (display only)
        self.create_subscription(String, '/v4_telemetry', self._v4telemetry_cb, 10)

        # Simulate odometry since hardware might not publish
        self.create_timer(0.05, self._simulate_odom_loop)
        # Renewable lease: the root manager shuts the optional MIPI pipelines
        # down if the dashboard disappears or no side-view client remains.
        self.create_timer(2.0, self._side_camera_lease_loop)
        # Only performs work while a client is watching V4 Road. It replaces a
        # stopped stream with an explicit stale card instead of freezing old data.
        self.create_timer(0.25, self._v4_comp_watchdog)

        self.get_logger().info('Dashboard subscriptions ready')

    def _side_camera_lease_loop(self) -> None:
        with self.camera_clients_lock:
            has_viewer = self.num_camera_clients > 0
        # Silence is how an idle requester releases its lease.  Publishing
        # repeated "off" messages here would fight independent requesters
        # such as the V4 parking/BEV pipeline.
        if not has_viewer:
            return
        source = self.active_camera_source
        mode = 'right' if source == 'second' else 'left' if source == 'third' else 'off'
        self.side_camera_request_pub.publish(String(data=mode))

    def _simulate_odom_loop(self) -> None:
        """Simulates odometry position based on commanded velocities.
        Useful when the hardware driver fails to publish /odom data."""
        use_hw_odom = self.get_parameter('use_hw_odom').value
        if use_hw_odom:
            return # Let the hardware Odometry cb update the state completely

        now = time.time()
        if not hasattr(self, '_last_sim_t'):
            self._last_sim_t = now
            return
            
        dt = min(now - self._last_sim_t, 0.2)
        self._last_sim_t = now
        
        with self.data_lock:
            # If no cmd_vel received in last 0.5s, assume robot is stopped
            cmd_age = now - self.data.get('_cmd_time', 0)
            if cmd_age > 0.5:
                vel_x = 0.0
                vel_z = 0.0
            else:
                vel_x = self.data['cmd_lin_x']
                vel_z = self.data['cmd_ang_z']
            
            # Odometry calibration: measured 1m real → scale to match
            # Residual error is from wheel slip/coasting (robot moves after cmd_vel=0)
            odom_scale = float(self.get_parameter('sim_odom_scale').value)
            cal_vel_x = vel_x * odom_scale
            
            # Minimum velocity threshold: ignore tiny commanded speeds
            # (prevents odometer drift when motor can't actually move)
            if abs(cal_vel_x) < 0.03:
                cal_vel_x = 0.0
            
            # Distance integration
            self.data['speed'] = cal_vel_x
            self.data['distance'] += abs(cal_vel_x) * dt
            
            # Position integration (dead reckoning)
            self.data['odom_yaw'] += vel_z * dt
            
            # X and Y based on current heading
            self.data['odom_x'] += cal_vel_x * math.cos(self.data['odom_yaw']) * dt
            self.data['odom_y'] += cal_vel_x * math.sin(self.data['odom_yaw']) * dt
            self.topic_last_update['odom_sim'] = time.monotonic()

    def _set(self, key, value, source_key=None) -> None:
        with self.data_lock:
            self.data[key] = value
            if source_key:
                self.topic_last_update[source_key] = time.monotonic()

    def _auto_mode_cb(self, msg: Bool) -> None:
        current = bool(msg.data)
        with self.data_lock:
            previous = bool(self.data.get('auto_mode', False))
            self.data['auto_mode'] = current
            # The publisher uses a heartbeat, so refresh health on every message
            # even when the mode value itself is unchanged.
            self.topic_last_update['auto_mode'] = time.monotonic()
        if current == previous:
            return
        mode = "AUTO" if current else "MANUAL"
        self.get_logger().info(f'Mode changed: {mode}')

    def _lidar_cb(self, msg: Bool) -> None:
        """Update LiDAR obstacle flag."""
        self._set('lidar_obstacle', msg.data, 'obstacle_front')

    def _cam_cb(self, msg: Bool) -> None:
        """Update camera obstacle flag."""
        self._set('camera_obstacle', msg.data, 'obstacle_camera')

    def _fused_cb(self, msg: Bool) -> None:
        """Update fused obstacle flag."""
        self._set('fused_obstacle', msg.data, 'obstacle_fused')

    def _tl_cb(self, msg: String) -> None:
        """Update traffic light state."""
        self._set('traffic_light', msg.data, 'traffic_light')

    def _gate_cb(self, msg: Bool) -> None:
        """Update boom gate state."""
        self._set('boom_gate', msg.data, 'boom_gate')

    def _tunnel_cb(self, msg: Bool) -> None:
        """Update tunnel detection flag."""
        self._set('tunnel_detected', msg.data, 'tunnel_detected')

    def _obst_cb(self, msg: Bool) -> None:
        """Update obstruction avoidance flag."""
        self._set('obstruction_active', msg.data, 'obstruction_active')

    def _park_cb(self, msg: Bool) -> None:
        """Update parking completion flag."""
        self._set('parking_complete', msg.data, 'parking_complete')

    def _lane_cb(self, msg: Float32) -> None:
        """Update lane error."""
        self._set('lane_error', msg.data, 'lane_error')

    def _health_cb(self, msg: String) -> None:
        """Update parsed health summary from /health_status."""
        try:
            payload = json.loads(msg.data)
            with self.data_lock:
                self.data['health_ok'] = bool(payload.get('ok', False))
                self.data['health_summary'] = str(payload.get('summary', ''))
                self.data['health_stale'] = list(payload.get('stale', []))
                self.topic_last_update['health_status'] = time.monotonic()
        except Exception:
            with self.data_lock:
                self.data['health_ok'] = False
                self.data['health_summary'] = msg.data
                self.topic_last_update['health_status'] = time.monotonic()

    def _cmd_safety_cb(self, msg: String) -> None:
        """Update command safety status fields."""
        try:
            payload = json.loads(msg.data)
            with self.data_lock:
                self.data['cmd_safety_estop'] = bool(payload.get('estop', False))
                self.data['cmd_safety_timeout_count'] = int(payload.get('timeout_count', 0))
                self.data['cmd_safety_autonomy_source'] = str(
                    payload.get('autonomy_source', 'unknown'))
                self.topic_last_update['cmd_safety_status'] = time.monotonic()
        except Exception:
            self._set('cmd_safety_estop', False, 'cmd_safety_status')

    def _v4_status_cb(self, component: str, msg: String) -> None:
        """Keep the latest status from each V4 stage for the dashboard only."""
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('V4 status must be an object')
            item = dict(payload)
            item['_received_mono'] = time.monotonic()
            with self.data_lock:
                statuses = dict(self.data.get('v4_status', {}))
                statuses[component] = item
                self.data['v4_status'] = statuses
        except (json.JSONDecodeError, TypeError, ValueError):
            return

    def _v4telemetry_cb(self, msg: String) -> None:
        """Cache the latest V4 telemetry document for the track map."""
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('telemetry must be an object')
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        self._v4telemetry = payload
        self._v4telemetry_mono = time.monotonic()

    def _loop_stats_cb(self, msg: String) -> None:
        """Track latest loop stat payloads by node:loop key."""
        try:
            payload = json.loads(msg.data)
            node = str(payload.get('node', 'unknown'))
            loop = str(payload.get('loop', 'unknown'))
            key = f'{node}:{loop}'
            with self.data_lock:
                stats = dict(self.data.get('loop_stats', {}))
                stats[key] = payload
                self.data['loop_stats'] = stats
                self.topic_last_update['loop_stats'] = time.monotonic()
        except Exception:
            pass

    def _cmd_cb(self, msg: Twist) -> None:
        with self.data_lock:
            self.data['cmd_lin_x'] = msg.linear.x
            self.data['cmd_ang_z'] = msg.angular.z
            self.data['_cmd_time'] = time.time()
            self.topic_last_update['cmd_vel'] = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        use_hw_odom = self.get_parameter('use_hw_odom').value
        if not use_hw_odom:
            return  # Ignore real hardware if simulation failsafe is active
            
        # We receive actual odometry! Use real data instead of dead reckoning.
        with self.data_lock:
            # Natively, many basic firmwares only populate Twist (speeds) and leave Pose (X, Y, Yaw) as exactly 0.0
            vel_x = msg.twist.twist.linear.x
            hw_scale = float(self.get_parameter('hw_odom_scale').value)
            yaw_scale = float(self.get_parameter('hw_odom_yaw_scale').value)
            vel_x_scaled = vel_x * hw_scale
            self.data['speed'] = vel_x_scaled
            
            now = time.time()
            if hasattr(self, '_last_real_odom_t'):
                dt = min(now - self._last_real_odom_t, 0.2)
                self.data['distance'] += abs(vel_x_scaled) * dt
            else:
                dt = 0.0
            self._last_real_odom_t = now

            # Check if Firmware is actually publishing Pose Quaternions
            q = msg.pose.pose.orientation
            if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
                # Dead reckon yaw from twist angular Z
                self.data['odom_yaw'] += msg.twist.twist.angular.z * yaw_scale * dt
            else:
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                self.data['odom_yaw'] = math.atan2(siny_cosp, cosy_cosp)
            
            # Check if Firmware is actually publishing Pose X/Y
            hw_x = msg.pose.pose.position.x
            hw_y = msg.pose.pose.position.y
            
            if abs(hw_x) < 0.0001 and abs(hw_y) < 0.0001 and self.data['distance'] > 0.0:
                # Firmware omitted Pose but shipped Speeds. Dead reckon the speeds using current Yaw!
                self.data['odom_x'] += vel_x_scaled * math.cos(self.data['odom_yaw']) * dt
                self.data['odom_y'] += vel_x_scaled * math.sin(self.data['odom_yaw']) * dt
            else:
                self.data['odom_x'] = hw_x
                self.data['odom_y'] = hw_y
            self.topic_last_update['odom'] = time.monotonic()

    def _joy_cb(self, msg: Joy) -> None:
        with self.data_lock:
            self.data['buttons'] = list(msg.buttons)
            if self.initial_joy_axes is None:
                self.initial_joy_axes = list(msg.axes)
            
            if self.initial_joy_axes and list(msg.axes) == self.initial_joy_axes:
                self.data['axes'] = [0.0] * len(msg.axes)
            else:
                self.initial_joy_axes = []  # Clear forever once moved
                self.data['axes'] = list(msg.axes)
            self.topic_last_update['joy'] = time.monotonic()

    def _dash_state_cb(self, msg: String) -> None:
        """Parse STATE|LAP|TOTAL_DIST|STOP_REASON format from auto_driver."""
        try:
            parts = msg.data.split('|')
            with self.data_lock:
                if parts[0] != self.data.get('state'):
                    self._state_entry_time = time.time()  # Reset timer on state change
                self.data['state'] = parts[0]
                if len(parts) > 1:
                    self.data['lap'] = int(parts[1])
                if len(parts) > 2:
                    self.data['state_dist'] = parts[2]  # Now represents total distance
                if len(parts) > 3:
                    self.data['stop_reason'] = parts[3]
                else:
                    self.data['stop_reason'] = ''
                self.topic_last_update['dashboard_state'] = time.monotonic()
        except Exception:
            self._set('state', msg.data)

    def _dash_ctrl_cb(self, msg: String) -> None:
        """Parse SPD_PCT|STATE_INDEX|STATE_NAME format from servo_controller."""
        try:
            parts = msg.data.split('|')
            with self.data_lock:
                self.data['speed_pct'] = int(parts[0])
                if len(parts) > 1:
                    self.data['ctrl_state_index'] = int(parts[1])
                if len(parts) > 2:
                    self.data['ctrl_state_name'] = parts[2]
                self.topic_last_update['dashboard_ctrl'] = time.monotonic()
        except Exception:
            pass

    def _set_challenge_cb(self, msg: String) -> None:
        """Also listen to /set_challenge as fallback for state cycle display."""
        name = msg.data.upper()
        self._set('ctrl_state_name', name, 'set_challenge')

    def _scan_cb(self, msg: LaserScan) -> None:
        """Convert LiDAR scan to Cartesian points for 2D visualization.
        Downsamples to every 4th point to keep bandwidth low."""
        import math
        pts = []
        offset = self.lidar_angle_offset
        step = 4  # downsample: take every 4th point
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if not (msg.range_min <= r <= msg.range_max):
                continue
            if math.isnan(r) or math.isinf(r) or r > 2.0:
                continue
            angle = msg.angle_min + i * msg.angle_increment + offset
            x = round(r * math.cos(angle), 3)
            y = round(r * math.sin(angle), 3)
            pts.append({'x': x, 'y': y})
        with self.lidar_lock:
            self.lidar_points = pts

    def _tunnel_debug_cb(self, msg: String) -> None:
        """Store tunnel debug info for dashboard overlay."""
        with self.tunnel_debug_lock:
            self.tunnel_debug = msg.data

    def _rp_state_cb(self, msg: String) -> None:
        """Update record/playback state from servo_controller."""
        try:
            payload = json.loads(msg.data)
            with self.data_lock:
                self.data['rp_state'] = str(payload.get('state', 'IDLE'))
                self.data['rp_buffer_size'] = int(payload.get('buffer_size', 0))
                self.data['rp_playback_index'] = int(payload.get('playback_index', 0))
                self.data['rp_recording_name'] = str(payload.get('recording_name', ''))
                self.data['rp_active_parking'] = str(payload.get('active_parking_recording', ''))
                self.data['rp_saved_recordings'] = list(payload.get('saved_recordings', []))
                self.topic_last_update['record_playback_state'] = time.monotonic()
        except Exception:
            pass

    def _parking_sign_cb(self, msg: Bool) -> None:
        """Update parking signboard detection flag."""
        self._set('parking_sign_detected', msg.data, 'parking_sign')

    def _imu_cb(self, msg: String) -> None:
        """Update IMU roll/pitch/yaw from /imu/rpy JSON payload."""
        try:
            payload = json.loads(msg.data)
            with self.data_lock:
                self.data['imu_roll']  = round(float(payload.get('roll',  0.0)), 2)
                self.data['imu_pitch'] = round(float(payload.get('pitch', 0.0)), 2)
                self.data['imu_yaw']   = round(float(payload.get('yaw',   0.0)), 2)
                self.topic_last_update['imu_rpy'] = time.monotonic()
        except Exception:
            pass

    def _init_v4_composite_state(self) -> None:
        """Initialize bounded, timestamp-keyed state for the V4 Road view."""
        self._v4_lock = threading.Lock()
        self._v4_sets = {}
        self._v4_last_stamp_by_name = {}
        self._v4_render_token = None
        self._v4_diag_last = {}
        self._v4_sync_wait_sec = _V4_COMPOSITE_SYNC_WAIT_SEC
        self._v4_stale_sec = _V4_COMPOSITE_STALE_SEC

    def _reset_v4_composite(self) -> None:
        """Drop cached V4 frames after a camera source or view change."""
        with self._v4_lock:
            self._v4_sets.clear()
            self._v4_last_stamp_by_name.clear()
            self._v4_render_token = None
        self._last_camera_source_mono = 0.0

    def _warn_rate_limited(self, key: str, message: str) -> None:
        """Emit useful camera diagnostics without flooding the ROS log."""
        now_mono = time.monotonic()
        last = self._v4_diag_last.get(key)
        if (last is not None
                and now_mono - last < _DASHBOARD_DIAGNOSTIC_INTERVAL_SEC):
            return
        self._v4_diag_last[key] = now_mono
        self.get_logger().warning(message)

    @staticmethod
    def _v4_stamp_ns(msg: Image):
        """Return a valid ROS source stamp as integer nanoseconds."""
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        try:
            sec = int(stamp.sec)
            nanosec = int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return None
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            return None
        total = sec * 1_000_000_000 + nanosec
        return total if total > 0 else None

    @staticmethod
    def _v4_tile(image, label: str, status_lines=(), status_color=(80, 80, 220)):
        """Letterbox one image into a labeled tile without changing its aspect."""
        tile_w, tile_h = _V4_COMPOSITE_TILE_SIZE
        header_h = 32
        tile = np.full((tile_h, tile_w, 3), 12, dtype=np.uint8)
        if image is not None and len(image.shape) >= 2:
            image_h, image_w = image.shape[:2]
            if image_h > 0 and image_w > 0:
                available_h = tile_h - header_h
                scale = min(tile_w / image_w, available_h / image_h)
                render_w = max(1, int(round(image_w * scale)))
                render_h = max(1, int(round(image_h * scale)))
                interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
                fitted = cv2.resize(image, (render_w, render_h), interpolation=interpolation)
                x0 = (tile_w - render_w) // 2
                y0 = header_h + (available_h - render_h) // 2
                tile[y0:y0 + render_h, x0:x0 + render_w] = fitted

        cv2.rectangle(tile, (0, 0), (tile_w, header_h), (28, 31, 38), -1)
        cv2.putText(
            tile, label, (10, 23), cv2.FONT_HERSHEY_SIMPLEX,
            0.62, (245, 245, 245), 2, cv2.LINE_AA,
        )
        if status_lines:
            lines = list(status_lines)
            first_y = 112 - (len(lines) - 1) * 16
            for index, line in enumerate(lines):
                text = str(line)
                size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 2)[0]
                x = max(8, (tile_w - size[0]) // 2)
                cv2.putText(
                    tile, text, (x, first_y + index * 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, status_color, 2,
                    cv2.LINE_AA,
                )
        return tile

    def _v4_snapshot(self, now_mono: float):
        """Take a coherent newest-stamp snapshot, or defer during sync grace."""
        with self._v4_lock:
            if not self._v4_sets:
                token = ('waiting',)
                if token == self._v4_render_token:
                    return None
                return {
                    'token': token, 'mode': 'waiting', 'stamp_ns': None,
                    'age': None, 'tiles': {}, 'last_stamps': {},
                    'source_mono': 0.0,
                }

            stamp_ns = max(self._v4_sets)
            group = self._v4_sets[stamp_ns]
            age = max(0.0, now_mono - group['first_received'])
            tiles = dict(group['tiles'])
            present = set(tiles)
            if age > self._v4_stale_sec:
                mode = 'stale'
                tiles = {}
            elif present == _V4_COMPOSITE_NAMES:
                mode = 'fresh'
            elif age >= self._v4_sync_wait_sec:
                mode = 'partial'
            else:
                return None
            token = (stamp_ns, mode, tuple(sorted(tiles)))
            if token == self._v4_render_token:
                return None
            return {
                'token': token, 'mode': mode, 'stamp_ns': stamp_ns,
                'age': age, 'tiles': tiles,
                'last_stamps': dict(self._v4_last_stamp_by_name),
                'source_mono': group['first_received'],
            }

    def _v4_composite_image(self, snapshot):
        """Build a five-stage V4 strip, replacing absent/stale images with cards."""
        mode = snapshot['mode']
        stamp_ns = snapshot['stamp_ns']
        tiles = snapshot['tiles']
        rendered = []
        missing = []
        for name, label in _V4_COMPOSITE_TILES:
            if name in tiles:
                rendered.append(tiles[name])
                continue
            missing.append(name)
            if mode == 'stale':
                status = ('STALE', f"{snapshot['age']:.1f} s old")
            elif mode == 'waiting':
                status = ('NO INPUT', 'waiting for frame')
            else:
                previous = snapshot['last_stamps'].get(name)
                if previous is None:
                    detail = 'no frame received'
                else:
                    behind = max(0.0, (stamp_ns - previous) * 1e-9)
                    detail = f'last {behind:.3f} s behind'
                status = ('MISSING', detail)
            rendered.append(self._v4_tile(None, label, status))

        if mode == 'fresh':
            state_text = 'SYNCHRONIZED  |  5 / 5 stages'
            state_color = (90, 210, 120)
        elif mode == 'partial':
            state_text = (
                f'INCOMPLETE FRAME  |  {len(tiles)} / 5 stages  |  '
                'missing: ' + ', '.join(missing)
            )
            state_color = (30, 190, 245)
        elif mode == 'stale':
            state_text = f"STALE INPUT  |  {snapshot['age']:.1f} s old"
            state_color = (80, 80, 230)
        else:
            state_text = 'WAITING FOR V4  |  no source-timestamped input'
            state_color = (160, 160, 160)
        if stamp_ns is not None:
            state_text += f'  |  source t={stamp_ns * 1e-9:.3f}'

        # The reference view is a left-to-right pipeline. Keep status outside
        # the five image stages so no diagnostic card can be mistaken for a
        # sixth pipeline output. The footer remains readable when the wide
        # strip is scaled down in the responsive camera card.
        strip = cv2.hconcat(rendered)
        footer = np.full(
            (_V4_COMPOSITE_FOOTER_HEIGHT, strip.shape[1], 3),
            (28, 31, 38), dtype=np.uint8,
        )
        cv2.putText(
            footer, state_text, (10, 23), cv2.FONT_HERSHEY_SIMPLEX,
            0.62, state_color, 2, cv2.LINE_AA,
        )
        return cv2.vconcat((strip, footer)), missing

    def _maybe_publish_v4_composite(self, now_mono: float) -> None:
        """Encode at most one bounded composite when the active viewer needs it."""
        generation = self._camera_selection_generation
        if self.active_camera_view != 'road' or self.active_camera_source != 'forward':
            return
        with self.camera_clients_lock:
            if self.num_camera_clients == 0:
                return
        if now_mono - self._last_encode_mono < self._encode_min_interval:
            return
        snapshot = self._v4_snapshot(now_mono)
        if snapshot is None:
            return
        try:
            composite, missing = self._v4_composite_image(snapshot)
            ok, jpeg = cv2.imencode(
                '.jpg', composite, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if not ok:
                raise ValueError('cv2.imencode returned false')
        except (cv2.error, TypeError, ValueError) as exc:
            self._warn_rate_limited(
                'v4_composite_encode', f'V4 Road composite encode failed: {exc}')
            return

        if (generation != self._camera_selection_generation
                or self.active_camera_view != 'road'
                or self.active_camera_source != 'forward'):
            return

        with self._v4_lock:
            self._v4_render_token = snapshot['token']
        self._last_encode_mono = now_mono
        if snapshot['source_mono'] > 0.0:
            self._last_camera_source_mono = snapshot['source_mono']
        with self.jpeg_condition:
            self.latest_jpeg = jpeg.tobytes()
            self.frame_id += 1
            self.jpeg_condition.notify_all()

        if snapshot['mode'] == 'partial':
            self._warn_rate_limited(
                'v4_composite_missing',
                'V4 Road composite is missing timestamp-matched tiles: '
                + ', '.join(missing),
            )
        elif snapshot['mode'] == 'stale':
            self._warn_rate_limited(
                'v4_composite_stale',
                f"V4 Road composite input is stale ({snapshot['age']:.1f} s)",
            )
        elif snapshot['mode'] == 'waiting':
            self._warn_rate_limited(
                'v4_composite_waiting',
                'V4 Road view has no valid source-timestamped input',
            )

    def _v4_comp_watchdog(self) -> None:
        """Expire a stopped V4 stream while a Road-view client is connected."""
        self._maybe_publish_v4_composite(time.monotonic())

    def _image_cb(self, msg: Image, view_name: str) -> None:
        """Convert ROS Image to JPEG conditionally, tracking active view and clients."""
        active = self.active_camera_view
        source = self.active_camera_source
        generation = self._camera_selection_generation
        if self.bridge is None:
            return
        if view_name in ('second', 'third'):
            # Side cameras are raw-only: shown only when selected with raw view.
            if source != view_name or active != 'raw':
                return
        elif source != 'forward':
            return
        # 'signage' topic covers both the 'signage' and 'traffic_light' dashboard views
        elif view_name != active and not (view_name == 'signage' and active == 'traffic_light'):
            return
            
        with self.camera_clients_lock:
            if self.num_camera_clients == 0:
                return  # Skip processing entirely if nobody is watching

        now_mono = time.monotonic()
        if now_mono - self._last_encode_mono < self._encode_min_interval:
            return  # throttle MJPEG encode (dashboard only needs ~10Hz)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Force to fixed 320x240 — prevents visual jumping when source sends varying sizes
            ih, iw = cv_image.shape[:2]
            if iw != 320 or ih != 240:
                cv_image = cv2.resize(cv_image, (320, 240))
            ok, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if not ok:
                raise ValueError('cv2.imencode returned false')
            if (generation != self._camera_selection_generation
                    or active != self.active_camera_view
                    or source != self.active_camera_source):
                return
            self._last_encode_mono = now_mono
            self._last_camera_source_mono = now_mono
            with self.jpeg_condition:
                self.latest_jpeg = jpeg.tobytes()
                self.frame_id += 1
                self.jpeg_condition.notify_all()
        except (cv2.error, CvBridgeError, TypeError, ValueError) as exc:
            self._warn_rate_limited(
                f'camera_encode_{view_name}',
                f'Dashboard camera encode failed for {view_name}: {exc}',
            )

    def _v4_comp_cb(self, msg: Image, name: str) -> None:
        if name not in _V4_COMPOSITE_NAMES:
            self._warn_rate_limited(
                'v4_composite_unknown_tile',
                f'Ignoring unknown V4 Road tile {name!r}',
            )
            return
        if (self.active_camera_view != 'road'
                or self.active_camera_source != 'forward'
                or self.bridge is None):
            return
        generation = self._camera_selection_generation
        with self.camera_clients_lock:
            if self.num_camera_clients == 0:
                return
        stamp_ns = self._v4_stamp_ns(msg)
        if stamp_ns is None:
            self._warn_rate_limited(
                f'v4_composite_stamp_{name}',
                f'Ignoring V4 Road {name} tile without a valid source timestamp',
            )
            self._maybe_publish_v4_composite(time.monotonic())
            return
        now_mono = time.monotonic()
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            label = _V4_COMPOSITE_LABELS[name]
            tile = self._v4_tile(cv_img, label)
        except (cv2.error, CvBridgeError, TypeError, ValueError) as exc:
            self._warn_rate_limited(
                f'v4_composite_convert_{name}',
                f'V4 Road {name} conversion failed: {exc}',
            )
            return

        if (generation != self._camera_selection_generation
                or self.active_camera_view != 'road'
                or self.active_camera_source != 'forward'):
            return

        with self._v4_lock:
            if self._v4_sets:
                newest_stamp = max(self._v4_sets)
                if stamp_ns < newest_stamp:
                    self._warn_rate_limited(
                        f'v4_composite_out_of_order_{name}',
                        f'Ignoring out-of-order V4 Road {name} tile '
                        f'({stamp_ns} < {newest_stamp})',
                    )
                    return
            group = self._v4_sets.setdefault(
                stamp_ns,
                {'first_received': now_mono, 'tiles': {}},
            )
            group['tiles'][name] = tile
            self._v4_last_stamp_by_name[name] = stamp_ns
            for old_stamp in sorted(self._v4_sets)[:-_V4_COMPOSITE_MAX_STAMP_SETS]:
                del self._v4_sets[old_stamp]

        self._maybe_publish_v4_composite(now_mono)

    def get_json(self) -> str:
        with self.data_lock:
            d = dict(self.data)
            updates = dict(self.topic_last_update)
        d['state_time'] = int(time.time() - self._state_entry_time)
        now_mono = time.monotonic()
        last_source = getattr(self, '_last_camera_source_mono', self._last_encode_mono)
        d['cam_age_ms'] = int((now_mono - last_source) * 1000) if last_source > 0.0 else None
        stale_sec = float(self.get_parameter('freshness_stale_sec').value)
        freshness = {}
        stale_streams = []
        event_driven_topics = {'boom_gate', 'parking_complete', 'auto_mode', 'set_challenge', 'record_playback_state', 'joy', 'obstacle_fused'}
        inactive_odom = 'odom_sim' if self.get_parameter('use_hw_odom').value else 'odom'
        for key, last_t in updates.items():
            if key == inactive_odom:
                continue
            if last_t <= 0.0:
                freshness[key] = None
                if key not in event_driven_topics:
                    stale_streams.append(key)
                continue
            age = round(now_mono - last_t, 3)
            freshness[key] = age
            if age > stale_sec and key not in event_driven_topics:
                stale_streams.append(key)
        d['freshness_sec'] = freshness
        d['stale_streams'] = stale_streams

        # Convert internal receive timestamps to stable, browser-friendly ages.
        v4_status = {}
        for component, raw in d.get('v4_status', {}).items():
            item = dict(raw)
            received = item.pop('_received_mono', None)
            item['age_sec'] = (None if received is None
                               else round(max(0.0, now_mono - received), 3))
            v4_status[component] = item
        d['v4_status'] = v4_status
        
        # Ensure odometry types are standard python floats for JSON serialization
        for k in ['distance', 'speed', 'odom_x', 'odom_y', 'odom_yaw']:
            if k in d and d[k] is not None:
                d[k] = float(d[k])
                
        return json.dumps(d)

    def get_jpeg(self) -> None:
        # We no longer use this standalone method because do_GET
        # manages the condition variable directly to block until new frame.
        pass


# ======================== HTTP Server ========================

# Cached service clients for parameter operations
_param_clients_lock = threading.Lock()
_param_clients = {}  # {'/node_name': {'get': client, 'set': client}}

# Dedicated lightweight node + executor for param Get/Set.
# Isolated from the main dashboard node so camera callbacks never block it.
_param_helper_node = None
_param_executor = None

def _get_client(node_name, svc_type):
    """Get or create a cached service client on the dedicated param helper node."""
    n = node_name if node_name.startswith('/') else '/' + node_name
    key = (n, svc_type)
    with _param_clients_lock:
        if key not in _param_clients:
            if _param_helper_node is None:
                return None
            if svc_type == 'get':
                _param_clients[key] = _param_helper_node.create_client(GetParameters, n + '/get_parameters')
            else:
                _param_clients[key] = _param_helper_node.create_client(SetParameters, n + '/set_parameters')
    return _param_clients[key]

def _ros_get_param(node_name, param_name):
    """Get a parameter via native rclpy service (no subprocess)."""
    try:
        client = _get_client(node_name, 'get')
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.15):
                return None, f'Node /{node_name} not available'
        req = GetParameters.Request()
        req.names = [param_name]
        future = client.call_async(req)
        deadline = time.time() + 2.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return None, 'Timeout'
        res = future.result()
        if res and res.values:
            v = res.values[0]
            if v.type == ParameterType.PARAMETER_BOOL:
                return str(v.bool_value).lower(), None
            elif v.type == ParameterType.PARAMETER_INTEGER:
                return str(v.integer_value), None
            elif v.type == ParameterType.PARAMETER_DOUBLE:
                return str(v.double_value), None
            elif v.type == ParameterType.PARAMETER_STRING:
                return v.string_value, None
            elif v.type == ParameterType.PARAMETER_BYTE_ARRAY:
                return list(v.byte_array_value), None
            elif v.type == ParameterType.PARAMETER_BOOL_ARRAY:
                return list(v.bool_array_value), None
            elif v.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                return list(v.integer_array_value), None
            elif v.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                return list(v.double_array_value), None
            elif v.type == ParameterType.PARAMETER_STRING_ARRAY:
                return list(v.string_array_value), None
            elif v.type == ParameterType.PARAMETER_NOT_SET:
                return None, 'Not set'
            else:
                return '?', None
        return None, 'No result'
    except Exception as e:
        return None, str(e)

def _ros_set_param(node_name, param_name, value_str):
    """Set a parameter via native rclpy service (no subprocess)."""
    try:
        client = _get_client(node_name, 'set')
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.15):
                return False, f'Node /{node_name} not available'
        param = RosParameter()
        param.name = param_name
        pv = ParameterValue()
        
        # Read the declared type instead of guessing from decimal punctuation.
        get_client = _get_client(node_name, 'get')
        if not get_client.wait_for_service(timeout_sec=0.15):
            return False, 'Parameter service unavailable'
        get_req = GetParameters.Request()
        get_req.names = [param_name]
        get_future = get_client.call_async(get_req)
        deadline = time.monotonic() + 2.0
        while not get_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not get_future.done():
            return False, 'Parameter type lookup timed out'
        current = get_future.result()
        if not current or not current.values:
            return False, 'Parameter not declared'
        pv.type = current.values[0].type
        value = coerce_parameter_value(value_str, pv.type)
        fields = {1:'bool_value', 2:'integer_value', 3:'double_value', 4:'string_value',
                  5:'byte_array_value', 6:'bool_array_value', 7:'integer_array_value',
                  8:'double_array_value', 9:'string_array_value'}
        setattr(pv, fields[pv.type], value)
        param.value = pv
        req = SetParameters.Request()
        req.parameters = [param]
        future = client.call_async(req)
        deadline = time.time() + 2.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, 'Timeout'
        res = future.result()
        if res and res.results:
            r = res.results[0]
            if r.successful:
                return True, 'OK'
            else:
                return False, r.reason or 'Failed'
        return False, 'No result'
    except Exception as e:
        return False, str(e)


def _save_params_to_yaml():
    """Persist live legacy and V4 parameters to their source YAML files."""
    global _PARAMS_SOURCE_PATHS, _DEFAULT_PARAMS
    paths = [path for path in _PARAMS_SOURCE_PATHS if os.path.exists(path)]
    if not paths:
        return {'ok': False, 'error': 'No parameter YAML paths resolved'}

    def yaml_scalar(value):
        rendered = yaml.safe_dump(
            value, default_flow_style=True, allow_unicode=True, width=120,
        ).strip()
        if rendered.endswith('\n...'):
            rendered = rendered[:-4].rstrip()
        return rendered

    def replace_values(raw_text, updates):
        current_node = None
        result = []
        node_pattern = re.compile(r'^([A-Za-z0-9_]+):\s*(?:#.*)?$')
        param_pattern = re.compile(
            r'^(\s{4})([A-Za-z0-9_]+):(\s*)(.*?)(\s+#.*)?$'
        )
        for line in raw_text.splitlines(keepends=True):
            newline = '\r\n' if line.endswith('\r\n') else ('\n' if line.endswith('\n') else '')
            body = line[:-len(newline)] if newline else line
            node_match = node_pattern.match(body)
            if node_match:
                current_node = node_match.group(1)
            param_match = param_pattern.match(body)
            key = None if not param_match else (current_node, param_match.group(2))
            if key in updates:
                comment = param_match.group(5) or ''
                body = (
                    f'{param_match.group(1)}{param_match.group(2)}:'
                    f'{param_match.group(3)}{yaml_scalar(updates[key])}{comment}'
                )
            result.append(body + newline)
        return ''.join(result)

    try:
        updated_count = 0
        saved_paths = []
        for path in paths:
            with open(path, 'r', encoding='utf-8', newline='') as handle:
                raw_text = handle.read()
            yaml_data = yaml.safe_load(raw_text)
            if not yaml_data:
                continue
            file_updates = {}
            for node_name, node_data in yaml_data.items():
                if not isinstance(node_data, dict) or 'ros__parameters' not in node_data:
                    continue
                params = node_data['ros__parameters']
                for param_name in list(params.keys()):
                    safe_v4 = _V4_DASHBOARD_SAFE_PARAMS.get(node_name)
                    if node_name.startswith('v4_') and (
                        safe_v4 is None or param_name not in safe_v4
                    ):
                        continue
                    value, err = _ros_get_param(node_name, param_name)
                    if err is not None:
                        continue
                    old_val = params[param_name]
                    try:
                        if isinstance(old_val, bool):
                            new_val = value.lower() == 'true'
                        elif isinstance(old_val, int):
                            new_val = int(float(value))
                        elif isinstance(old_val, float):
                            new_val = float(value)
                        elif isinstance(old_val, list):
                            parsed = yaml.safe_load(value)
                            new_val = parsed if isinstance(parsed, list) else old_val
                        else:
                            new_val = value
                    except (ValueError, TypeError, yaml.YAMLError):
                        continue
                    if old_val != new_val:
                        params[param_name] = new_val
                        file_updates[(node_name, param_name)] = new_val
                        updated_count += 1
                _DEFAULT_PARAMS.setdefault(node_name, {}).update(params)
            if file_updates:
                updated_text = replace_values(raw_text, file_updates)
                with open(path, 'w', encoding='utf-8', newline='') as handle:
                    handle.write(updated_text)
            saved_paths.append(path)

        return {
            'ok': True,
            'msg': f'Saved {updated_count} changed params across {len(saved_paths)} YAML files',
            'updated': updated_count,
            'paths': saved_paths,
        }
    except Exception as e:
        return {'ok': False, 'error': str(e)}


_node_ref = None

class DashboardHandler(http.server.BaseHTTPRequestHandler):
    """HTTP handler for dashboard HTML, JSON, and MJPEG streams."""
    def do_GET(self):
        """Serve dashboard pages, JSON data, MJPEG stream, and sim assets."""
        ctx = registry.make_context(self, _node_ref, {
            'get_param': _ros_get_param,
            'set_param': _ros_set_param,
            'save_defaults': _save_params_to_yaml,
            'param_defaults': _DEFAULT_PARAMS,
            'dashboard_html': DASHBOARD_HTML,
            'teach_html': TEACH_HTML,
        })
        registry.dispatch(ctx, 'GET', self.path)

    def do_POST(self):
        """Handle dashboard API POST requests."""
        ctx = registry.make_context(self, _node_ref, {
            'get_param': _ros_get_param,
            'set_param': _ros_set_param,
            'save_defaults': _save_params_to_yaml,
            'param_defaults': _DEFAULT_PARAMS,
            'dashboard_html': DASHBOARD_HTML,
            'teach_html': TEACH_HTML,
        })
        registry.dispatch(ctx, 'POST', self.path)

    def log_message(self, format, *args):
        """Suppress default HTTP logging."""
        pass

    def handle_one_request(self):
        """Override to suppress BrokenPipeError from disconnecting clients."""
        try:
            super().handle_one_request()
        except BrokenPipeError:
            pass
        except ConnectionResetError:
            pass


def main(args=None) -> None:
    global _node_ref
    cv2.setNumThreads(1)
    rclpy.init(args=args)
    load_default_params()
    node = DashboardNode()
    _node_ref = node

    # ── Dedicated param helper node (isolated from camera/subscription load) ──
    global _param_helper_node, _param_executor
    from rclpy.executors import SingleThreadedExecutor
    _param_helper_node = rclpy.create_node('dashboard_param_helper', use_global_arguments=False)
    _param_executor = SingleThreadedExecutor()
    _param_executor.add_node(_param_helper_node)

    def _spin_param_executor():
        while rclpy.ok():
            try:
                _param_executor.spin_once(timeout_sec=0.05)
            except Exception:
                break

    param_spin_thread = threading.Thread(target=_spin_param_executor, daemon=True)
    param_spin_thread.start()

    # Start HTTP server in background thread
    class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
        allow_reuse_address = True   # Prevent 'Address already in use' after crash
    try:
        dashboard_port = int(node.get_parameter('dashboard_port').value)
    except Exception:
        dashboard_port = 8080
    if not 1024 <= dashboard_port <= 65535:
        dashboard_port = 8080
    server = ThreadedHTTPServer(('0.0.0.0', dashboard_port), DashboardHandler)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    # Print user-friendly access URLs
    import socket
    hostname = socket.gethostname()
    try:
        ip = socket.getsockname() if hasattr(socket, 'getsockname') else '?'
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        ip = '?.?.?.?'
    node.get_logger().info(f'Dashboard live!')
    node.get_logger().info(f'  → http://{hostname}.local:{dashboard_port}')
    node.get_logger().info(f'  → http://{ip}:{dashboard_port}')

    # All dashboard callbacks share the default mutually exclusive group.
    # Additional executor workers only add contention; HTTP and parameter
    # requests already have their own threads.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        executor.shutdown()
        _param_executor.shutdown()
        _param_helper_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
