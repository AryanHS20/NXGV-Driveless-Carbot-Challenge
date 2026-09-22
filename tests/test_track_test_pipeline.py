"""Exercise lane-test authority and motor duty through real node classes."""

import json
import math
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np

import ros_stub

ros_stub.install()
ros_stub.module('rclpy.executors', ExternalShutdownException=type('ExternalShutdownException', (Exception,), {}))
from ros_stub import Message, Parameter, Twist
from risabot_automode.auto_driver import AutoDriver, ChallengeState
from risabot_automode.cmd_safety_controller import CmdSafetyController
from control_servo.servo_controller import ServoControllerV9
from risabot_v4_control.motion_executor import MotionExecutor
from risabot_v4_control.track_test_config import track_test_overrides
from risabot_v4_experimental.arbitration_shadow import ArbitrationShadow
from risabot_v4_experimental.trajectory_shadow import TrajectoryShadow
from risabot_v4_experimental.road_mask_shadow import RoadMaskShadow
from risabot_v4_experimental.bev_core import CameraProfile


class TrackTestPipelineTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch('time.monotonic', return_value=100.0)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.overrides = track_test_overrides()
        merged = {name: {'ros__parameters': {
            **ros_stub.PARAMS.get(name, {}).get('ros__parameters', {}), **values,
        }} for name, values in self.overrides.items()}
        params = patch.dict(ros_stub.PARAMS, merged)
        params.start()
        self.addCleanup(params.stop)

    def servo(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        with patch('os.path.expanduser', side_effect=lambda s: directory.name + '/' + s.removeprefix('~/')):
            node = ServoControllerV9()
        node.manual_mode = False
        node.motion_permitted = True
        node.last_auto_cmd_time = node.last_permit_time = 100.0
        return node

    @staticmethod
    def trajectory(steer=0.2):
        return {'enabled': True, 'blockers': [], 'selected_diagnostic_only': {
            'valid': True, 'command_steer_rad_diagnostic_only': steer,
        }}

    def test_lane_chain_slows_for_turn_without_stopping(self):
        driver = AutoDriver()
        driver.in_auto_mode = True
        driver.motion_permitted = True
        driver.permit_stamp = 100.0
        driver.v4_lane_status_callback(Message(json.dumps(self.trajectory())))
        driver._dash_counter = 9
        driver.publish_cmd_vel()
        self.assertEqual(driver.state, ChallengeState.LANE_FOLLOW)
        arb = ArbitrationShadow()
        arb._set('state', driver.dash_state_pub.messages[-1].data)
        arb._set('permit', True)
        arb._set('trajectory', self.trajectory())
        arb._evaluate()
        executor = MotionExecutor()
        self.assertTrue(all(not value for value in executor._gates.values()))
        executor._state_cb(driver.dash_state_pub.messages[-1])
        executor._proposal_cb(arb._proposal_pub.messages[-1])
        executor._control_loop()
        raw = executor._cmd_pub.messages[-1]
        self.assertGreaterEqual(raw.linear.x * 255.0, 40.0)
        self.assertLess(raw.linear.x * 255.0, 65.0)
        self.assertLess(raw.angular.z, 0.0)
        safety = CmdSafetyController()
        safety.last_image_t = 100.0
        safety._v4_cmd_cb(raw)
        for _ in range(4):
            safety.last_loop_t = 99.9
            safety._control_loop()
        self.assertTrue(safety.permit_pub.messages[-1].data)
        servo = self.servo()
        servo.process_twist(safety.cmd_pub.messages[-1])
        self.assertGreaterEqual(servo.target_motor_val, 40)
        self.assertLess(servo.target_motor_val, 65)
        self.assertLess(servo.target_servo_val, 80)

    def test_stale_camera_and_command_and_estop_still_stop(self):
        for fault in ('camera', 'command', 'estop'):
            node = CmdSafetyController()
            node.last_image_t = 100.0
            cmd = Twist()
            cmd.linear.x = 65.0 / 255.0
            node._v4_cmd_cb(cmd)
            if fault == 'camera':
                node.last_image_t = 98.0
            elif fault == 'command':
                node.v4_last_input_t = 98.0
            else:
                node.estop = True
            node._control_loop()
            self.assertEqual(node.output_cmd.linear.x, 0.0)
            self.assertFalse(node.permit_pub.messages[-1].data)

    def test_invalid_or_stale_lane_cannot_move(self):
        driver = AutoDriver()
        driver.in_auto_mode = driver.motion_permitted = True
        driver.permit_stamp = 100.0
        driver.v4_lane_status_callback(Message(json.dumps(self.trajectory())))
        driver.v4_lane_stamp = 98.0
        driver.publish_cmd_vel()
        self.assertEqual(driver.state, ChallengeState.LANE_RECOVERY)
        arb = ArbitrationShadow()
        arb._set('state', 'LANE_FOLLOW')
        arb._set('permit', True)
        status = self.trajectory()
        status['blockers'] = ['road mask is stale']
        arb._set('trajectory', status)
        arb._evaluate()
        self.assertEqual(json.loads(arb._proposal_pub.messages[-1].data)['action'], 'stop')

    def test_manual_startup_and_non_lane_states_never_request_motion(self):
        driver = AutoDriver()
        driver.publish_cmd_vel()
        self.assertEqual(driver.state, ChallengeState.MANUAL)
        node = MotionExecutor()
        node._state_stamp = node._proposal_stamp = 100.0
        node._proposal = {'source': 'trajectory', 'action': 'follow_curvature',
                          'reference': self.trajectory()['selected_diagnostic_only']}
        for state in ('MANUAL', 'TUNNEL', 'PARALLEL_PARK', 'LANE_RECOVERY', 'HILL'):
            node._state = state
            self.assertEqual(node._select(100.0).linear.x, 0.0)

    def test_servo_final_cap_and_reverse_stop_are_immediate(self):
        node = self.servo()
        cmd = Twist()
        cmd.linear.x = 0.65  # A mistaken m/s request must not become 100% duty.
        node.process_twist(cmd)
        self.assertEqual(node.target_motor_val, 65)
        node.sent_motor_val = -65
        node.apply_hardware(0, 80)
        self.assertEqual(node.target_motor_val, 0)

    def test_small_left_right_commands_round_symmetrically(self):
        node = self.servo()
        node.servo_range_left = node.servo_range_right = 50
        for steering, expected in ((0.015, 81), (-0.015, 79), (0.0, 80)):
            cmd = Twist()
            cmd.angular.z = steering
            node.process_twist(cmd)
            self.assertEqual(node.target_servo_val, expected)

    def test_manual_reference_controls_remain_proportional(self):
        node = self.servo()
        node.manual_mode = True
        node.joy_alive = node.joy_unlocked = True
        node.awaiting_neutral = False
        node.current_speed_limit = 100
        message = Message(
            axes=[0.0, 0.40, 0.20, 0.0, 0.0, 0.0, 0.0, 0.0],
            buttons=[0] * 15,
        )
        node.joy_callback(message)
        self.assertEqual(node.target_motor_val, 40)
        self.assertNotEqual(node.target_servo_val, node.servo_center)
        proportional_steer = node.target_servo_val

        node.rp_state = 'RECORDING'
        node.joy_callback(message)
        self.assertEqual(node.target_motor_val, 40)
        self.assertEqual(node.target_servo_val, proportional_steer)

    def test_track_manual_control_starts_at_full_range(self):
        self.assertEqual(
            self.overrides['servo_controller']['default_speed_index'], 4
        )

    def test_track_mode_does_not_invent_camera_calibration(self):
        node = TrajectoryShadow()
        node._profiles = {'primary': types.SimpleNamespace(calibrated=False)}
        self.assertIn('primary camera profile is uncalibrated', node._blockers(100.0))
        self.assertNotIn('no LiDAR scan', node._blockers(100.0))
        self.assertNotIn('vehicle geometry is not measured and validated', node._blockers(100.0))

    def test_vehicle_trim_and_invalid_settings(self):
        self.assertEqual(track_test_overrides('risabot1')['servo_controller']['servo_center'], 110)
        self.assertEqual(track_test_overrides()['servo_controller']['servo_center'], 80)
        for duty in (0.0, -1.0, 101.0, math.nan):
            with self.assertRaises(ValueError):
                track_test_overrides(motor_duty=duty)
        with self.assertRaises(ValueError):
            track_test_overrides('unknown')

    def test_clipped_camera_rows_are_not_steering_targets(self):
        node = TrajectoryShadow()
        node._profiles = {'primary': object()}
        node._road_status = {'corridor': {'primary': [
            {'forward_m': x, 'left_m': left, 'width_m': width,
             'boundaries_observed': observed}
            for x, left, width, observed in (
                (.37, .015, .20, False), (.41, .015, .25, False),
                (.45, .0125, .28, False), (.49, .0025, .30, False),
                (.53, 0., .32, True), (.57, 0., .32, True),
            )]}}
        with patch.object(node, '_blockers', return_value=[]), \
             patch.object(node, '_inputs_synchronized', return_value=True), \
             patch.object(node, '_expected_road_stamp', return_value=100.), \
             patch('risabot_v4_experimental.trajectory_shadow.generate_candidates', return_value=[]) as generate:
            node._try_process()
        reference = generate.call_args.args[0]
        self.assertEqual([p.x for p in reference], [.53, .57])
        self.assertTrue(all(p.y == 0. for p in reference))

    def test_lane_test_does_not_use_unverified_odometry_to_fuse_road(self):
        node = RoadMaskShadow()
        self.assertFalse(node._use_road_memory)
        node._profiles = {'primary': CameraProfile(
            name='primary', calibrated=True, resolution=(60, 60),
            camera_matrix=np.eye(3), distortion=np.zeros(5),
            source_points_px=np.array([[0,0],[59,0],[59,59],[0,59]], float),
            ground_points_m=np.array([[0,-.3],[.59,-.3],[.59,.29],[0,.29]], float),
            pixels_per_meter=100., forward_bounds_m=(0., .59), left_bounds_m=(-.3, .29))}
        image = np.full((60, 60, 3), 220, np.uint8)
        image[:, 20:40] = 40
        node._bridge = types.SimpleNamespace(
            imgmsg_to_cv2=lambda msg, **kw: image,
            cv2_to_imgmsg=lambda data, **kw: Message(data.copy(), header=None))
        # A fresh but arbitrarily wrong pose and nonempty history must not
        # select or paint a remembered lane during this camera-only test.
        node._pose = object()
        node._pose_mono = 100.
        node._memory = Mock()
        node._memory.stats.return_value = {'cells': 100}
        msg = Message(header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=100, nanosec=0), frame_id='camera'))
        with patch.object(node, '_publish_status'):
            node._process_image('primary', msg, np.full((60, 60), 255, np.uint8))
        self.assertEqual(node._last_error['primary'], '')
        np.testing.assert_array_equal(node._fused_pubs['primary'].messages[-1].data,
                                      node._connected_pubs['primary'].messages[-1].data)
        self.assertFalse(node._memory.render.called)
        self.assertFalse(node._memory.integrate.called)

    def test_track_planner_holds_a_recent_plan_through_one_bad_frame(self):
        node = TrajectoryShadow()
        previous = {'valid': True, 'command_steer_rad_diagnostic_only': 0.3}
        node._selected = previous
        node._last_candidates = [previous]
        node._last_success_mono = 99.7
        node._plan_hold = 0.6
        node._road_status = {'corridor': {'primary': []}}
        node._profiles = {'primary': object()}
        with patch.object(node, '_blockers', return_value=[]), \
             patch.object(node, '_inputs_synchronized', return_value=True), \
             patch.object(node, '_expected_road_stamp', return_value=100.):
            node._try_process()
        self.assertIs(node._selected, previous)
        self.assertEqual(node._last_error, '')
        self.assertIn('holding recent valid plan', node._last_warning)

    def test_low_support_cannot_reverse_a_recent_reliable_turn(self):
        node = TrajectoryShadow()
        node._profiles = {'primary': object()}
        node._mask = np.full((20, 20), 255, np.uint8)
        node._road_status = {'corridor': {'primary': [
            {'forward_m': x, 'left_m': 0.03, 'width_m': 0.32,
             'boundaries_observed': observed,
             'left_boundary_observed': observed,
             'right_boundary_observed': False}
            for x, observed in zip(
                (0.30, 0.40, 0.50, 0.60, 0.70, 0.80),
                (True, True, True, False, False, False),
            )
        ]}}
        node._reliable_steer_rad = 0.20
        node._reliable_steer_mono = 99.0
        candidate = types.SimpleNamespace(
            candidate_id=0, offset_m=0.0, valid=True, cost=1.0,
            minimum_support=0.0, road_blocked=10, obstacle_blocked=0,
            command_steer_rad=-0.5, reject_reason='',
            points=[types.SimpleNamespace(x=0.0, y=0.0, yaw=0.0),
                    types.SimpleNamespace(x=0.5, y=0.0, yaw=0.0)],
        )
        diagnostics = {
            'lateral_error_m': -0.03, 'heading_error_rad': -0.2,
            'curvature_per_m': -2.0, 'feedforward_steer_rad': -0.3,
            'evaluation_forward_m': 0.3,
        }
        with patch.object(node, '_blockers', return_value=[]), \
             patch.object(node, '_inputs_synchronized', return_value=True), \
             patch.object(node, '_expected_road_stamp', return_value=100.), \
             patch('risabot_v4_experimental.trajectory_shadow.generate_candidates',
                   return_value=[candidate]), \
             patch('risabot_v4_experimental.trajectory_shadow.centerline_steering_command',
                   return_value=(-0.50, diagnostics)):
            node._try_process()
        self.assertGreater(
            node._selected['command_steer_rad_diagnostic_only'], 0.0
        )
        self.assertEqual(
            node._selected['steering_source'], 'low_support_direction_hold'
        )

    def test_observed_centerline_remains_live_when_footprint_support_is_low(self):
        node = TrajectoryShadow()
        node._profiles = {'primary': object()}
        node._mask = np.full((20, 20), 255, np.uint8)
        node._road_status = {'corridor': {'primary': [
            {'forward_m': x, 'left_m': -0.03, 'width_m': 0.32,
             'boundaries_observed': True,
             'left_boundary_observed': True,
             'right_boundary_observed': True}
            for x in (0.30, 0.40, 0.50)
        ]}}
        node._reliable_steer_rad = 0.20
        node._reliable_steer_mono = 95.0
        candidate = types.SimpleNamespace(
            candidate_id=0, offset_m=0.0, valid=True, cost=1.0,
            minimum_support=0.0, road_blocked=10, obstacle_blocked=0,
            command_steer_rad=-0.5, reject_reason='',
            points=[types.SimpleNamespace(x=0.0, y=0.0, yaw=0.0),
                    types.SimpleNamespace(x=0.5, y=0.0, yaw=0.0)],
        )
        diagnostics = {
            'lateral_error_m': -0.03, 'control_lateral_error_m': -0.03,
            'heading_error_rad': -0.2, 'curvature_per_m': -2.0,
            'feedforward_steer_rad': -0.3, 'evaluation_forward_m': 0.3,
        }
        with patch.object(node, '_blockers', return_value=[]), \
             patch.object(node, '_inputs_synchronized', return_value=True), \
             patch.object(node, '_expected_road_stamp', return_value=100.), \
             patch('risabot_v4_experimental.trajectory_shadow.generate_candidates',
                   return_value=[candidate]), \
             patch('risabot_v4_experimental.trajectory_shadow.centerline_steering_command',
                   return_value=(-0.50, diagnostics)):
            node._try_process()
        self.assertLess(
            node._selected['command_steer_rad_diagnostic_only'], 0.0
        )
        self.assertEqual(
            node._selected['steering_source'],
            'low_support_observed_centerline',
        )
        self.assertEqual(
            node._selected['observed_centerline_fraction'], 1.0
        )

    def test_reverse_recovery_is_opt_in_and_requires_rear_clearance(self):
        disabled = MotionExecutor()
        reference = {'boundary_clearance_m': -0.01, 'lateral_error_m': 0.04}
        self.assertIsNone(disabled._boundary_recovery_command(reference, 100.0))

        enabled_overrides = track_test_overrides(enable_reverse_recovery=True)
        merged = {'v4_motion_executor': {'ros__parameters': {
            **ros_stub.PARAMS['v4_motion_executor']['ros__parameters'],
            **enabled_overrides['v4_motion_executor'],
        }}}
        with patch.dict(ros_stub.PARAMS, merged):
            enabled = MotionExecutor()
        enabled._rear_clearance_m = 0.60
        enabled._scan_stamp = 100.0
        self.assertIsNone(enabled._boundary_recovery_command(reference, 100.0))
        enabled._scan_stamp = 100.36
        command = enabled._boundary_recovery_command(reference, 100.36)
        self.assertLess(command.linear.x, 0.0)
        self.assertLess(command.angular.z, 0.0)
        self.assertEqual(enabled._last_source, 'boundary_reverse_recovery')

    def test_test_mode_cannot_change_mid_run(self):
        for node in (AutoDriver(), CmdSafetyController(), MotionExecutor()):
            result = node.set_parameters([Parameter('track_test_mode', value=False)])[0]
            self.assertFalse(result.successful)
