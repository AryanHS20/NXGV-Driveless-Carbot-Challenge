"""Behavioral regressions: real node classes, inert ROS and hardware interfaces."""
import json
import math
import tempfile
import types
import unittest
from unittest.mock import patch
import numpy as np
import ros_stub
ros_stub.install()
from ros_stub import Message, Twist, Odometry, Parameter
from risabot_automode.auto_driver import AutoDriver, ChallengeState as S
from risabot_automode.cmd_safety_controller import CmdSafetyController
from risabot_automode.signage_detector import SignageDetector
from risabot_automode.line_follower_camera import LineFollowerCamera
from risabot_automode.tunnel_wall_follower import TunnelWallFollower
from risabot_automode.obstruction_avoidance import ObstructionAvoidance, AvoidState
from control_servo.servo_controller import ServoControllerV9
from risabot_automode.bag_regression_validator import BagRegressionValidator
from risabot_automode.parking_controller import ParkingController, ParkingPhase
from risabot_automode.control_contract import swept_path_clear
from risabot_automode.parameter_values import coerce_parameter_value
from risabot_sim.sim_servo_bridge import SimServoBridge


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.clock=patch('time.monotonic',return_value=100.0); self.clock.start(); self.addCleanup(self.clock.stop)
        ros_stub.NOW=100.0

    def car(self):
        d=AutoDriver(); d.in_auto_mode=True; d.state_entry_time=99
        d.motion_permitted=True; d.permit_stamp=100
        d.boom_gate_last_time=100.
        for attr in ('lane_stamp','lane_lost_stamp','obstruction_last_time','tunnel_last_time',
                     'obstruction_cmd_stamp','tunnel_cmd_stamp'):
            setattr(d,attr,100.)
        return d

    def safe(self):
        c=CmdSafetyController(); c.last_image_t=c.last_scan_t=c.signage_stamp=c.last_input_t=100.
        c.scan_valid=c.signage_valid=True; c.last_loop_t=99.98
        c.last_scan=self.scan()
        return c

    def scan(self):
        return Message(ranges=[2.]*360, angle_min=-math.pi, angle_increment=math.pi/180,
                       range_min=.02, range_max=16., header=Odometry().header)

    def image(self):
        return Message(header=Odometry().header, width=320, height=240, data=bytes(320*240*3))

    def servo(self):
        tmp=tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup)
        with patch('os.path.expanduser',side_effect=lambda s:tmp.name+'/'+s.removeprefix('~/')):
            v=ServoControllerV9()
        v.manual_mode=False; v.last_auto_cmd_time=v.last_permit_time=100.
        v.motion_permitted=True
        v.joy_unlocked=True; v.joy_alive=True; v.last_joy_time=100.
        return v

    def test_startup_requires_observations(self):
        d=AutoDriver(); d.in_auto_mode=True; d.publish_cmd_vel()
        self.assertEqual(d.last_cmd.linear.x,0)

    def test_camera_processing_expiry_stops(self):
        d=self.car(); d.lane_stamp=98.; d.publish_cmd_vel()
        self.assertEqual(d.state,S.LANE_RECOVERY); self.assertEqual(d.last_cmd.linear.x,0)

    def test_red_preempts_obstruction(self):
        d=self.car(); d.traffic_light_state='red'; d.traffic_light_last_time=100.
        d.obstruction_active=True; d.obstruction_cmd.linear.x=.12; d.publish_cmd_vel()
        self.assertEqual(d.last_cmd.linear.x,0); self.assertEqual(d.state,S.TRAFFIC_LIGHT)

    def test_unknown_does_not_release_red(self):
        d=self.car(); d.traffic_light_callback(Message('red')); d.publish_cmd_vel()
        d.traffic_light_callback(Message('unknown')); d.publish_cmd_vel()
        self.assertEqual(d.last_cmd.linear.x,0)
        d.traffic_light_callback(Message('green')); d.publish_cmd_vel()
        self.assertGreater(d.last_cmd.linear.x,0)

    def test_gate_preempts_tunnel(self):
        d=self.car(); d.tunnel_detected=True; d.tunnel_cmd.linear.x=.12
        d.boom_gate_callback(Message(False)); d.publish_cmd_vel()
        self.assertEqual(d.last_cmd.linear.x,0)

    def test_warning_only_does_not_stop_lane_following(self):
        d = self.car()
        d._warning_cb(Message(True))
        d.traffic_light_callback(Message('unknown'))
        d.publish_cmd_vel()
        self.assertGreater(d.last_cmd.linear.x, 0)

    def test_uncertain_lamp_hold_expires_without_confirmed_red(self):
        d = self.car()
        for light in ('unresolved', 'unknown'):
            d.traffic_light_callback(Message(light))
            d.publish_cmd_vel()
            self.assertEqual(d.last_cmd.linear.x, 0)
            self.assertEqual(d.state, S.TRAFFIC_LIGHT)
        d.lamp_pending_stamp = 98.0
        d.publish_cmd_vel()
        self.assertGreater(d.last_cmd.linear.x, 0)

    def test_estop_label_takes_priority_over_red(self):
        d = self.car()
        d.traffic_light_callback(Message('red'))
        d.cmd_safety_estop = True
        d.publish_cmd_vel()
        self.assertEqual(d.state, S.EMERGENCY_STOP)

    def test_warning_and_uncertain_lamp_have_distinct_outputs(self):
        s = SignageDetector()
        s._update_states(np.zeros((1, 4)), np.array([8]))
        self.assertEqual(s.traffic_light_active, 'unknown')
        for _ in range(5):
            s._update_states(np.zeros((1, 4)), np.array([7]), ['unknown'])
        self.assertEqual(s.traffic_light_active, 'unresolved')
        for _ in range(5):
            s._update_states(np.zeros((1, 4)), np.array([7]), ['green'])
        self.assertEqual(s.traffic_light_active, 'green')
        s._update_states(np.empty((0, 4)), np.array([], dtype=int))
        self.assertEqual(s.traffic_light_active, 'unknown')

    def test_transient_lamp_detection_does_not_stop(self):
        s = SignageDetector()
        for _ in range(4):
            s._update_states(np.zeros((1, 4)), np.array([7]), ['unknown'])
            self.assertEqual(s.traffic_light_active, 'unknown')

    def test_no_reverse_for_proximity_flag(self):
        d=self.car(); d.obstacle_active=True; d.publish_cmd_vel()
        self.assertEqual(d.last_cmd.linear.x,0)

    def test_obstacle_sign_advisory_stops_until_lidar_behavior(self):
        d=self.car(); d.obstacle_sign_callback(Message(True)); d.publish_cmd_vel()
        self.assertIn('/obstacle_sign_detected',d.subs)
        self.assertEqual(d.last_cmd.linear.x,0)
        self.assertIn('OBSTACLE SIGN',d.stop_reason)
        d.obstruction_active=True; d.obstruction_cmd.linear.x=.1; d.publish_cmd_vel()
        self.assertGreater(d.last_cmd.linear.x,0)
        stale=self.car(); stale.obstacle_sign_active=True; stale.obstacle_sign_stamp=98.
        stale.publish_cmd_vel(); self.assertGreater(stale.last_cmd.linear.x,0)

    def test_lane_lost_stops_roundabout(self):
        d=self.car(); d.roundabout_seen=True; d.lane_lost=True; d.publish_cmd_vel()
        self.assertEqual(d.last_cmd.linear.x,0)

    def test_absolute_heading_has_no_steering_effect(self):
        d=self.car(); d.fused_heading_rad=math.pi/2; d.publish_cmd_vel()
        self.assertEqual(d.last_cmd.angular.z,0)

    def test_finish_latches(self):
        d=self.car(); d.mission_finished=True
        for _ in range(3):
            d.publish_cmd_vel(); self.assertEqual(d.state,S.FINISHED); self.assertEqual(d.last_cmd.linear.x,0)

    def test_idle_does_not_complete_unacknowledged_parking(self):
        d=self.car(); d.parking_requested='parallel'; d.parking_request_time=99.
        d.rp_state='IDLE'; d.publish_cmd_vel()
        self.assertNotIn('parallel',d.completed_parking)

    def test_missing_recording_fault_stops_mission(self):
        d=self.car(); d.parking_requested='parallel'; d.parking_request_time=99.
        d.record_playback_state_callback(Message(json.dumps(dict(state='IDLE',kind='parallel',result='missing_recording'))))
        d.publish_cmd_vel(); self.assertTrue(d.mission_fault); self.assertEqual(d.last_cmd.linear.x,0)

    def test_two_acknowledged_parking_completions_finish(self):
        d=self.car()
        for kind in ('parallel','perpendicular'):
            d.parking_requested=kind; d.parking_request_time=99.
            d.record_playback_state_callback(Message(json.dumps(dict(state='PLAYBACK',kind=kind,result='running'))))
            d.publish_cmd_vel(); self.assertTrue(d.parking_acknowledged)
            d.record_playback_state_callback(Message(json.dumps(dict(state='IDLE',kind=kind,result='complete'))))
            d.publish_cmd_vel(); self.assertIn(kind,d.completed_parking)
        self.assertEqual(d.state,S.FINISHED)

    def test_gate_observation_selects_route_not_distance(self):
        for gate,route in ((True,'through'),(False,'right')):
            d=self.car(); d._roundabout_cb(Message(True)); d.boom_gate_callback(Message(gate)); d.distance=100.
            d.publish_cmd_vel(); self.assertEqual(d.route,route); self.assertTrue(d.roundabout_seen)

    def test_scan_expiry_stops_safety_output(self):
        c=self.safe(); c.last_scan_t=98.; c.output_cmd.linear.x=.15; c._control_loop()
        self.assertEqual(c.output_cmd.linear.x,0); self.assertFalse(c.permit_pub.messages[-1].data)

    def test_estop_bypasses_slew(self):
        c=self.safe(); c.output_cmd.linear.x=.15; c.estop=True; c._control_loop()
        self.assertEqual(c.output_cmd.linear.x,0)

    def test_nan_does_not_create_forward_motion(self):
        c=self.safe(); cmd=Twist(); cmd.linear.x=float('nan'); c._raw_cmd_cb(cmd); c._control_loop()
        self.assertEqual(c.output_cmd.linear.x,0)

    def test_signage_invalid_removes_permit(self):
        c=self.safe(); c.signage_valid=False; c._control_loop()
        self.assertFalse(c.permit_pub.messages[-1].data)

    def test_parameter_update_uses_proposed_threshold(self):
        s=SignageDetector(); s.set_parameters([Parameter('thresh_parallelp',value=.9)])
        self.assertAlmostEqual(float(s._class_thresh_array[3]),.9,places=5)

    def test_sign_miss_never_activates_two_hit_candidate(self):
        s=SignageDetector(); count=0; active=False
        for seen in (True,True,False): count,active=s._bump(seen,count,active)
        self.assertFalse(active)

    def test_signage_expiry_clears_observations(self):
        s=SignageDetector(); s.last_observation=98; s.parking_kind='parallel'; s.parking_sign_active=True
        s.publish_states(); self.assertFalse(s.valid_pub.messages[-1].data); self.assertEqual(s.kind_pub.messages[-1].data,'')

    def test_empty_detection_still_publishes_debug_frame(self):
        s = SignageDetector()
        s.bpu_available = True
        s._param_cache['show_debug'] = True
        s.bridge = types.SimpleNamespace(imgmsg_to_cv2=lambda *a: np.zeros((240, 320, 3), dtype=np.uint8))
        s.model = types.SimpleNamespace(forward=lambda *a: [types.SimpleNamespace(buffer=np.zeros(1)) for _ in range(6)])
        empty = (np.empty((0, 4)), np.empty(0), np.empty(0, dtype=np.int32))
        with patch.object(s, '_decode_level', return_value=empty), patch.object(s, 'draw_debug') as draw:
            s.image_callback(self.image())
            draw.assert_called_once()
            self.assertEqual(len(draw.call_args.args[1]), 0)
            self.assertTrue(s.valid_pub.messages[-1].data)
            s._param_cache['show_debug'] = False
            s.image_callback(self.image())
            self.assertEqual(draw.call_count, 1)

    def test_kalman_accepts_centered_measurement(self):
        lane=LineFollowerCamera(); lane._kalman.x=np.array([.3,0.]); lane._param_cache['poly_fit_enabled']=False
        lane.bridge=types.SimpleNamespace(imgmsg_to_cv2=lambda *a:np.zeros((240,320,3),dtype=np.uint8))
        def centered(*_):
            lane._last_measured_count=4; lane._last_inferred_count=0; lane._last_point_measured=[True]*4
            return [],[],[(160,110),(160,90),(160,70),(160,50)],[1.]*4,4
        lane._detect_sliding_windows=centered
        for _ in range(30): lane.color_callback(self.image())
        self.assertLess(abs(lane.lane_error),.05)

    def test_single_border_without_learned_width_is_not_a_lane(self):
        lane=LineFollowerCamera(); lane._param_cache['invert_binary']=False
        binary=np.zeros((120,320),dtype=np.uint8); binary[:,50:58]=255
        _,_,centers,_,valid=lane._detect_scanlines(binary,120,320)
        self.assertEqual(valid,0); self.assertEqual(centers,[])
        self.assertEqual(lane._last_measured_count,0)

    def test_two_borders_are_measured_not_inferred(self):
        lane=LineFollowerCamera(); lane._param_cache['invert_binary']=False
        binary=np.zeros((120,320),dtype=np.uint8)
        binary[:,60:68]=255; binary[:,250:258]=255
        _,_,_,_,valid=lane._detect_scanlines(binary,120,320)
        self.assertGreaterEqual(valid,2)
        self.assertEqual(lane._last_measured_count,valid)
        self.assertEqual(lane._last_inferred_count,0)

    def test_implausibly_wide_room_edges_are_rejected(self):
        lane=LineFollowerCamera(); lane._param_cache['invert_binary']=False
        binary=np.zeros((120,320),dtype=np.uint8)
        binary[:,20:28]=255; binary[:,292:300]=255
        _,_,centers,_,valid=lane._detect_scanlines(binary,120,320)
        self.assertEqual(valid,0); self.assertEqual(centers,[])

    def test_robust_polyfit_rejects_large_center_outlier(self):
        lane=LineFollowerCamera()
        y=np.linspace(0.,1.,8); x=.15*y*y; x[4]+=.8
        poly,mask=lane._robust_polyfit(y,x,np.ones(8))
        self.assertIsNotNone(poly); self.assertFalse(mask[4])
        self.assertLess(abs(np.polyval(poly,.5)-.15*.25),.03)

    def test_signage_preview_size_and_rate_limit(self):
        s = SignageDetector()
        s.bridge = types.SimpleNamespace(cv2_to_imgmsg=lambda image, **kw: Message(data=image.shape))
        args = (np.zeros((640, 640, 3), dtype=np.uint8), [], [], [])
        s.draw_debug(*args)
        s.draw_debug(*args)
        self.assertEqual(len(s.debug_pub.messages), 1)
        self.assertEqual(s.debug_pub.messages[0].data, (240, 320, 3))
        with patch('time.monotonic', return_value=100.11):
            s.draw_debug(*args)
        self.assertEqual(len(s.debug_pub.messages), 2)

    def test_ipm_false_is_honored_by_sliding(self):
        lane=LineFollowerCamera(); lane.bridge=types.SimpleNamespace(imgmsg_to_cv2=lambda *a:np.zeros((240,320,3),dtype=np.uint8))
        with patch.object(lane,'_apply_ipm') as warp:
            lane.color_callback(self.image()); warp.assert_not_called()

    def test_tunnel_heartbeat_expires_motion(self):
        t=TunnelWallFollower(); t.scan_stamp=98; t.last_in_tunnel=True; t.last_cmd.linear.x=.12
        t._heartbeat_publish(); self.assertEqual(t.cmd_vel_pub.messages[-1].linear.x,0)

    def test_dodge_stops_on_new_close_obstacle(self):
        o=ObstructionAvoidance(); o.state=AvoidState.SPLINE_DODGE; o.dodge_start_time=100.
        o.scan_valid=True; o.scan_stamp=o.odom_stamp=100.; o.min_forward_dist=.05
        o.control_loop(); self.assertEqual(o.cmd_vel_pub.messages[-1].linear.x,0)

    def test_left_dodge_uses_negative_servo_command(self):
        o=ObstructionAvoidance(); o.state=AvoidState.SPLINE_DODGE; o.dodge_start_time=100.; o.avoid_dir=1
        o.scan_valid=o.selected=True; o.scan_stamp=o.odom_stamp=o.selected_stamp=100.; o.min_forward_dist=1.
        o.control_loop(); self.assertLess(o.cmd_vel_pub.messages[-1].angular.z,0)
        self.assertEqual(o.dist_progress,0) # no commanded-speed integration

    def test_playback_estop_cancels_timer_and_blocks_next_step(self):
        v=self.servo(); v.record_buffer=[dict(motor_pwm=60,servo_angle=100)]*5
        v._start_playback(); v._estop_callback(Message(True)); v.bot.writes=[]; v._playback_step()
        self.assertEqual(v.rp_state,'IDLE'); self.assertNotEqual(v.playback_result,'complete')
        self.assertFalse(any(k=='motor' and a[0] for k,a in v.bot.writes))

    def test_permit_loss_cancels_playback(self):
        v=self.servo(); v.record_buffer=[dict(motor_pwm=60,servo_angle=100)]*5
        v._start_playback(); v._permit_callback(Message(False)); self.assertEqual(v.rp_state,'IDLE')
        self.assertEqual(v.target_motor_val,0)

    def test_recording_path_traversal_rejected(self):
        v=self.servo(); self.assertFalse(v._load_recording_by_name('../other'))

    def test_motor_duty_respects_library_range(self):
        v=self.servo(); v.apply_hardware(127,100); self.assertEqual(v.target_motor_val,100)

    def test_playback_publishes_request_without_hardware_write(self):
        v=self.servo(); v.bot.writes=[]; v.record_buffer=[dict(motor_pwm=60,servo_angle=100)]*5
        v._start_playback()
        self.assertFalse(v.bot.writes)
        self.assertGreater(v.playback_cmd_pub.messages[-1].linear.x,0)

    def test_safe_zero_is_applied_during_playback(self):
        v=self.servo(); v.rp_state='PLAYBACK'; v.target_motor_val=60
        v.cmd_vel_auto_callback(Twist())
        self.assertEqual(v.target_motor_val,0)

    def test_brain_timeout_removes_playback_permit(self):
        c=self.safe(); c.last_input_t=98; c.playback_active=True
        c.playback_cmd_stamp=c.playback_state_stamp=100.; c.playback_cmd.linear.x=.15
        c._control_loop(); self.assertEqual(c.output_cmd.linear.x,0)
        self.assertFalse(c.permit_pub.messages[-1].data)

    def test_safety_selects_playback_and_checks_rear_clearance(self):
        c=self.safe(); c.playback_active=True; c.playback_cmd_stamp=c.playback_state_stamp=100.
        c.playback_cmd.linear.x=-.12
        # angle_min=-pi + mount pi => index 180 faces rear.
        c.last_scan.ranges[180]=.3
        c._control_loop(); self.assertFalse(c.permit_pub.messages[-1].data)
        self.assertEqual(c.output_cmd.linear.x,0)

    def test_swept_path_detects_obstacle_ahead(self):
        scan=self.scan(); scan.ranges[0]=.3
        self.assertFalse(swept_path_clear(scan,.15,0))
        self.assertTrue(swept_path_clear(scan,-.15,0))

    def test_old_sensor_header_is_not_refreshed_on_receipt(self):
        c=self.safe(); scan=self.scan(); scan.header.stamp.sec=90
        c._scan_cb(scan); self.assertFalse(c.scan_valid)
        c._control_loop(); self.assertFalse(c.permit_pub.messages[-1].data)

    def test_parameter_parser_preserves_declared_double(self):
        self.assertEqual(coerce_parameter_value('0',3),0.)
        self.assertIsInstance(coerce_parameter_value('0',3),float)
        with self.assertRaises(ValueError): coerce_parameter_value('nan',3)
        with self.assertRaises(ValueError): coerce_parameter_value('1.2',2)

    def test_validator_requires_actual_samples(self):
        v=BagRegressionValidator(); v._finalize()
        self.assertFalse(v.result_ok)
        self.assertIn('missing_samples:health',v.summary['failures'])
        self.assertIn('missing_samples:cmd',v.summary['failures'])

    def test_validator_uses_counter_window_baseline(self):
        v=BagRegressionValidator()
        v._cmd_safety_cb(Message(json.dumps(dict(timeout_count=50,estop_count=2))))
        self.assertEqual(v.cmd_safety_timeout_count,0)
        v._cmd_safety_cb(Message(json.dumps(dict(timeout_count=52,estop_count=3))))
        self.assertEqual(v.cmd_safety_timeout_count,2)
        self.assertEqual(v.cmd_safety_estop_count,1)

    def test_validator_retains_bad_loop_sample(self):
        v=BagRegressionValidator()
        for rate in (1.,50.):
            v._loop_cb(Message(json.dumps(dict(node='auto_driver',loop='auto_driver_cmd',target_hz=50,avg_hz=rate,overruns=1,samples=50))))
        self.assertEqual(v.loop_latest['auto_driver:auto_driver_cmd']['avg_hz'],1)

    def test_simulation_converts_right_steering_to_negative_yaw_rate(self):
        s=SimServoBridge(); cmd=Twist(); cmd.linear.x=.15; cmd.angular.z=.5
        s._cmd_callback(cmd); self.assertLess(s.cmd_pub.messages[-1].angular.z,0)
        self.assertEqual(s.cmd_pub.messages[-1].linear.x,.15)

    def test_parking_turn_moves_chassis_and_finishes_on_yaw(self):
        p=ParkingController(); p.scan_valid=True; p.scan_stamp=p.odom_stamp=100.
        p._start_phase(ParkingPhase.PERP_TURN_IN); p.control_loop()
        self.assertGreater(p.cmd_vel_pub.messages[-1].linear.x,0)
        p.cumulative_yaw=math.pi/2; p.control_loop()
        self.assertEqual(p.phase,ParkingPhase.PERP_FORWARD)
        self.assertEqual(p.cmd_vel_pub.messages[-1].linear.x,0)

    def test_lane_route_requires_visible_distinct_branches(self):
        lane=LineFollowerCamera(); lane._route_callback(Message('right'))
        binary=np.zeros((120,320),dtype=np.uint8)
        for x in (25,135,245): binary[:,x:x+6]=255
        for _ in range(3): lane._select_visible_branch(binary)
        self.assertEqual(lane.route_confirmed,'right')
        self.assertGreater(lane._expected_left,100)


if __name__ == '__main__': unittest.main()
