#!/usr/bin/env python3
"""Signage Detector Node — YOLO11n BPU Model Inference via hobot_dnn.

REVIEW DRAFT for NXGV YOLO11n 10-class model. Drop into
src/risabot_automode/risabot_automode/ as signage_detector.py (replacing the
YOLOv5 version) after review. Topics unchanged except tunnel handling — see below.

BPU output protocol (D-Robotics ultralytics_yolo, YOLO11xDetect):
  6 outputs: [cls8, box8, cls16, box16, cls32, box32], NHWC float32
  cls  = raw logits  -> sigmoid on CPU, per-class threshold
  box  = DFL regs    -> softmax expected value + stride decode on CPU
  NMS  = class-wise on CPU (reused vectorised implementation)

NXGV class map (nxgv.yaml, alphabetical):
   0 end_of_tunnel_sign  -> TUNNEL_CONF_TOPIC False (advisory only, gated)
   1 hill_sign           -> HILL_SIGN_TOPIC
   2 obstacle_sign       -> OBSTACLE_SIGN_TOPIC (mission advisory)
   3 parallel_parking    -> PARKING_SIGN_TOPIC (width gate kept)
   4 perpendicular_park  -> PARKING_SIGN_TOPIC (width gate kept)
   5 roundabout_sign     -> debug only (auto_driver is time-based; wire here)
   6 speed_bump_sign     -> debug only (no topic yet; wire to speed logic)
   7 traffic_light lamp  -> TRAFFIC_LIGHT_TOPIC via HSV (red/green/unknown)
   8 traffic_warn_sign   -> debug only
   9 tunnel_sign         -> TUNNEL_CONF_TOPIC True (advisory only, gated)

NOTE: /tunnel_detected (TUNNEL_DETECTED_TOPIC) is NOT published by this node.
tunnel_wall_follower.py is the sole owner of that topic (LiDAR wall-pair
ground truth). This node only publishes the advisory /tunnel_confidence
signal above — do not repoint it back at /tunnel_detected, or the two nodes
will race for the same topic again (last-publisher-wins, non-deterministic).
"""

import time
import math
from .control_contract import confirmed_counter, fresh, observation_age
from typing import Dict, List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

# Import topics from our shared module (same as YOLOv5 node)
from .topics import (
    CAMERA_IMAGE_TOPIC,
    HILL_SIGN_TOPIC,
    OBSTACLE_SIGN_TOPIC,
    PARKING_SIGN_TOPIC,
    SIGNAGE_DEBUG_TOPIC,
    TRAFFIC_LIGHT_TOPIC,
    TUNNEL_CONF_TOPIC,
)

try:
    try:
        from hobot_dnn import pyeasy_dnn as dnn
    except ImportError:
        from hobot_dnn_rdkx5 import pyeasy_dnn as dnn
    BPU_AVAILABLE = True
except ImportError:
    BPU_AVAILABLE = False

# NXGV class names (must match nxgv.yaml order)
CLASS_NAMES = [
    'end_of_tunnel_sign',      # 0
    'hill_sign',               # 1
    'obstacle_sign',           # 2
    'parallel_parking_sign',   # 3
    'perpendicular_park_sign', # 4
    'roundabout_sign',         # 5
    'speed_bump_sign',         # 6
    'traffic_light',           # 7 lamp (HSV -> red/green)
    'traffic_warn_sign',       # 8 signals-ahead warning
    'tunnel_sign',             # 9
]

STRIDES = (8, 16, 32)
REG = 16  # DFL bins per box edge
_REGW = np.arange(REG, dtype=np.float32)


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x.astype(np.float32)))


class SignageDetector(Node):
    """BPU-accelerated NXGV signage detector (YOLO11n, 10-class)."""

    def __init__(self):
        super().__init__('signage_detector')

        # ── Tunable parameters ─────────────────────────────────────────
        self.declare_parameter('model_path', '/home/sunrise/nxgv_yolo11n_640x640_nv12.bin')
        self.declare_parameter('conf_threshold', 0.25)   # global fallback
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('show_debug', False)
        self.declare_parameter('heartbeat_sec', 0.5)
        self.declare_parameter('min_parking_sign_width', 0)
        self.declare_parameter('observation_timeout', 0.5)
        self.declare_parameter('tunnel_publish_enabled', True)  # False = never publish /tunnel_confidence (vision sign hint, advisory only)

        # Per-class thresholds. Strict where FPs were measured on laptop:
        # parking 0.50 (phone UI hit 0.72), obstacle 0.50 (blue clutter),
        # speed_bump 0.50 + traffic lamp 0.30 (HSV decides colour anyway).
        self.declare_parameter('thresh_end_tunnel',  0.40)  # 0
        self.declare_parameter('thresh_hill',        0.25)  # 1
        self.declare_parameter('thresh_obstacle',    0.50)  # 2
        self.declare_parameter('thresh_parallelp',   0.50)  # 3
        self.declare_parameter('thresh_perpendp',    0.50)  # 4
        self.declare_parameter('thresh_roundabout',  0.35)  # 5
        self.declare_parameter('thresh_speedbump',   0.50)  # 6
        self.declare_parameter('thresh_tl_lamp',     0.30)  # 7
        self.declare_parameter('thresh_tl_warn',     0.40)  # 8
        self.declare_parameter('thresh_tunnel',      0.40)  # 9

        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        self.bridge = CvBridge()
        self.bpu_available = BPU_AVAILABLE
        self._last_log_time = 0.0

        # ── Gated states ─────────────────────────────────────────────
        self.hill_sign_active = False
        self.parking_sign_active = False
        self.obstacle_sign_active = False
        self.tunnel_active = False
        self.traffic_light_active = 'unknown'
        self.last_observation = 0.0
        self.parking_kind = ''
        self.roundabout_active = False
        self._cnt_roundabout = 0
        self._cnt_warning = 0
        self.warning_active = False
        self._cnt_parallel = self._cnt_perpendicular = 0
        self.parallel_active = self.perpendicular_active = False

        self._cnt_hill = 0
        self._cnt_parking = 0
        self._cnt_obstacle = 0
        self._cnt_tunnel = 0
        self._cnt_endtunnel = 0
        self._cnt_red = 0
        self._cnt_green = 0
        self._last_tunnel_pub = None  # edge-triggered /tunnel_confidence (advisory vision sign hint)

        # ── ROS interfaces (topics UNCHANGED from YOLOv5 node + tunnel) ──
        self.parking_pub = self.create_publisher(Bool, PARKING_SIGN_TOPIC, 10)
        self.traffic_light_pub = self.create_publisher(String, TRAFFIC_LIGHT_TOPIC, 10)
        self.hill_pub = self.create_publisher(Bool, HILL_SIGN_TOPIC, 10)
        self.obstacle_pub = self.create_publisher(Bool, OBSTACLE_SIGN_TOPIC, 10)
        self.kind_pub = self.create_publisher(String, '/parking_sign_kind', 10)
        self.roundabout_pub = self.create_publisher(Bool, '/roundabout_detected', 10)
        self.warning_pub = self.create_publisher(Bool, '/traffic_warning_detected', 10)
        self.valid_pub = self.create_publisher(Bool, '/signage_valid', 10)
        self.tunnel_pub = self.create_publisher(Bool, TUNNEL_CONF_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, SIGNAGE_DEBUG_TOPIC, 10)

        self._heartbeat_timer = self.create_timer(
            float(self._param_cache['heartbeat_sec']), self.publish_states)

        if self.bpu_available:
            try:
                model_path = str(self._param_cache['model_path'])
                self.get_logger().info(f'Loading BPU model from: {model_path}')
                self.models = dnn.load(model_path)
                self.model = self.models[0]
                self.get_logger().info('BPU model loaded successfully.')
            except Exception as e:
                self.get_logger().error(f'Failed to load BPU model: {e}')
                self.bpu_available = False

        if not self.bpu_available:
            self.get_logger().warn(
                'hobot_dnn runtime not available. Node idles (no BPU inference).')

        self.color_sub = self.create_subscription(
            Image, CAMERA_IMAGE_TOPIC, self.image_callback,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.get_logger().info('Signage Detector (YOLO11n NXGV) initialized.')

    # ── Parameters ───────────────────────────────────────────────────
    _THRESH_KEYS = ('thresh_end_tunnel', 'thresh_hill', 'thresh_obstacle',
                    'thresh_parallelp', 'thresh_perpendp', 'thresh_roundabout',
                    'thresh_speedbump', 'thresh_tl_lamp', 'thresh_tl_warn',
                    'thresh_tunnel')

    def _update_param_cache(self) -> None:
        self._param_cache = {
            'model_path': str(self.get_parameter('model_path').value),
            'observation_timeout': float(self.get_parameter('observation_timeout').value),
            'conf_threshold': float(self.get_parameter('conf_threshold').value),
            'iou_threshold': float(self.get_parameter('iou_threshold').value),
            'show_debug': bool(self.get_parameter('show_debug').value),
            'heartbeat_sec': float(self.get_parameter('heartbeat_sec').value),
            'min_parking_sign_width': int(self.get_parameter('min_parking_sign_width').value),
            'tunnel_publish_enabled': bool(self.get_parameter('tunnel_publish_enabled').value),
        }
        for k in self._THRESH_KEYS:
            self._param_cache[k] = float(self.get_parameter(k).value)
        conf = self._param_cache['conf_threshold']
        self._class_thresh_array = np.array(
            [self._param_cache.get(k, conf) for k in self._THRESH_KEYS],
            dtype=np.float32)

    def _on_params(self, params) -> SetParametersResult:
        proposed = dict(self._param_cache)
        for p in params:
            if isinstance(p.value, (float, int)) and not isinstance(p.value, bool):
                if not math.isfinite(p.value) or p.value < 0:
                    return SetParametersResult(successful=False, reason='Finite nonnegative values required')
            if (p.name.startswith('thresh_') or p.name in ('iou_threshold', 'conf_threshold')) and not 0 <= p.value <= 1:
                return SetParametersResult(successful=False, reason='Threshold must be 0..1')
            if p.name == 'model_path' and p.value != self._param_cache['model_path']:
                return SetParametersResult(successful=False, reason='Restart required to load another model')
            if p.name in proposed:
                proposed[p.name] = p.value
        self._param_cache = proposed
        self._class_thresh_array = np.array([proposed[k] for k in self._THRESH_KEYS], dtype=np.float32)
        return SetParametersResult(successful=True)

    def publish_states(self) -> None:
        valid = fresh(self.last_observation, time.monotonic(), self._param_cache['observation_timeout'])
        if not valid:
            self.hill_sign_active = self.parking_sign_active = self.obstacle_sign_active = False
            self.parallel_active = self.perpendicular_active = self.roundabout_active = False
            self._cnt_hill = self._cnt_parking = self._cnt_obstacle = 0
            self._cnt_parallel = self._cnt_perpendicular = self._cnt_roundabout = 0
            self._cnt_warning = 0
            self.warning_active = False
            self._cnt_red = self._cnt_green = 0
            self.traffic_light_active = 'unknown'
            self.parking_kind = ''
        self.valid_pub.publish(Bool(data=valid))
        self.warning_pub.publish(Bool(data=self.warning_active if valid else False))
        self.kind_pub.publish(String(data=self.parking_kind))
        self.roundabout_pub.publish(Bool(data=self.roundabout_active))
        self.parking_pub.publish(Bool(data=self.parking_sign_active))
        self.traffic_light_pub.publish(String(data=self.traffic_light_active))
        self.hill_pub.publish(Bool(data=self.hill_sign_active))
        self.obstacle_pub.publish(Bool(data=self.obstacle_sign_active))
        # Edge-triggered /tunnel_confidence: an advisory vision-based hint only.
        # tunnel_wall_follower is the sole publisher of /tunnel_detected (the
        # actual state-machine signal) — this topic no longer writes to it,
        # so there's nothing left to stomp. Kept edge-triggered to reduce
        # topic chatter, not to avoid a conflict.
        if self._param_cache['tunnel_publish_enabled']:
            if self._last_tunnel_pub is None or self.tunnel_active != self._last_tunnel_pub:
                self.tunnel_pub.publish(Bool(data=self.tunnel_active))
                self._last_tunnel_pub = self.tunnel_active

    # ── NV12 preprocessing (unchanged: 640 BGR -> NV12) ──────────────
    def bgr_to_nv12(self, bgr640: np.ndarray) -> np.ndarray:
        yuv = cv2.cvtColor(bgr640, cv2.COLOR_BGR2YUV_I420)
        y = yuv[0:640, :]
        u = yuv[640:800, :]
        v = yuv[800:960, :]
        uv_planar = np.stack([u.ravel(), v.ravel()], axis=1).ravel().reshape(320, 640)
        return np.vstack((y, uv_planar))

    # ── YOLO11 BPU decode: sigmoid cls + DFL boxes, per level ────────
    def _decode_level(self, cls_out: np.ndarray, box_out: np.ndarray,
                      stride: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Decode one stride level. Returns (boxes xyxy@640, scores, class_ids)."""
        cls = np.squeeze(np.asarray(cls_out, dtype=np.float32))
        box = np.squeeze(np.asarray(box_out, dtype=np.float32))
        if cls.ndim == 3 and cls.shape[-1] == len(CLASS_NAMES):  # NHWC -> HW
            pass
        elif cls.ndim == 3 and cls.shape[0] == len(CLASS_NAMES):  # NCHW -> transpose
            cls = np.transpose(cls, (1, 2, 0))
            box = np.transpose(box, (1, 2, 0))
        h, w = cls.shape[:2]
        scores_all = _sigmoid(cls)
        class_ids = np.argmax(scores_all, axis=-1)
        max_scores = scores_all[np.arange(h)[:, None], np.arange(w), class_ids]
        keep = (max_scores >= self._class_thresh_array[class_ids])
        ys, xs = np.where(keep)
        if ys.size == 0:
            return (np.empty((0, 4), np.float32), np.empty((0,), np.float32),
                    np.empty((0,), np.int32))
        # DFL expected value over REG bins, 4 edges
        ltrb = box[ys, xs].reshape(-1, 4, REG)
        ltrb = ltrb - ltrb.max(axis=-1, keepdims=True)
        exp = np.exp(ltrb)
        off = (exp / exp.sum(axis=-1, keepdims=True) * _REGW).sum(axis=-1)  # (N,4)
        l, t, r, b = off[:, 0], off[:, 1], off[:, 2], off[:, 3]
        gx = xs.astype(np.float32) + 0.5
        gy = ys.astype(np.float32) + 0.5
        boxes = np.stack([(gx - l) * stride, (gy - t) * stride,
                          (gx + r) * stride, (gy + b) * stride], axis=1)
        return boxes.astype(np.float32), max_scores[ys, xs].astype(np.float32), class_ids[ys, xs].astype(np.int32)

    def nms(self, boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list:
        if len(boxes) == 0:
            return []
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[np.where(ovr <= iou_threshold)[0] + 1]
        return keep

    # ── Main callback ────────────────────────────────────────────────
    def image_callback(self, msg: Image) -> None:
        if not self.bpu_available:
            return
        age = observation_age(msg, self.get_clock().now().nanoseconds / 1e9)
        if age > self._param_cache['observation_timeout']:
            return
        observed_at = time.monotonic() - age
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            bgr640 = cv2.resize(bgr, (640, 640), interpolation=cv2.INTER_LINEAR)
            nv12 = self.bgr_to_nv12(bgr640)

            outputs = self.model.forward([nv12])
            bufs = [np.asarray(o.buffer) for o in outputs]
            # Expect 6: [cls8, box8, cls16, box16, cls32, box32]
            if len(bufs) < 6:
                self.get_logger().error(f'Expected 6 BPU outputs, got {len(bufs)}')
                return
            iou_thr = float(self._param_cache['iou_threshold'])
            all_boxes, all_scores, all_cids = [], [], []
            for li, stride in enumerate(STRIDES):
                bb, ss, cc = self._decode_level(bufs[li * 2], bufs[li * 2 + 1], stride)
                all_boxes.append(bb)
                all_scores.append(ss)
                all_cids.append(cc)
            if sum(len(b) for b in all_boxes) == 0:
                self.last_observation = observed_at
                self._update_states(np.empty((0, 4)), np.empty((0,), dtype=np.int32))
                self.publish_states()
                return
            boxes = np.concatenate(all_boxes)
            scores = np.concatenate(all_scores)
            cids = np.concatenate(all_cids)

            # Class-wise NMS
            keep_idx = []
            for c in np.unique(cids):
                m = cids == c
                for i in self.nms(boxes[m], scores[m], iou_thr):
                    keep_idx.append(np.where(m)[0][i])
            final_boxes = boxes[keep_idx]
            final_scores = scores[keep_idx]
            final_cids = cids[keep_idx].astype(np.int32)

            # Hybrid HSV traffic verification on lamp boxes (class 7)
            h_img, w_img = bgr640.shape[:2]
            verdicts = []
            for idx, cid in enumerate(final_cids):
                if cid == 7:
                    x1c, y1c = max(0, int(final_boxes[idx, 0])), max(0, int(final_boxes[idx, 1]))
                    x2c, y2c = min(w_img, int(final_boxes[idx, 2])), min(h_img, int(final_boxes[idx, 3]))
                    if x2c > x1c and y2c > y1c:
                        verdicts.append(self.classify_traffic_light_color(
                            bgr640[y1c:y2c, x1c:x2c]))
                    else:
                        verdicts.append('unknown')
                else:
                    verdicts.append(None)

            now = time.time()
            if now - self._last_log_time > 1.0:
                self.get_logger().info(
                    f'YOLO11n BPU: post_nms={len(final_boxes)} classes={list(final_cids)}')
                self._last_log_time = now

            self._update_states(final_boxes, final_cids, verdicts)
            self.last_observation = observed_at
            self.publish_states()
            if self._param_cache['show_debug']:
                self.draw_debug(bgr640, final_boxes, final_scores, final_cids)
        except Exception as e:
            self.last_observation = 0.0
            self.get_logger().error(f'Inference error: {e}')

    # ── Temporal gating (3 consecutive frames, decay otherwise) ──────
    def _bump(self, seen: bool, cnt: int, active=False) -> Tuple[int, bool]:
        return confirmed_counter(seen, cnt, active)

    def _update_states(self, boxes, class_ids, verdicts=None) -> None:
        cids = set(int(c) for c in np.atleast_1d(class_ids))
        self._cnt_hill, self.hill_sign_active = self._bump(1 in cids, self._cnt_hill, self.hill_sign_active)
        self._cnt_obstacle, self.obstacle_sign_active = self._bump(2 in cids, self._cnt_obstacle, self.obstacle_sign_active)
        self._cnt_roundabout, self.roundabout_active = self._bump(5 in cids, self._cnt_roundabout, self.roundabout_active)
        self._cnt_warning, self.warning_active = self._bump(8 in cids, self._cnt_warning, self.warning_active)

        # Parking with optional width gate
        saw_park = False
        min_w = int(self._param_cache['min_parking_sign_width'])
        for idx, cid in enumerate(np.atleast_1d(class_ids)):
            if int(cid) in (3, 4):
                if min_w > 0:
                    if boxes[idx, 2] - boxes[idx, 0] < min_w:
                        continue
                saw_park = True
                break
        valid_kinds = {int(cid) for idx, cid in enumerate(np.atleast_1d(class_ids))
                       if int(cid) in (3, 4) and (min_w <= 0 or boxes[idx, 2] - boxes[idx, 0] >= min_w)}
        self._cnt_parallel, self.parallel_active = self._bump(3 in valid_kinds, self._cnt_parallel, self.parallel_active)
        self._cnt_perpendicular, self.perpendicular_active = self._bump(4 in valid_kinds, self._cnt_perpendicular, self.perpendicular_active)
        self.parking_kind = ('parallel' if self.parallel_active and not self.perpendicular_active
                             else 'perpendicular' if self.perpendicular_active and not self.parallel_active else '')
        self.parking_sign_active = bool(self.parking_kind)

        # Tunnel pair: tunnel sets True, end-of-tunnel clears
        if 9 in cids:
            self._cnt_tunnel = min(10, self._cnt_tunnel + 1)
            self._cnt_endtunnel = 0
            if self._cnt_tunnel >= 3:
                self.tunnel_active = True
        elif 0 in cids:
            self._cnt_endtunnel = min(10, self._cnt_endtunnel + 1)
            self._cnt_tunnel = 0
            if self._cnt_endtunnel >= 3:
                self.tunnel_active = False
        else:
            self._cnt_tunnel = max(0, self._cnt_tunnel - 1)
            self._cnt_endtunnel = max(0, self._cnt_endtunnel - 1)

        # Traffic lamp: HSV verdicts decide red/green
        reds = greens = 0
        if verdicts:
            for cid, v in zip(np.atleast_1d(class_ids), verdicts):
                if int(cid) == 7:
                    if v == 'red':
                        reds += 1
                    elif v == 'green':
                        greens += 1
        if reds > 0 and reds >= greens:
            self._cnt_red = min(10, self._cnt_red + 1)
            self._cnt_green = 0
            if self._cnt_red >= 3:
                self.traffic_light_active = 'red'
        elif greens > 0:
            self._cnt_green = min(10, self._cnt_green + 1)
            self._cnt_red = 0
            if self._cnt_green >= 3:
                self.traffic_light_active = 'green'
        else:
            self._cnt_red = max(0, self._cnt_red - 1)
            self._cnt_green = max(0, self._cnt_green - 1)
            if self._cnt_red == 0 and self._cnt_green == 0:
                self.traffic_light_active = 'unknown'

    def classify_traffic_light_color(self, crop: np.ndarray) -> str:
        """HSV vote on lamp crop -> 'red' | 'green' | 'unknown'."""
        if crop is None or crop.size == 0:
            return 'unknown'
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        red = (cv2.countNonZero(cv2.inRange(hsv, np.array([0, 80, 80]), np.array([10, 255, 255])))
               + cv2.countNonZero(cv2.inRange(hsv, np.array([160, 80, 80]), np.array([180, 255, 255]))))
        yellow = cv2.countNonZero(cv2.inRange(hsv, np.array([11, 80, 80]), np.array([38, 255, 255])))
        green = cv2.countNonZero(cv2.inRange(hsv, np.array([40, 80, 80]), np.array([90, 255, 255])))
        total = crop.shape[0] * crop.shape[1]
        need = max(10, int(total * 0.02))
        if red + yellow >= need and red + yellow >= green:
            return 'red'
        if green >= need:
            return 'green'
        return 'unknown'

    # ── Debug overlay ────────────────────────────────────────────────
    _COLORS = [(255, 255, 255)] * 10

    def draw_debug(self, bgr640, boxes, scores, class_ids) -> None:
        dbg = bgr640.copy()
        for box, score, cid in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(dbg, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(dbg, f'{CLASS_NAMES[int(cid)]}:{score:.2f}', (x1, max(15, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(dbg,
                    f"HILL:{'Y' if self.hill_sign_active else '-'} "
                    f"PARK:{'Y' if self.parking_sign_active else '-'} "
                    f"TL:{self.traffic_light_active.upper()} "
                    f"TUN:{'Y' if self.tunnel_active else '-'}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))
        except Exception as e:
            self.get_logger().error(f'Debug publish failed: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SignageDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
