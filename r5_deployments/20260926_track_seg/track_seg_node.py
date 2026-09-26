#!/usr/bin/env python3
"""BPU track segmentation node for the V4 road-mask pipeline (RDK X5).

Runs the LRASPP track-seg model next to the YOLO11n signage node and hands its
mask to `v4_road_mask_shadow` as the road candidate, in place of the classical
HSV threshold.  It does not publish any vehicle command and does not touch
/lane_error.

  sub  /camera/color/image_raw                       Image (same feed as signage)
  pub  /v4_experimental/road/primary/seg_candidate   mono8 BEV mask, camera header
  pub  /v4_experimental/road/seg_status              String JSON, 1 Hz timing
  pub  /track_mask                                   mono8 camera-view mask (publish_mask)

Reverting: launch with road_source:=classical (the default). This node is then
never started and the shadow ignores the topic.
"""

import json
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from risabot_v4_experimental.bev_core import BevTransformer, CalibrationError, load_profiles

CAMERA_IMAGE_TOPIC = '/camera/color/image_raw'
SEG_CANDIDATE_TOPIC = '/v4_experimental/road/primary/seg_candidate'
SEG_STATUS_TOPIC = '/v4_experimental/road/seg_status'
TRACK_MASK_TOPIC = '/track_mask'

IN_W, IN_H = 512, 288  # must match track_seg_512x288_nv12.bin

try:
    from hbm_runtime import HB_HBMRuntime
    BPU_AVAILABLE = True
except ImportError:
    BPU_AVAILABLE = False


def bgr_to_nv12(bgr: np.ndarray) -> np.ndarray:
    h, w = bgr.shape[:2]
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
    y = yuv[0:h, :]
    u = yuv[h:h + h // 4, :]
    v = yuv[h + h // 4:h + h // 2, :]
    uv = np.stack([u.ravel(), v.ravel()], axis=1).ravel().reshape(h // 2, w)
    return np.vstack((y, uv))


def logits_to_mask(logits: np.ndarray) -> np.ndarray:
    """Model output (1,2,H,W) / (2,H,W) / (H,W,2) -> uint8 (H,W), 1 = track."""
    logits = np.asarray(logits, dtype=np.float32)
    if logits.ndim == 4:
        logits = logits[0]
    if logits.shape[0] == 2:
        return (logits[1] > logits[0]).astype(np.uint8)
    if logits.shape[-1] == 2:
        return (logits[..., 1] > logits[..., 0]).astype(np.uint8)
    raise ValueError(f'expected 2-class logits, got {logits.shape}')


class TrackSegNode(Node):
    def __init__(self):
        super().__init__('track_seg')
        self.declare_parameter('model_path', '/home/sunrise/track_seg_512x288_nv12.bin')
        self.declare_parameter('profile_path', '')
        self.declare_parameter('camera_topic', CAMERA_IMAGE_TOPIC)
        # [] = runtime default scheduling. Pin explicitly only after bench_bpu.py
        # shows it helps on this board.
        self.declare_parameter('bpu_cores', [])
        self.declare_parameter('priority', 0)
        self.declare_parameter('max_fps', 0.0)         # 0 = every camera frame
        self.declare_parameter('publish_mask', False)  # camera-view mask for debugging

        self.bridge = CvBridge()
        self.runtime = None
        self.model_name = ''
        self.input_name = ''
        self.transformer = None
        self.profile_error = ''
        try:
            profiles = load_profiles(str(self.get_parameter('profile_path').value))
            self.transformer = BevTransformer(profiles['primary'])
            self.cam_size = tuple(profiles['primary'].resolution)  # (w, h)
        except (CalibrationError, OSError, ValueError, KeyError) as exc:
            self.profile_error = str(exc)
            self.get_logger().error(f'Camera profile rejected, node idles: {exc}')

        if BPU_AVAILABLE and self.transformer is not None:
            try:
                path = str(self.get_parameter('model_path').value)
                self.get_logger().info(f'Loading BPU model from: {path}')
                self.runtime = HB_HBMRuntime(path)
                self.model_name = self.runtime.model_names[0]
                self.input_name = list(self.runtime.input_names[self.model_name])[0]
                cores = list(self.get_parameter('bpu_cores').value)
                if cores:
                    try:
                        self.runtime.set_scheduling_params(
                            priority=int(self.get_parameter('priority').value),
                            bpu_cores=[int(c) for c in cores])
                    except Exception as exc:
                        self.get_logger().warn(f'Could not set BPU scheduling ({exc}); using defaults.')
            except Exception as exc:
                self.get_logger().error(f'Failed to load BPU model: {exc}')
                self.runtime = None
        elif not BPU_AVAILABLE:
            self.get_logger().warn('hbm_runtime unavailable - node idles (road mask falls back).')

        self._min_period = 0.0
        max_fps = float(self.get_parameter('max_fps').value)
        if max_fps > 0.0:
            self._min_period = 1.0 / max_fps
        self._last_run = 0.0
        self._infer_ms = deque(maxlen=120)
        self._total_ms = deque(maxlen=120)
        self._done = deque(maxlen=120)
        self._frames = 0
        self._errors = 0
        self._skipped = 0
        self._last_error = ''

        self.cand_pub = self.create_publisher(Image, SEG_CANDIDATE_TOPIC, qos_profile_sensor_data)
        self.mask_pub = self.create_publisher(Image, TRACK_MASK_TOPIC, 2)
        self.status_pub = self.create_publisher(String, SEG_STATUS_TOPIC, 10)
        self.create_timer(1.0, self._publish_status)
        # depth 1: always work on the newest frame, never build a backlog.
        self.create_subscription(
            Image, str(self.get_parameter('camera_topic').value), self._on_image,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
        self.get_logger().info('Track-Seg node initialized.')

    def _on_image(self, msg: Image) -> None:
        if self.runtime is None or self.transformer is None:
            return
        now = time.monotonic()
        if self._min_period and now - self._last_run < self._min_period:
            self._skipped += 1
            return
        self._last_run = now
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cam_w, cam_h = self.cam_size
            if bgr.shape[1] != cam_w or bgr.shape[0] != cam_h:
                raise CalibrationError(f'frame {bgr.shape[1]}x{bgr.shape[0]} != profile {cam_w}x{cam_h}')
            nv12 = bgr_to_nv12(cv2.resize(bgr, (IN_W, IN_H), interpolation=cv2.INTER_LINEAR))
            t0 = time.monotonic()
            out = self.runtime.run({self.model_name: {self.input_name: nv12}})[self.model_name]
            t1 = time.monotonic()
            small = logits_to_mask(out[list(out.keys())[0]])
            cam_mask = cv2.resize(small, (cam_w, cam_h), interpolation=cv2.INTER_NEAREST) * 255
            bev_mask, _ = self.transformer.warp(cam_mask)
            bev_mask = ((bev_mask > 127).astype(np.uint8)) * 255

            cand = self.bridge.cv2_to_imgmsg(bev_mask, encoding='mono8')
            cand.header = msg.header  # exact camera stamp: the shadow matches on it
            self.cand_pub.publish(cand)
            if bool(self.get_parameter('publish_mask').value):
                dbg = self.bridge.cv2_to_imgmsg(cam_mask, encoding='mono8')
                dbg.header = msg.header
                self.mask_pub.publish(dbg)
            done = time.monotonic()
            self._infer_ms.append((t1 - t0) * 1000.0)
            self._total_ms.append((done - now) * 1000.0)
            self._done.append(done)
            self._frames += 1
        except Exception as exc:  # keep the node alive; the shadow falls back on silence
            self._errors += 1
            self._last_error = str(exc)
            self.get_logger().warn(f'track_seg frame failed: {exc}', throttle_duration_sec=5.0)

    @staticmethod
    def _pct(values, q):
        return round(float(np.percentile(values, q)), 2) if values else None

    def _publish_status(self) -> None:
        fps = None
        if len(self._done) >= 2 and self._done[-1] > self._done[0]:
            fps = round((len(self._done) - 1) / (self._done[-1] - self._done[0]), 2)
        payload = {
            'bpu_loaded': self.runtime is not None,
            'profile_error': self.profile_error,
            'frames': self._frames, 'skipped_by_rate_cap': self._skipped,
            'errors': self._errors, 'last_error': self._last_error,
            'fps': fps,
            'forward_ms_p50': self._pct(self._infer_ms, 50),
            'forward_ms_p95': self._pct(self._infer_ms, 95),
            'frame_ms_p50': self._pct(self._total_ms, 50),
            'frame_ms_p95': self._pct(self._total_ms, 95),
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = TrackSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
