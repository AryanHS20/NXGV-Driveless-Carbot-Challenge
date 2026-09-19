#!/usr/bin/env python3
"""Board-side BPU smoke test for YOLO11n sign models (run ON the RDK X5).

Uses the pyeasy_dnn API that is actually installed on the robot (the stock
verify_*.py scripts need hbm_runtime, which the board lacks). Same load path
as risabot_automode/signage_detector.py: dnn.load -> models[0] ->
forward([nv12]) -> o.buffer. No ROS needed.

Usage:
    python3 tools/smoke_unified16.py \
        --model tools/bpu_model/model_output/unified16_yolo11n_640x640_nv12.bin \
        --image tools/bpu_model/test_images/boom_open.jpg --expect boom_open
    python3 tools/smoke_unified16.py --model nxgv_yolo11n_640x640_nv12.bin \
        --image test_0.jpg --expect hill

PASS = model loads, 6 outputs (cls+box x strides 8/16/32), expected label
present above --conf. Works for the 10-class and 16-class models: classes
beyond the model's own count are simply never predicted.
"""
import argparse
import time

import cv2
import numpy as np

try:
    try:
        from hobot_dnn import pyeasy_dnn as dnn
    except ImportError:
        from hobot_dnn_rdkx5 import pyeasy_dnn as dnn
except ImportError:
    raise SystemExit('FAIL: neither hobot_dnn nor hobot_dnn_rdkx5 importable')

NAMES_16 = ['end_of_tunnel', 'hill', 'obstacle', 'parallel_P', 'perpendicular_P',
            'roundabout', 'speed_bump', 'traffic_lamp', 'warn_signals_ahead', 'tunnel',
            'traffic_red', 'traffic_yellow', 'traffic_green',
            'boom_closed', 'boom_partial', 'boom_open']
STRIDES = (8, 16, 32)


def bgr_to_nv12(bgr640):
    yuv = cv2.cvtColor(bgr640, cv2.COLOR_BGR2YUV_I420)
    y, u, v = yuv[0:640, :], yuv[640:800, :], yuv[800:960, :]
    uv = np.stack([u.ravel(), v.ravel()], axis=1).ravel().reshape(320, 640)
    return np.vstack((y, uv))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--image', default=None)
    ap.add_argument('--conf', type=float, default=0.30)
    ap.add_argument('--expect', default=None,
                    help='substring that must appear in a predicted label')
    args = ap.parse_args()

    t0 = time.time()
    models = dnn.load(args.model)
    model = models[0]
    print(f'load OK in {(time.time() - t0) * 1000:.0f} ms')

    if args.image is None:
        nv12 = np.zeros((960, 640), np.uint8)
    else:
        img = cv2.imread(args.image)
        if img is None:
            raise SystemExit(f'FAIL: cannot read image {args.image}')
        nv12 = bgr_to_nv12(cv2.resize(img, (640, 640)))

    t0 = time.time()
    outputs = model.forward([nv12])
    bufs = [np.asarray(o.buffer) for o in outputs]
    print(f'forward {(time.time() - t0) * 1000:.0f} ms, outputs: {len(bufs)}')
    assert len(bufs) >= 6, f'expected >=6 BPU outputs, got {len(bufs)}'

    ncls = None
    hits = []
    for li, s in enumerate(STRIDES):
        cls = np.asarray(bufs[li * 2], dtype=np.float32)
        cls = np.squeeze(cls)
        if cls.ndim == 3 and cls.shape[0] in (10, 16):  # NCHW -> HWNC
            cls = np.transpose(cls, (1, 2, 0))
        if not (cls.ndim == 3 and cls.shape[-1] in (10, 16)):
            print(f'stride {s}: unexpected cls shape {cls.shape}, skipping')
            continue
        ncls = cls.shape[-1]
        scores = 1 / (1 + np.exp(-cls))
        best = float(scores.max())
        flat = scores.reshape(-1, ncls)
        cid = int(flat[flat.max(-1).argmax()].argmax())
        label = NAMES_16[cid] if cid < len(NAMES_16) else f'class_{cid}'
        print(f'stride {s}: best {best:.3f} '
              f'(class {label if best > args.conf else "-"})')
        if best > args.conf:
            hits.append(label)

    if args.expect:
        ok = any(args.expect in h for h in hits)
        print(('SMOKE PASS' if ok else 'SMOKE FAIL') +
              f': expected [{args.expect}], got {hits}')
        raise SystemExit(0 if ok else 1)
    print('SMOKE PASS: image ran end-to-end on BPU')


if __name__ == '__main__':
    main()
