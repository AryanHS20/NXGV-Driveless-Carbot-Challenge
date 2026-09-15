#!/usr/bin/env python3
"""Verify the current NXGV artifact hash; optionally run one BPU forward pass."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--infer', action='store_true', help='Run on RDK X5 only')
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    manifest = json.loads((base/'model_manifest.json').read_text())
    path = args.model or base/manifest['file']
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != manifest['sha256']:
        raise SystemExit('FAIL: model hash differs from the reviewed manifest')
    print('Model hash matches NXGV YOLO11n manifest')
    if args.infer:
        import numpy as np
        try:
            from hobot_dnn import pyeasy_dnn as dnn
        except ImportError:
            from hobot_dnn_rdkx5 import pyeasy_dnn as dnn
        model = dnn.load(str(path))[0]
        outputs = model.forward([np.zeros((960,640), dtype=np.uint8)])
        if len(outputs) != 6:
            raise SystemExit('FAIL: expected six YOLO11 outputs')
        for i, output in enumerate(outputs):
            arr = np.squeeze(np.asarray(output.buffer))
            side = 640 // manifest['strides'][i//2]
            channels = 10 if i % 2 == 0 else 64
            if arr.shape not in ((side,side,channels), (channels,side,side)):
                raise SystemExit(f'FAIL: unexpected {manifest["outputs"][i]} shape {arr.shape}')
            print(manifest['outputs'][i], arr.shape, arr.dtype)
        print('BPU shape contract verified; detection accuracy still needs labeled images')


if __name__ == '__main__':
    main()
