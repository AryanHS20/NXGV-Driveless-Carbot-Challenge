#!/usr/bin/env python3
"""Inspect and fit a Risabot 1 floor-board camera profile offline.

This reuses the validated detector and geometry checks from the Risabot 5
calibration tool, but binds the vehicle identity and evidence directory to
Risabot 1. It only writes a candidate profile with calibrated: false; the
operator must review the numbered corners and independent validation markers
before promoting it to the active runtime profile.
"""
import argparse
import json
from pathlib import Path

import cv2

import calibrate_risabot5_camera as calibration


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'calibration_risabot1'
PROFILE = ROOT / 'src/risabot_v4_experimental/config/camera_profiles.yaml'
VEHICLE = 'Risabot 1 (car 1)'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    inspect = sub.add_parser('inspect', help='detect 6x4 floor-board corners')
    inspect.add_argument('images', nargs='+', help='image paths or quoted glob patterns')
    inspect.add_argument('--roi', nargs=4, type=int, metavar=('X', 'Y', 'WIDTH', 'HEIGHT'))
    inspect.add_argument('--scale', nargs=2, type=float, default=[1., 1.],
                         metavar=('SX', 'SY'))
    inspect.add_argument('--output', required=True,
                         help='new directory under calibration_risabot1')
    fit = sub.add_parser('fit', help='write an unactivated measured profile candidate')
    fit.add_argument('manifest')
    fit.add_argument('--output', required=True,
                     help='new directory under calibration_risabot1')
    fit.add_argument('--board-only', action='store_true',
                     help='save a preliminary fit when independent markers are unavailable')
    args = parser.parse_args(argv)
    try:
        if args.command == 'inspect':
            calibration.inspect(args, evidence_root=EVIDENCE)
            return 0
        report = calibration.fit_session(
            args.manifest, args.output, allow_board_only=args.board_only,
            vehicle=VEHICLE, profile_path=PROFILE, evidence_root=EVIDENCE,
        )
        print(json.dumps({'geometry_pass': report['geometry_pass'],
                          'calibrated': False}))
        return 0 if report['geometry_pass'] else 2
    except (ValueError, KeyError, TypeError, OSError, cv2.error) as error:
        parser.exit(2, f'Calibration blocked: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
