#!/usr/bin/env python3
"""Read-only source preflight, usable on Windows and on the robot.

Does not start ROS, load BPU inference, pull git, or modify files.
"""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


def main():
    root = Path(__file__).resolve().parents[1]
    paths = subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard'], cwd=root, text=True).splitlines()
    failures=[]; compiled=0
    for name in paths:
        path=root/name
        if path.suffix == '.py':
            try:
                compile(path.read_bytes(),str(path),'exec')
                compiled+=1
            except Exception as exc:
                failures.append(f'{name}: {exc}')
    package_paths=[root/'src']
    nested_camera=root/'src/ros2_astra_camera/astra_camera/package.xml'
    if not nested_camera.exists():
        package_paths.append(root/'ros2_astra_camera')
    packages={}
    for source in package_paths:
        for path in source.rglob('package.xml'):
            if any((parent/'COLCON_IGNORE').exists() for parent in path.parents if parent.is_relative_to(source)):
                continue
            try:
                name=ET.parse(path).getroot().findtext('name')
                if name in packages:
                    failures.append(f'Duplicate package {name}: {path}')
                packages[name]=str(path.relative_to(root))
            except Exception as exc:
                failures.append(str(exc))
    manifest=json.loads((root/'tools/bpu_model/model_manifest.json').read_text())
    artifact=root/'tools/bpu_model'/manifest['file']
    if hashlib.sha256(artifact.read_bytes()).hexdigest() != manifest['sha256']:
        failures.append('Current model hash mismatch')
    for name in ('control_servo','obstacle_avoidance','obstacle_avoidance_camera','ydlidar_ros2_driver'):
        if not (root/name/'COLCON_IGNORE').exists():
            failures.append('Legacy root package not excluded: '+name)
    print(json.dumps({'compiled_python':compiled,'packages':packages,
                      'recommended_base_paths':[str(p) for p in package_paths],
                      'model_hash_ok':not any('hash' in f for f in failures),
                      'failures':failures},indent=2))
    return bool(failures)


if __name__ == '__main__':
    sys.exit(main())
