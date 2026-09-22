#!/usr/bin/env python3
"""Launch the installed lane-test stack in MANUAL with live diagnostics."""

import json
from pathlib import Path
import subprocess


assert subprocess.check_output(['hostname'], text=True).strip() == 'risabot5'
service = Path('/etc/systemd/system/risabot5-track-stack.service')
if service.is_file():
    subprocess.check_call([
        'sudo', '-n', 'systemctl', 'start', 'risabot5-track-stack.service'
    ])
    print(json.dumps({
        'service': 'risabot5-track-stack.service',
        'status': subprocess.check_output([
            'systemctl', 'is-active', 'risabot5-track-stack.service'
        ], text=True).strip(),
        'startup_mode': 'MANUAL', 'requested_motor_duty_percent': 65,
        'dashboard': 'http://192.168.137.74:8080',
        'reverse_recovery_enabled': False,
    }))
    raise SystemExit(0)
profile = Path('/home/sunrise/risabot5_profiles/camera_profiles.yaml')
assert profile.is_file()
processes = subprocess.check_output(['ps', '-eo', 'args'], text=True).splitlines()
conflicts = [line for line in processes if any(marker in line for marker in (
    '/lib/control_servo/servo_controller', 'ros2 launch /tmp/front_capture.launch.py',
    'ros2 launch risabot_v4_control track_test.launch.py',
    '/lib/rclcpp_components/component_container',
))]
if conflicts:
    raise RuntimeError('Existing camera/control stack: ' + repr(conflicts))
directory = Path('/home/sunrise/track_test_run/full_stack')
directory.mkdir(parents=True, exist_ok=True)
command = (
    'source /opt/tros/humble/setup.bash && '
    'source /home/sunrise/risabotcar_ws/install/setup.bash && '
    'export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0 && '
    'exec ros2 launch risabot_v4_control track_test.launch.py '
    'vehicle:=risabot5 motor_duty:=65 steering_gain:=2.8 '
    'minimum_turn_duty:=48 steering_slowdown_gain:=0.85 '
    'enable_reverse_recovery:=false '
    'profile_path:=/home/sunrise/risabot5_profiles/camera_profiles.yaml '
    'dashboard:=true'
)
with (directory / 'launch.log').open('ab') as log:
    process = subprocess.Popen(
        ['bash', '-lc', command], stdin=subprocess.DEVNULL,
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
(directory / 'launch.pid').write_text(str(process.pid) + '\n')
print(json.dumps({
    'pid': process.pid, 'log': str(directory / 'launch.log'),
    'startup_mode': 'MANUAL', 'requested_motor_duty_percent': 65,
    'dashboard': 'http://192.168.137.74:8080',
    'reverse_recovery_enabled': False,
}))
