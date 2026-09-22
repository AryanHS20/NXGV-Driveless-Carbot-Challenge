#!/usr/bin/env bash
# Build and briefly run Stage 7 in disabled diagnostic mode.
set -o pipefail
workspace=${V4_WORKSPACE:-/home/sunrise/risabotcar_ws}
cd "$workspace" || exit 2
source /opt/tros/humble/setup.bash
colcon build --packages-select risabot_v4_experimental --symlink-install || exit 10
source install/setup.bash
ros2 launch risabot_v4_experimental stage7_arbitration.launch.py enabled:=false \
  > /tmp/v4-stage7-smoke.log 2>&1 &
launch_pid=$!
for _ in 1 2 3 4 5 6 7 8; do
  ros2 topic list 2>/dev/null | grep -qx '/v4_experimental/arbitration/status' && break
  sleep 1
done
timeout 7 ros2 topic echo /v4_experimental/arbitration/status --once
status_rc=$?
kill -TERM "$launch_pid" 2>/dev/null || true
for _ in 1 2 3 4 5; do kill -0 "$launch_pid" 2>/dev/null || break; sleep 1; done
kill -KILL "$launch_pid" 2>/dev/null || true
wait "$launch_pid" 2>/dev/null || true
pattern='/risabot_v4_experimental/lib/risabot_v4_experimental/arbitration_shadow'
child_pids=$(pgrep -f "$pattern" || true)
if [ -n "$child_pids" ]; then
  kill -INT $child_pids 2>/dev/null || true
  sleep 2
fi
child_pids=$(pgrep -f "$pattern" || true)
if [ -n "$child_pids" ]; then
  kill -TERM $child_pids 2>/dev/null || true
  sleep 1
fi
leftover=$(pgrep -f "$pattern" || true)
if [ -n "$leftover" ]; then
  echo "leftover Stage 7 process: $leftover"
  kill -KILL $leftover 2>/dev/null || true
  exit 11
fi
cat /tmp/v4-stage7-smoke.log
exit "$status_rc"
