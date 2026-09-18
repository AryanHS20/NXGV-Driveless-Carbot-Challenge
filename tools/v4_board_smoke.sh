#!/usr/bin/env bash
# Build and briefly run the isolated V4 monitor in its default disabled mode.
set -o pipefail

cd /home/sunrise/risabotcar_ws || exit 2
source /opt/tros/humble/setup.bash

colcon build --packages-select risabot_v4_experimental --symlink-install || exit 10
source install/setup.bash

ros2 launch risabot_v4_experimental shadow.launch.py \
  > /tmp/v4-shadow-smoke.log 2>&1 &
launch_pid=$!

for _ in 1 2 3 4 5 6 7 8; do
  if ros2 topic list 2>/dev/null | grep -qx '/v4_experimental/status'; then
    break
  fi
  sleep 1
done

timeout 7 ros2 topic echo /v4_experimental/status --once
status_rc=$?

kill -TERM "$launch_pid" 2>/dev/null || true
for _ in 1 2 3 4 5; do
  kill -0 "$launch_pid" 2>/dev/null || break
  sleep 1
done
kill -KILL "$launch_pid" 2>/dev/null || true
wait "$launch_pid" 2>/dev/null || true

# A launch-process failure must never leave its experimental child behind.
child_pids=$(pgrep -f \
  '/risabot_v4_experimental/lib/risabot_v4_experimental/shadow_monitor' || true)
if [ -n "$child_pids" ]; then
  kill -TERM $child_pids 2>/dev/null || true
  sleep 1
fi

leftover=$(pgrep -f \
  '/risabot_v4_experimental/lib/risabot_v4_experimental/shadow_monitor' || true)
if [ -n "$leftover" ]; then
  echo "leftover shadow process: $leftover"
  kill -TERM $leftover 2>/dev/null || true
  exit 11
fi

echo '--- launch log ---'
cat /tmp/v4-shadow-smoke.log
exit "$status_rc"
