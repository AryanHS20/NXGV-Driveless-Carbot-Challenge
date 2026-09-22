#!/usr/bin/env bash
# Build and briefly run Stage 1 in its default disabled, diagnostic-only mode.
set -o pipefail

cd /home/sunrise/risabotcar_ws || exit 2
source /opt/tros/humble/setup.bash

enabled=${1:-false}
if [ "$enabled" != 'false' ] && [ "$enabled" != 'true' ]; then
  echo 'usage: v4_stage1_board_smoke.sh [false|true]'
  exit 2
fi

colcon build --packages-select risabot_v4_experimental --symlink-install || exit 10
source install/setup.bash

ros2 launch risabot_v4_experimental stage1_bev.launch.py enabled:="$enabled" \
  > /tmp/v4-stage1-smoke.log 2>&1 &
launch_pid=$!

for _ in 1 2 3 4 5 6 7 8; do
  if ros2 topic list 2>/dev/null | grep -qx '/v4_experimental/bev/status'; then
    break
  fi
  sleep 1
done

timeout 7 ros2 topic echo /v4_experimental/bev/status --once
status_rc=$?

kill -TERM "$launch_pid" 2>/dev/null || true
for _ in 1 2 3 4 5; do
  kill -0 "$launch_pid" 2>/dev/null || break
  sleep 1
done
kill -KILL "$launch_pid" 2>/dev/null || true
wait "$launch_pid" 2>/dev/null || true

child_pids=$(pgrep -f \
  '/risabot_v4_experimental/lib/risabot_v4_experimental/bev_shadow' || true)
if [ -n "$child_pids" ]; then
  kill -INT $child_pids 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    pgrep -f '/risabot_v4_experimental/lib/risabot_v4_experimental/bev_shadow' \
      >/dev/null || break
    sleep 1
  done
fi

child_pids=$(pgrep -f \
  '/risabot_v4_experimental/lib/risabot_v4_experimental/bev_shadow' || true)
if [ -n "$child_pids" ]; then
  kill -TERM $child_pids 2>/dev/null || true
  sleep 2
fi

leftover=$(pgrep -f \
  '/risabot_v4_experimental/lib/risabot_v4_experimental/bev_shadow' || true)
if [ -n "$leftover" ]; then
  echo "leftover Stage 1 process: $leftover"
  kill -KILL $leftover 2>/dev/null || true
  exit 11
fi

echo '--- launch log ---'
cat /tmp/v4-stage1-smoke.log
exit "$status_rc"
