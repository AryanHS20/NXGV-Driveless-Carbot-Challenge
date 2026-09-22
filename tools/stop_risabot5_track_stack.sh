#!/usr/bin/env bash
set -euo pipefail
if sudo -n systemctl is-active --quiet risabot5-track-stack.service 2>/dev/null; then
  sudo -n systemctl stop risabot5-track-stack.service
  exit 0
fi
pid_file=/home/sunrise/track_test_run/full_stack/launch.pid
if [[ ! -f "$pid_file" ]]; then
  exit 0
fi
pid=$(cat "$pid_file")
if [[ ! "$pid" =~ ^[0-9]+$ ]]; then
  echo "invalid launch pid: $pid" >&2
  exit 1
fi
if kill -0 "$pid" 2>/dev/null; then
  kill -INT -- "-$pid"
  for _ in $(seq 1 50); do
    kill -0 "$pid" 2>/dev/null || exit 0
    sleep 0.1
  done
  kill -TERM -- "-$pid"
fi
