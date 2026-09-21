#!/usr/bin/env bash
# Record one evidence-complete RISA-Bot track session.
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: record_track_session.sh [options]

Options:
  --name NAME          Short session name (default: track)
  --output-root DIR    Parent directory (default: ~/track_validation)
  --duration SEC       Stop automatically after SEC; otherwise use Ctrl+C
  --min-free-gb GB     Required free space before starting (default: 10)
  --require-v4         Refuse to record unless the core V4 status topics exist
  -h, --help           Show this help

Run after the full stack is started and while the car is in MANUAL.
EOF
}

session_name=track
output_root="$HOME/track_validation"
duration=0
min_free_gb=10
require_v4=false

while (($#)); do
  case "$1" in
    --name) session_name=${2:?missing value for --name}; shift 2 ;;
    --output-root) output_root=${2:?missing value for --output-root}; shift 2 ;;
    --duration) duration=${2:?missing value for --duration}; shift 2 ;;
    --min-free-gb) min_free_gb=${2:?missing value for --min-free-gb}; shift 2 ;;
    --require-v4) require_v4=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! "$session_name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "Session name must contain only letters, digits, dot, underscore or dash." >&2
  exit 2
fi
if ! [[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Duration must be a non-negative number of seconds." >&2
  exit 2
fi
if ! [[ "$min_free_gb" =~ ^[1-9][0-9]*$ ]]; then
  echo "Minimum free space must be a positive whole number of GiB." >&2
  exit 2
fi

command -v ros2 >/dev/null || { echo "ros2 is not available; source ROS first." >&2; exit 3; }
ros2 bag --help >/dev/null 2>&1 || { echo "ros2 bag is not available." >&2; exit 3; }

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
stamp=$(date -u +%Y%m%dT%H%M%SZ)
session_dir="${output_root%/}/${stamp}_${session_name}"
mkdir -p "$output_root"
free_kb=$(df -Pk "$output_root" | awk 'NR==2 {print $4}')
required_kb=$((min_free_gb * 1024 * 1024))
if [[ -z "$free_kb" || "$free_kb" -lt "$required_kb" ]]; then
  echo "At least ${min_free_gb} GiB free is required in $output_root." >&2
  echo "Recording was not started." >&2
  exit 3
fi

mapfile -t live_topics < <(ros2 topic list 2>/dev/null | sort -u)
topic_exists() {
  local wanted=$1 item
  for item in "${live_topics[@]}"; do
    [[ "$item" == "$wanted" ]] && return 0
  done
  return 1
}

required_topics=(
  /camera/color/image_raw
  /scan
  /odom
)
core_v4_topics=(
  /v4_experimental/bev/status
  /v4_experimental/road/status
  /v4_experimental/trajectory/status
  /v4_experimental/arbitration/status
)
candidate_topics=(
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/depth/image_raw
  /camera/depth/camera_info
  /camera/second/image_raw
  /camera/third/image_raw
  /scan
  /odom
  /imu/data
  /imu/rpy
  /imu/pitch
  /joy
  /tf
  /tf_static
  /uwb_fix
  /lane_error
  /lane_lost
  /lane_curvature
  /traffic_light_state
  /traffic_light_confidence
  /boom_gate_open
  /boom_gate_confidence
  /obstacle_front
  /obstacle_detected_camera
  /obstacle_sign_detected
  /obstacle_detected_fused
  /tunnel_detected
  /tunnel_cmd_vel
  /tunnel_confidence
  /obstruction_active
  /obstruction_cmd_vel
  /obstruction_confidence
  /parking_command
  /parking_cmd_vel
  /parking_complete
  /parking_signboard_detected
  /parking_status
  /hill_sign_detected
  /health_status
  /loop_stats
  /dashboard_state
  /record_playback_state
  /auto_mode
  /e_stop
  /cmd_safety_status
  /cmd_vel_auto_raw
  /cmd_vel_v4_raw
  /cmd_vel_auto
  /cmd_vel
  /servo_geometry
  /side_camera/status
  /v4_telemetry
  /v4_experimental/status
  /v4_experimental/bev/status
  /v4_experimental/bev/primary/image
  /v4_experimental/bev/primary/coverage
  /v4_experimental/bev/secondary/image
  /v4_experimental/bev/secondary/coverage
  /v4_experimental/road/status
  /v4_experimental/road/primary/fused
  /v4_experimental/road/secondary/fused
  /v4_experimental/pose/status
  /v4_experimental/pose/local
  /v4_experimental/pose/coarse
  /v4_experimental/uwb/status
  /v4_experimental/uwb/fix
  /v4_experimental/trajectory/status
  /v4_experimental/parking/goal
  /v4_experimental/parking/goal_source_status
  /v4_experimental/parking/status
  /v4_experimental/parking/proposed_path
  /v4_experimental/recovery/request
  /v4_experimental/recovery/request_source_status
  /v4_experimental/recovery/status
  /v4_experimental/recovery/proposed_path
  /v4_experimental/arbitration/status
  /v4_experimental/arbitration/proposed_request
)

missing_required=()
for topic in "${required_topics[@]}"; do
  topic_exists "$topic" || missing_required+=("$topic")
done
if ((${#missing_required[@]})); then
  printf 'Critical topic missing: %s\n' "${missing_required[@]}" >&2
  echo "Recording was not started." >&2
  exit 4
fi

if $require_v4; then
  missing_v4=()
  for topic in "${core_v4_topics[@]}"; do
    topic_exists "$topic" || missing_v4+=("$topic")
  done
  if ((${#missing_v4[@]})); then
    printf 'Required V4 topic missing: %s\n' "${missing_v4[@]}" >&2
    echo "Recording was not started." >&2
    exit 5
  fi
fi

record_topics=()
missing_optional=()
for topic in "${candidate_topics[@]}"; do
  if topic_exists "$topic"; then
    record_topics+=("$topic")
  else
    missing_optional+=("$topic")
  fi
done

# Record each side view once. Prefer the stable relay names consumed by V4;
# fall back to the camera-driver source only when its relay is absent.
if ! topic_exists /camera/second/image_raw && topic_exists /cam_imx219/image_raw; then
  record_topics+=(/cam_imx219/image_raw)
fi
if ! topic_exists /camera/third/image_raw && topic_exists /cam_ov5647/image_raw; then
  record_topics+=(/cam_ov5647/image_raw)
fi

if [[ -e "$session_dir" ]]; then
  echo "Session directory already exists: $session_dir" >&2
  exit 6
fi
mkdir -p "$session_dir/metadata" "$session_dir/params"
printf '%s\n' "${live_topics[@]}" > "$session_dir/metadata/live_topics.txt"
printf '%s\n' "${record_topics[@]}" > "$session_dir/metadata/recorded_topics.txt"
printf '%s\n' "${missing_optional[@]}" > "$session_dir/metadata/missing_optional_topics.txt"

{
  echo "session_name=$session_name"
  echo "started_utc=$(date -u --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "user=$(id -un)"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
  echo "rmw_implementation=${RMW_IMPLEMENTATION:-default}"
  echo "repo_root=$repo_root"
  git -C "$repo_root" rev-parse HEAD 2>/dev/null | sed 's/^/git_commit=/' || true
  git -C "$repo_root" status --short 2>/dev/null | sed 's/^/git_status=/' || true
} > "$session_dir/metadata/session.env"
uname -a > "$session_dir/metadata/uname.txt"
ros2 node list > "$session_dir/metadata/nodes.txt" 2>&1 || true
timeout 30 ros2 doctor --report > "$session_dir/metadata/ros2_doctor.txt" 2>&1 || true
df -h "$output_root" > "$session_dir/metadata/disk_before.txt" 2>&1 || true
shopt -s nullglob
model_files=(/home/sunrise/*.bin "$repo_root"/tools/bpu_model/model_output/*.bin)
if ((${#model_files[@]})); then
  sha256sum "${model_files[@]}" > "$session_dir/metadata/model_sha256.txt" 2>&1 || true
fi
shopt -u nullglob

for camera_info in /camera/color/camera_info /camera/depth/camera_info; do
  if topic_exists "$camera_info"; then
    safe_name=${camera_info#/}; safe_name=${safe_name//\//_}
    timeout 8 ros2 topic echo --once "$camera_info" \
      > "$session_dir/metadata/${safe_name}.yaml" 2>&1 || true
  fi
done

important_nodes=(
  /line_follower_camera
  /auto_driver
  /cmd_safety_controller
  /servo_controller
  /signage_detector
  /traffic_light_detector
  /obstacle_detector_lidar
  /obstacle_detector_camera
  /v4_motion_executor
)
for node in "${important_nodes[@]}"; do
  if grep -Fxq "$node" "$session_dir/metadata/nodes.txt"; then
    safe_name=${node#/}; safe_name=${safe_name//\//_}
    timeout 8 ros2 param dump "$node" \
      > "$session_dir/params/${safe_name}.yaml" 2>&1 || true
  fi
done

if compgen -G "$repo_root/src/*/config/*.yaml" >/dev/null; then
  mkdir -p "$session_dir/config_snapshot"
  while IFS= read -r -d '' config; do
    package=$(basename "$(dirname "$(dirname "$config")")")
    cp "$config" "$session_dir/config_snapshot/${package}_$(basename "$config")"
  done < <(find "$repo_root/src" -path '*/config/*.yaml' -print0)
fi

health_log="$session_dir/system_health.csv"
echo 'utc,temperature_c,load_1m,mem_available_kb,disk_available_kb' > "$health_log"
health_monitor() {
  while true; do
    now=$(date -u --iso-8601=seconds)
    temp=NA
    [[ -r /sys/class/thermal/thermal_zone0/temp ]] && \
      temp=$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp)
    load=$(awk '{print $1}' /proc/loadavg 2>/dev/null || echo NA)
    mem=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo NA)
    disk=$(df -Pk "$output_root" 2>/dev/null | awk 'NR==2 {print $4}')
    echo "$now,$temp,$load,${mem:-NA},${disk:-NA}" >> "$health_log"
    sleep 2
  done
}

bag_pid=''
health_pid=''
duration_pid=''
finished=false
finalize() {
  local rc=$?
  $finished && return
  finished=true
  trap - EXIT INT TERM
  [[ -n "$duration_pid" ]] && kill "$duration_pid" 2>/dev/null || true
  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$bag_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
  fi
  [[ -n "$health_pid" ]] && kill "$health_pid" 2>/dev/null || true
  [[ -n "$health_pid" ]] && wait "$health_pid" 2>/dev/null || true
  echo "finished_utc=$(date -u --iso-8601=seconds)" >> "$session_dir/metadata/session.env"
  df -h "$output_root" > "$session_dir/metadata/disk_after.txt" 2>&1 || true
  ros2 bag info "$session_dir/bag" > "$session_dir/metadata/bag_info.txt" 2>&1 || true
  python3 "$repo_root/tools/verify_track_session.py" "$session_dir" --json \
    > "$session_dir/metadata/verification.json" 2>&1 || true
  (
    cd "$session_dir"
    find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
  )
  echo
  echo "Session saved: $session_dir"
  echo "Events:       $session_dir/events.jsonl"
  echo "Bag summary:  $session_dir/metadata/bag_info.txt"
  exit "$rc"
}
trap finalize EXIT INT TERM

touch "$session_dir/events.jsonl"
health_monitor &
health_pid=$!

echo "Recording ${#record_topics[@]} live topics into: $session_dir"
echo "Mark an event from another terminal with:"
echo "  $repo_root/tools/mark_track_event.sh '$session_dir' EVENT_NAME 'optional note'"
echo "Stop with Ctrl+C. Keep the car in MANUAL while collecting the baseline."

ros2 bag record -o "$session_dir/bag" "${record_topics[@]}" \
  > "$session_dir/rosbag.log" 2>&1 &
bag_pid=$!

if awk -v value="$duration" 'BEGIN {exit !(value > 0)}'; then
  (sleep "$duration"; kill -INT "$bag_pid" 2>/dev/null || true) &
  duration_pid=$!
fi

set +e
wait "$bag_pid"
bag_rc=$?
set -e
if awk -v value="$duration" 'BEGIN {exit !(value > 0)}' \
    && [[ "$bag_rc" == 130 || "$bag_rc" == 143 ]]; then
  exit 0
fi
exit "$bag_rc"
