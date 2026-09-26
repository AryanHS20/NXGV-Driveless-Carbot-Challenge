#!/usr/bin/env bash
# Run ON THE BOARD. Restores the pre-seg files from a deploy backup and rebuilds.
# Usage: revert_track_seg.sh [BACKUP_DIR]   (default: newest pre_track_seg_* backup)
# Instant, no-rebuild revert instead: launch with road_source:=classical.
set -euo pipefail

WS="${WS:-$HOME/risabotcar_ws}"
SRC="$WS/src"
BK="${1:-$(ls -d "$WS"/track_test_backups/pre_track_seg_* 2>/dev/null | sort | tail -1)}"
[ -n "$BK" ] && [ -f "$BK/MANIFEST" ] || { echo "No backup with a MANIFEST found." >&2; exit 1; }
echo "Restoring from $BK"

while read -r kind path; do
  if [ "$kind" = existing ]; then
    install -m 0644 "$BK/$path" "$SRC/$path"
    echo "restored $path"
  else
    rm -f "$SRC/$path"
    echo "removed  $path"
  fi
done < "$BK/MANIFEST"

cd "$WS"
colcon build --packages-select risabot_automode risabot_v4_control risabot_v4_experimental
echo "Reverted to the pre-seg classical code. Restart risabot5-track-stack.service."
