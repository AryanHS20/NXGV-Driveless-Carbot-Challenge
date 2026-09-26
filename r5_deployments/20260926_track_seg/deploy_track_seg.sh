#!/usr/bin/env bash
# Run ON THE BOARD from the directory holding these files (scp the whole overlay dir).
# Applies the seg road source as PATCHES onto the board's real files (dry-run first,
# all-or-nothing), so nothing else on the board is overwritten.
# Behaviour stays classical until the launch is started with road_source:=seg.
set -euo pipefail

WS="${WS:-$HOME/risabotcar_ws}"
STAGE="$(cd "$(dirname "$0")" && pwd)"
SRC="$WS/src"
BK="$WS/track_test_backups/pre_track_seg_$(date +%Y%m%d_%H%M%S)"

# file:destination directory under $SRC
PATCHED=(
  "road_mask_core.py:risabot_v4_experimental/risabot_v4_experimental"
  "road_mask_shadow.py:risabot_v4_experimental/risabot_v4_experimental"
  "track_test.launch.py:risabot_v4_control/launch"
  "setup.py:risabot_automode"
)
NEW_FILE="track_seg_node.py:risabot_automode/risabot_automode"

# 1. Dry-run every patch first. Any failure aborts before anything is touched.
for pair in "${PATCHED[@]}"; do
  f="${pair%%:*}"; d="${pair#*:}"
  if ! patch --dry-run -s "$SRC/$d/$f" < "$STAGE/patches/$f.patch" >/dev/null; then
    echo "ABORT: patch for $d/$f does not apply cleanly to the board's file." >&2
    echo "The board differs from the snapshot this overlay was made against; nothing was changed." >&2
    exit 1
  fi
done

# 2. Back up exactly what we are about to change.
mkdir -p "$BK"
: > "$BK/MANIFEST"
for pair in "${PATCHED[@]}" "$NEW_FILE"; do
  f="${pair%%:*}"; d="${pair#*:}"
  if [ -f "$SRC/$d/$f" ]; then
    mkdir -p "$BK/$d"; cp -p "$SRC/$d/$f" "$BK/$d/$f"; echo "existing $d/$f" >> "$BK/MANIFEST"
  else
    echo "new $d/$f" >> "$BK/MANIFEST"
  fi
done
echo "Backup: $BK"

# 3. Apply.
for pair in "${PATCHED[@]}"; do
  f="${pair%%:*}"; d="${pair#*:}"
  patch -s "$SRC/$d/$f" < "$STAGE/patches/$f.patch"
done
install -m 0644 "$STAGE/track_seg_node.py" "$SRC/${NEW_FILE#*:}/track_seg_node.py"
if [ -f "$STAGE/track_seg_512x288_nv12.bin" ]; then
  install -m 0644 "$STAGE/track_seg_512x288_nv12.bin" "$HOME/track_seg_512x288_nv12.bin"
fi

# 4. Build.
cd "$WS"
colcon build --packages-select risabot_automode risabot_v4_control risabot_v4_experimental
echo
echo "Installed. Road source is still 'classical' by default."
echo "Enable:  ros2 launch risabot_v4_control track_test.launch.py road_source:=seg"
echo "Revert:  $STAGE/revert_track_seg.sh $BK"
