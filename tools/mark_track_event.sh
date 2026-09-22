#!/usr/bin/env bash
# Add a timestamped event to a track-recording session.
set -euo pipefail

if (($# < 2 || $# > 3)); then
  echo "Usage: mark_track_event.sh SESSION_DIR EVENT_NAME [NOTE]" >&2
  exit 2
fi

session_dir=$1
event_name=$2
note=${3:-}
events_file="$session_dir/events.jsonl"

[[ -d "$session_dir" ]] || { echo "Session directory not found: $session_dir" >&2; exit 3; }
[[ -f "$events_file" ]] || { echo "Not a recording session: $session_dir" >&2; exit 3; }

python3 - "$events_file" "$event_name" "$note" <<'PY'
import datetime
import json
import os
import sys
import time

path, event, note = sys.argv[1:]
event = event.strip()
if not event or len(event) > 80:
    raise SystemExit('EVENT_NAME must contain 1-80 characters')
record = {
    'utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    't_wall': time.time(),
    'event': event,
    'note': note.strip(),
}
line = (json.dumps(record, separators=(',', ':')) + '\n').encode()
fd = os.open(path, os.O_WRONLY | os.O_APPEND)
try:
    os.write(fd, line)
finally:
    os.close(fd)
print(f"marked {record['utc']} {event}")
PY
