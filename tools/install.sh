#!/bin/bash
# Canonical installer; do not create a second workspace layout.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/install_deps.sh" "$@"
