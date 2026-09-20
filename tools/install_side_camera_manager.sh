#!/usr/bin/env bash
# Install/update the demand-controlled root camera service on the RDK X5.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Run with sudo: sudo bash tools/install_side_camera_manager.sh" >&2
    exit 2
fi

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace=/home/sunrise/risabotcar_ws
test -x "$repo_dir/tools/risabot-cams.sh"
test -f "$workspace/install/setup.bash"

install -m 0755 "$repo_dir/tools/risabot-cams.sh" /usr/local/bin/risabot-cams.sh
install -m 0644 "$repo_dir/tools/risabot-cams.service" /etc/systemd/system/risabot-cams.service
systemctl daemon-reload
systemctl enable risabot-cams.service
systemctl restart risabot-cams.service
systemctl --no-pager --full status risabot-cams.service
