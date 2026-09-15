#!/bin/bash
# ============================================================
# RISA-Bot Autostart Setup (Tailscale Edition)
# Configures the robot to auto-launch all ROS nodes on boot.
#
# Usage (Run on the robot):
#   sudo bash tools/setup_autostart.sh
# ============================================================

set -e

echo "🤖 RISA-Bot Autostart Setup"
echo "============================"

# ---- 1. Create the startup script ----
echo "📝 Creating startup script..."

cat > /usr/local/bin/risabot-launch.sh << 'LAUNCH_EOF'
#!/bin/bash
# RISA-Bot startup script — called by systemd
set -e

export HOME=/home/sunrise
export USER=sunrise

# Source ROS2 environment (check common paths)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace
WS=/home/sunrise/risabotcar_ws
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

# Wait for hardware to be ready
sleep 5

# One launch graph owns all hardware.
exec ros2 launch risabot_automode bringup.launch.py
LAUNCH_EOF

chmod +x /usr/local/bin/risabot-launch.sh
echo "  ✅ Startup script created at /usr/local/bin/risabot-launch.sh"

# ---- 2. Create systemd service ----
echo "📝 Creating systemd service..."

cat > /etc/systemd/system/risabot.service << 'SERVICE_EOF'
[Unit]
Description=RISA-Bot ROS2 Autostart
After=network-online.target tailscaled.service
Wants=network-online.target tailscaled.service

[Service]
Type=simple
User=sunrise
Group=sunrise
Environment="HOME=/home/sunrise"
ExecStart=/usr/local/bin/risabot-launch.sh
KillMode=control-group
KillSignal=SIGINT
Restart=on-failure
RestartSec=10
TimeoutStartSec=60

[Install]
WantedBy=multi-user.target
SERVICE_EOF

# Reload systemd and enable
systemctl daemon-reload
systemctl enable risabot.service

echo "  ✅ Service 'risabot' created and enabled"

echo ""
echo "============================================"
echo "✅ AUTOLOAD SETUP COMPLETE!"
echo "============================================"
echo "The robot will now auto-start all ROS nodes whenever it boots."
echo ""
echo "Helpful Commands:"
echo "  Start manually (now):  sudo systemctl start risabot"
echo "  Stop the robot:        sudo systemctl stop risabot"
echo "  Check status:          sudo systemctl status risabot"
echo "  View live logs:        sudo journalctl -u risabot -f"
echo "============================================"
