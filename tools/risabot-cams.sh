#!/usr/bin/bash
# Root-run dual MIPI camera service for the RDK X5.
#
# The two VSE pipelines must not initialize simultaneously: doing so causes
# intermittent ret -217 failures or a live ROS publisher with no frames.
# Starting OV5647 first and allowing it to settle before IMX219 was verified
# on risabot1 with both streams sustaining about 31 FPS.

source /opt/tros/humble/setup.bash

OV_PID=''
IMX_PID=''

cleanup() {
    [ -z "$OV_PID" ] || kill -TERM "$OV_PID" 2>/dev/null || true
    [ -z "$IMX_PID" ] || kill -TERM "$IMX_PID" 2>/dev/null || true
    wait 2>/dev/null || true
}

trap 'exit 0' INT TERM
trap cleanup EXIT

/opt/tros/humble/lib/mipi_cam/mipi_cam \
    --ros-args -r __ns:=/cam_ov5647 \
    -p channel:=2 -p image_width:=960 -p image_height:=544 \
    --log-level warn &
OV_PID=$!

# The RDK X5 camera/VSE initialization is not safe when both hosts start at
# once. Three seconds was sufficient in repeated hardware checks.
sleep 3
if ! kill -0 "$OV_PID" 2>/dev/null; then
    wait "$OV_PID" 2>/dev/null || true
    exit 1
fi

/opt/tros/humble/lib/mipi_cam/mipi_cam \
    --ros-args -r __ns:=/cam_imx219 \
    -p channel:=0 -p image_width:=960 -p image_height:=544 \
    -p rotation:=180.0 \
    --log-level warn &
IMX_PID=$!
# NOTE: IMX219 (right-back) is mounted upside-down; rotation 180 corrects it.
# Remove the -p rotation line if the mount is ever fixed.

# If either camera exits, clean up the other. systemd restarts the pair.
wait -n "$OV_PID" "$IMX_PID"
exit 1
