# UWB integration assets

This folder contains the source needed to reproduce the RISA01 UWB tag and
inspect or calibrate its range data:

- `TagMicroROS/`: current ESP32 + DW1000 micro-ROS firmware. Set the Wi-Fi
  password locally before flashing; the committed configuration contains a
  placeholder and no credential.
- `uwb_xy.py`: three-anchor diagnostic position viewer.
- `uwb_calib.py`: per-anchor range-offset measurement tool.
- `UWB_Handoff.md`: hardware, network, QoS, frame, and calibration notes.
- `evidence/three_anchor_position_demo.mp4`: short recorded three-anchor
  positioning demonstration.

The original ZIP/RAR delivery archives are intentionally ignored because they
duplicate this readable source and make reviews harder.
