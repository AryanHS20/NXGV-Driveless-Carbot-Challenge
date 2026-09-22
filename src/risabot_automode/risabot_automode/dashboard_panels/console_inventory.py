"""Mockup control inventory embedded for installed-package availability.

Generated from console_control_inventory.json; parity is tested.
"""
import json

INVENTORY = json.loads(r'''{
  "screens": [
    {
      "buttons": [
        {
          "disabled": true,
          "label": "START",
          "navigation": null
        }
      ],
      "fields": [],
      "id": "drive",
      "title": "Drive"
    },
    {
      "buttons": [],
      "fields": [],
      "id": "map",
      "title": "Global map"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Road",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Parking",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Recovery",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "Stitched footage",
          "type": "checkbox",
          "value": "on"
        },
        {
          "label": "Road mask",
          "type": "checkbox",
          "value": "on"
        },
        {
          "label": "Final drivable edge",
          "type": "checkbox",
          "value": "on"
        },
        {
          "label": "Camera seams",
          "type": "checkbox",
          "value": "on"
        },
        {
          "label": "Rejected candidates",
          "type": "checkbox",
          "value": "on"
        },
        {
          "label": "Selected path",
          "type": "checkbox",
          "value": "on"
        },
        {
          "label": "Corridor guide",
          "type": "checkbox",
          "value": "on"
        }
      ],
      "id": "percplan",
      "title": "Perception + planner"
    },
    {
      "buttons": [],
      "fields": [],
      "id": "memory",
      "title": "Memory + LiDAR"
    },
    {
      "buttons": [],
      "fields": [],
      "id": "loc",
      "title": "Localization"
    },
    {
      "buttons": [],
      "fields": [],
      "id": "det",
      "title": "Detections"
    },
    {
      "buttons": [],
      "fields": [],
      "id": "control",
      "title": "Control + safety"
    },
    {
      "buttons": [],
      "fields": [],
      "id": "health",
      "title": "System health"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "All",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Mission",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Safety",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Warnings",
          "navigation": null
        }
      ],
      "fields": [],
      "id": "events",
      "title": "Events + log"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Revert to saved",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Apply live",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Save to session",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "candidate_count",
          "type": "text",
          "value": "15"
        },
        {
          "label": "min_clearance_m",
          "type": "text",
          "value": "0.060"
        },
        {
          "label": "lookahead_m",
          "type": "text",
          "value": "0.35"
        }
      ],
      "id": "tuning",
      "title": "Tuning"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-1"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-2"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-3"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-4"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-5"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-6"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-7"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-8"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-9"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-10"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-11"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-12"
        },
        {
          "disabled": false,
          "label": "Open",
          "navigation": "cal-13"
        },
        {
          "disabled": false,
          "label": "Roll back",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Roll back",
          "navigation": null
        }
      ],
      "fields": [],
      "id": "cal-overview",
      "title": "Calibration overview"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-overview"
        },
        {
          "disabled": false,
          "label": "Next step",
          "navigation": "cal-2"
        },
        {
          "disabled": false,
          "label": "Restart camera driversKills stale mipi_cam, codec and websocket nodes, then restarts them",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Run check",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [],
      "id": "cal-1",
      "title": "Step 1 · Sensor health check"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-1"
        },
        {
          "disabled": false,
          "label": "Next step",
          "navigation": "cal-3"
        },
        {
          "disabled": false,
          "label": "Swap left and rightUse if you covered the wrong side",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Start",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "Assigned rolesAstra Pro → front",
          "type": "select-one",
          "value": "Astra Pro → front"
        }
      ],
      "id": "cal-2",
      "title": "Step 2 · Camera identity"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Front ✓",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Left rear ✓",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Right rear",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-2"
        },
        {
          "disabled": true,
          "label": "Next step",
          "navigation": "cal-4"
        },
        {
          "disabled": false,
          "label": "Capture frame",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Delete last frame",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Compute",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Keep previous valueNo previous value exists: this step must pass",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "Auto-capture when the board is still and sharp",
          "type": "checkbox",
          "value": "on"
        }
      ],
      "id": "cal-3",
      "title": "Step 3 · Camera intrinsics"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-3"
        },
        {
          "disabled": true,
          "label": "Next step",
          "navigation": "cal-5"
        },
        {
          "disabled": false,
          "label": "Show expected cornersDraws where each corner should appear",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "Mat definitionA0 mat v2 (24 corners)",
          "type": "select-one",
          "value": "A0 mat v2 (24 corners)"
        }
      ],
      "id": "cal-4",
      "title": "Step 4 · 3-camera extrinsics + IPM"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-4"
        },
        {
          "disabled": true,
          "label": "Next step",
          "navigation": "cal-6"
        },
        {
          "disabled": false,
          "label": "Run",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "Box distance (m)",
          "type": "number",
          "value": "0.50"
        }
      ],
      "id": "cal-5",
      "title": "Step 5 · LiDAR–camera alignment"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-5"
        },
        {
          "disabled": true,
          "label": "Next step",
          "navigation": "cal-7"
        },
        {
          "disabled": false,
          "label": "Calibrate IMU hardwareSends /imu/calibrate. Keep the car still.",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Run",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "Measured distance (m)",
          "type": "number",
          "value": "2.00"
        }
      ],
      "id": "cal-6",
      "title": "Step 6 · IMU + wheel odometry"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-6"
        },
        {
          "disabled": true,
          "label": "Next step",
          "navigation": "cal-8"
        },
        {
          "disabled": false,
          "label": "Centre",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Nudge left",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Nudge right",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Find left limit",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Find right limit",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Save limits",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "Centre trim (base angular.z)",
          "type": "range",
          "value": "0.012"
        }
      ],
      "id": "cal-7",
      "title": "Step 7 · Servo centre + steering limits"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-7"
        },
        {
          "disabled": true,
          "label": "Next step",
          "navigation": "cal-9"
        },
        {
          "disabled": false,
          "label": "Run step test",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "kp",
          "type": "number",
          "value": "0.80"
        },
        {
          "label": "ki",
          "type": "number",
          "value": "0.20"
        },
        {
          "label": "kd",
          "type": "number",
          "value": "0.00"
        },
        {
          "label": "Step target (m/s)",
          "type": "number",
          "value": "0.40"
        }
      ],
      "id": "cal-8",
      "title": "Step 8 · Speed PID"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-8"
        },
        {
          "disabled": true,
          "label": "Next step",
          "navigation": "cal-10"
        },
        {
          "disabled": false,
          "label": "Sample road",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Sample line",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Save thresholds",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "Luma threshold",
          "type": "range",
          "value": "142"
        },
        {
          "label": "Chroma max",
          "type": "range",
          "value": "18"
        }
      ],
      "id": "cal-9",
      "title": "Step 9 · Venue colour + lighting"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-9"
        },
        {
          "disabled": false,
          "label": "Next step",
          "navigation": "cal-11"
        },
        {
          "disabled": false,
          "label": "Record point",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": null,
          "type": "select-one",
          "value": "K1"
        }
      ],
      "id": "cal-10",
      "title": "Step 10 · UWB anchor survey + offsets"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Full build (new venue)",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Quick re-align (points)",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-10"
        },
        {
          "disabled": true,
          "label": "Next step",
          "navigation": "cal-12"
        },
        {
          "disabled": false,
          "label": "Record lap",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Stop lap",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Auto-fitMoves and rotates the whole map onto the lap",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "RefitKeeps the handles you dragged",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Reset selected handle",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Discard lap",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save map",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [],
      "id": "cal-11",
      "title": "Step 11 · Build map from a lap"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "P0 ✓",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "P1 ✓",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "P2 ✓",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "P3",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Review",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-11"
        },
        {
          "disabled": true,
          "label": "Next step",
          "navigation": "cal-13"
        },
        {
          "disabled": false,
          "label": "−5°",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "−1°",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "+1°",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "+5°",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "FlipNose-in ↔ reverse-in",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Reset pose",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save mission",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Keep previous value20260921_190455 (passed)",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "Snap to lane centre, or centred in a bay",
          "type": "checkbox",
          "value": "on"
        }
      ],
      "id": "cal-12",
      "title": "Step 12 · Mission planner"
    },
    {
      "buttons": [
        {
          "disabled": false,
          "label": "Previous",
          "navigation": "cal-12"
        },
        {
          "disabled": false,
          "label": "Run practice",
          "navigation": null
        },
        {
          "disabled": true,
          "label": "Save",
          "navigation": null
        },
        {
          "disabled": false,
          "label": "Redo",
          "navigation": null
        }
      ],
      "fields": [
        {
          "label": "Challenge1 · Lane change3 · Tunnel7 · Traffic light10 · Parallel parking",
          "type": "select-one",
          "value": "1 · Lane change"
        }
      ],
      "id": "cal-13",
      "title": "Step 13 · Practice runs"
    }
  ],
  "shellButtons": [
    {
      "id": "",
      "label": "08Mission · ROAD · leg 2 of 3",
      "navigation": "map"
    },
    {
      "id": "",
      "label": "09–12Corridor + local planner · #7",
      "navigation": "percplan"
    },
    {
      "id": "",
      "label": "15Command owner · ROAD",
      "navigation": "control"
    },
    {
      "id": "",
      "label": "08Speed zone bump · 0.35 m/s",
      "navigation": "map"
    },
    {
      "id": "",
      "label": "14Safety · front_clearance 0.12 m < 0.15 m",
      "navigation": "control"
    },
    {
      "id": "",
      "label": "08Mission · TRAFFIC HOLD, light RED",
      "navigation": "det"
    },
    {
      "id": "",
      "label": "12Local planner · waiting on hold",
      "navigation": "percplan"
    },
    {
      "id": "",
      "label": "15Command owner · winner SAFETY_STOP",
      "navigation": "control"
    },
    {
      "id": "",
      "label": "Step 3Camera intrinsics · capturing right rear, 17 of 25 frames",
      "navigation": "cal-3"
    },
    {
      "id": "manualBtn",
      "label": "Manual controlCounts as manual intervention = 0 marksDrive with the controller Hand backReturn to autonomous control"
    },
    {
      "id": "",
      "label": "E-STOPCounts as manual intervention = 0 marks"
    },
    {
      "id": "",
      "label": "STOP MOTORSCancels the running step"
    },
    {
      "id": "splitbtn",
      "label": "Split view"
    },
    {
      "id": "",
      "label": "Drive",
      "tab": "drive"
    },
    {
      "id": "",
      "label": "1Global map",
      "tab": "map"
    },
    {
      "id": "",
      "label": "2Perception + planner",
      "tab": "percplan"
    },
    {
      "id": "",
      "label": "3Memory + LiDAR",
      "tab": "memory"
    },
    {
      "id": "",
      "label": "4Localization",
      "tab": "loc"
    },
    {
      "id": "",
      "label": "5Detections",
      "tab": "det"
    },
    {
      "id": "",
      "label": "6Control + safety",
      "tab": "control"
    },
    {
      "id": "",
      "label": "7System health",
      "tab": "health"
    },
    {
      "id": "",
      "label": "8Events + log",
      "tab": "events"
    },
    {
      "id": "tkCancel",
      "label": "Keep autonomous"
    },
    {
      "id": "tkGo",
      "label": "Take control, 0 marks"
    },
    {
      "id": "",
      "label": "calibrate.launch",
      "mode": "calibrate"
    },
    {
      "id": "",
      "label": "race.launch",
      "mode": "race"
    },
    {
      "id": "",
      "label": "Driving",
      "scenario": "drive"
    },
    {
      "id": "",
      "label": "Stopped",
      "scenario": "stop"
    }
  ]
}''')
