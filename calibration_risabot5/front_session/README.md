# Front-only capture session, 2026-09-22

**Current status: front ground-plane calibration reviewed and staged locally at
native320x240; secondary remains uncalibrated.** See `current_status.json` and
`final_front_calibration.json` for the authoritative final state. Earlier
`pre_adjustment_status.json` and `car_movement_report.json` are historical events,
superseded by the user's explicit realignment confirmation and new captures.

## Final confirmed-alignment captures

The user clarified that the Astra lens is physically offset within the housing;
no camera mount movement was confirmed. The user did report moving the car right,
then explicitly confirmed rear-wheel hub alignment to REAR AXLE and centreline
alignment to FORWARD again. `realignment_confirmed.json` preserves this reply.
Only the later `aligned_current320` and `aligned_current640` frames support the
final calibration. Primary profile SHA256:
`fe599255c1ebdb6c03e50f216f534db1a7e29e7795bad179d38d4fe771326a99`.

| Final capture | Detection | 20 held-out RMS / max | Extra sheet-reference max |
|---|---|---:|---:|
| aligned_current320 | Native failed; crop[5,135,315,52], cubic x1/y4; inverse map to original pixels | 2.230 / 4.855mm | 7.509mm |
| aligned_current640 | Native robust6x4,24 corners | 1.359 / 2.404mm | 7.803mm |

The two extra references are the far endpoint of the printed centreline and the
FORWARD arrow apex, outside the checkerboard. `validate_confirmed_sheet.py`
extracts their actual PDF drawing positions relative to the printed axle origin:
`[0.475,0]m` and `[0.398,0]m`. The user verified both print scale bars and alignment.
Their raw pixels were manually observed from the actual images, not projected
from the fitted H or optimized to expected results; annotations and enlarged
nearest-neighbour details are retained. Perturbing each selected native320 pixel
by up to1px in each direction still keeps errors below9.573mm. See
`sheet_reference_validation.json` for all per-point results and PDF coordinates.

These features are independent of the four fitting pixels and24 checkerboard
detections, but share the sheet scale/origin assumptions. They are **not a separate
tape survey**. The checked region is approximately0.398-0.695m forward and
plus/minus0.125m lateral. No claim is made that this alone validates the full
output BEV bounds, full-frame lens distortion or the entire track. No additional
physical input is required for this local sheet calibration. The main task owns
deployment and stationary live-track verification, including the user's requested
BEV/coverage/candidate/connected/fused dashboard strip.

The final320 profile uses actual native320 K,D and raw-pixel correspondences.
Observed current640-to320 board corner agreement is0.534px RMS/0.912px maximum;
this supports local mode consistency without assuming equivalence everywhere.
All tool-generated candidate files remain `calibrated:false`; only the reviewed
repository R5 profile was deliberately set to `primary.calibrated:true`.

Recompute a final fit without modifying the reviewed profile:

```powershell
python -B tools/calibrate_risabot5_camera.py fit calibration_risabot5/front_session/aligned_current320/validated_session.json --output calibration_risabot5/front_session/aligned_current320/replay_validated_fit
```

28 tests passed after final staging. The capture clock issue described below
affects only the early mode320 image, not either aligned_current capture.

## Earlier audit captures (history)

User confirmed rear axle and centreline alignment, and measured the printed
1000mm/800mm bars as correct. `sheet_alignment_confirmed.json` preserves the
exact replies and PDF hash. The front board coordinates derive from this verified
sheet: centre [0.620,0]m, yaw90, 6x4 inner corners, square50mm. No separate survey
precision or physical corner-to-axle measurements were supplied.

The main task owns robot operations and supplied native320 and native640 images,
each with complete same-session CameraInfo and provenance. It reported sensor
serial17121320119, color-only15Hz, no actuator node and no car/sheet/mount movement
between these two captures. Image/CameraInfo timestamps differ by70.15ms at320 and
69.01ms at640. The320 board clock was reset to2000-01-01; the main task corrected
it before640. Preserve original timestamps rather than treating the320 date as
actual capture date. The actual session is2026-09-22.

## Preliminary numerical findings

| Capture | Detection | Held-out RMS | Held-out maximum |
|---|---|---:|---:|
| mode320 | native failed; ROI[15,135,305,52], cubic x1/y4 recovered24, inverse-mapped to original pixels | 2.343mm | 3.683mm |
| mode640 | native robust6x4 succeeded,24 corners | 1.856mm | 3.517mm |

Four perimeter indices0,5,23,18 construct each H;20 other corners are held out.
Visual inspection of each numbered overlay supported flip_cols for yaw90:
far-left,far-right,near-right,near-left perimeter in vehicle coordinates. Both
`board_session.json` files preserve this reasoning, actual image hashes and
intrinsic/measurement provenance. Commands:

```powershell
python -B tools/calibrate_risabot5_camera.py fit calibration_risabot5/front_session/mode320/board_session.json --board-only --output calibration_risabot5/front_session/mode320/replay_fit
python -B tools/calibrate_risabot5_camera.py fit calibration_risabot5/front_session/mode640/board_session.json --board-only --output calibration_risabot5/front_session/mode640/replay_fit
```

Exit2 is expected: independent validation is incomplete. Outputs retain
`calibrated:false`, `geometry_pass:false`, `independent_markers:null`. The main
repository R5 profile remains empty and uncalibrated. The320 fit uses pixels and
K from actual native320, not an assumed camera-mode conversion.

`cross_mode_check.json` compares the actual same-scene detections. After
pixel-centre640-to320 scaling, they differ0.594px RMS/0.841px maximum at320.
Scaled K640 exactly matches published K320. Projecting scaled640 corners through
the native320 fit gives3.736mm RMS/6.003mm maximum against the same board geometry.
This checks agreement around the board, not physical independence or full-field
camera-mode equivalence. The native320 BEV was visually inspected; board squares
are approximately rectangular. No independent straight-line/coverage acceptance
has been recorded.

## Intrinsic provenance limitation

These are now actual R5 **driver-published** K,D messages, not values copied from
another car. Both publish five zero distortion coefficients. However, both P
matrices have principal-point entries inconsistent with K: P[2]=0 and P[6]=fy.
The local checked-in Astra `src/utils.cpp:getDefaultCameraInfo` can generate
centred K with zero D using stream FOV, and its P construction differs from the
observed message. Consequently the published values alone do not establish
individual factory lens calibration or identify the deployed driver branch.
The main task was asked to preserve driver identity/startup logs; no driver source
was modified by this calibration task. Raw-image `bev_core` uses K,D, not P, so
the preliminary planar fit can be evaluated conditional on K,D while this remains
open. Independent ground and straight-line validation are still required.

## Earlier physical-placement uncertainty (superseded)

The main task subsequently reported that the user saw a left-tilted camera and
clipped board and was advised to centre and secure the camera. The user must
finish that adjustment before the main task captures again into distinct
directories. `pre_adjustment_status.json` marks these old fits as audit-only.
User-confirmed sheet scale/alignment is retained unless the sheet/car moves.
Two independent measured floor crosses were initially requested; no coordinates
were supplied. The final workflow instead used the verified-sheet guide features
documented above. Do not invent tape measurements or label the printed-sheet
references as an independent survey.
