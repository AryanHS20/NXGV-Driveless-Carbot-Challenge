# Risabot 5 (car 2): camera calibration preparation

Prepared and updated 2026-09-22. **The front ground-plane profile is now calibrated
locally at native320x240; secondary remains uncalibrated.** The initial audit
below is preserved as history; the final measured handoff is
`calibration_risabot5/front_session/final_front_calibration.json`. This task
does not select a profile in launch/central parameters or deploy it. Existing
car 1 calibration is untouched. SSH was used only after the user authorized it,
for file reads and a bounded metadata subscription; no robot files, driver
parameters, running application processes, or motion commands were changed.

Exact profile for the integrating task:

`C:\Users\Aryan\OneDrive\Desktop\Competition\NXGV-Driveless-Carbot-Challenge\src\risabot_v4_experimental\config\risabot5_camera_profiles.yaml`

Both `primary` and `secondary` are present in the version-1 `bev_core` schema.
Primary now has measured native320 pixels and same-session R5 driver CameraInfo,
with `calibrated:true`. Secondary has `calibrated:false` and empty K,D and points.
Its480x272 resolution and sensor role remain unverified defaults. Output BEV
bounds and200px/m are layout choices, not claims of validation over the full area.
Primary's20 held-out corners give2.230mm RMS/4.855mm maximum; two additional
printed-sheet guide features give7.509mm maximum. Those features are independent
of board fitting pixels but share the sheet scale/origin assumptions; no separate
tape survey or full-field intrinsic calibration is claimed. The raw pipeline uses
K,D and does not consume the anomalous P matrix in the captured CameraInfo.

No additional physical input is needed for this local sheet calibration. The
main task owns deployment and stationary live-track verification, including the
requested five-panel BEV/coverage/candidate/connected/fused dashboard. Leave the
camera mount unchanged. A changed mount requires another ground calibration.

## What the audit establishes

Sources read: `camera_profiles.yaml`, `bev_core.py`, `TRACK_AUTONOMY_TODO.md`,
`HANDOVER_CALIBRATION_2-4.md`, bundled `calib_core.py`, `camera_model.py`, intrinsic
utilities/configuration and `make_boards.py`; parent-folder capture/fit/diagnostic
scripts, saved report and captures. SHA-256s are in
`calibration_risabot5/input_manifest.json`. Original parent-file bytes were saved
under `calibration_risabot5/inputs/`; script copies have `.txt` suffixes.

### Car 1: usable method, incomplete historical reproduction

- `bev_core` undistorts **raw-image** calibration pixels with K and D, constructs
  a four-point homography and warps the undistorted frame. Fisheye/equidistant
  coefficients cannot be inserted into its ordinary `cv2.undistort` path.
- The bundled floor target has **6 columns x 4 rows of inner corners** (24),
  **7 x 5 squares**. At the design square size 50 mm, the printed checker area is
  350 x 250 mm and the outer-inner-corner span is 250 x 150 mm. The separate
  intrinsics target is 9 x 6 inner corners, design squares 25 mm. These design
  dimensions do not establish the scale of an actual print.
- Origin is the rear-axle midpoint projected onto the ground, +x forward, +y left,
  +z up. Board centre is the centre of the inner-corner grid, coincident with the
  checker area's centre, not the paper edge or a car nose. Yaw is the directed
  column-axis angle counterclockwise from +x. The bundled default front centre
  `[0.62, 0]` differs from car 1's recorded `[0.765, 0]`; neither belongs to R5.
- The old pose fit tested identity, rot180, flip_rows and flip_cols, rejected
  under-floor poses and chose the candidate closest to a nominal mount. Its
  nominal position and angles are car 1 inputs, not transferable R5 evidence.
- Four perimeter indices `[0, 5, 23, 18]` fit H. The remaining 20 corners check
  interpolation. Car 1's saved claim is 11.804 mm RMS / 19.142 mm maximum.
  Those interior points share the same board pose/scale assumptions; they cannot
  independently prove rear-axle alignment, board scale or physical ordering.
- Historical K is `fx=fy=285.17110237076486`, `cx=159.5`, `cy=119.5` at 320x240,
  with five zero distortion coefficients. Local notes attribute it to factory
  CameraInfo in `/home/sunrise/track_validation/20260921T090557Z_full_course_raw/metadata/camera_color_camera_info.yaml`
  and a subsequent restart check. The extraction script instead deserializes the
  first CameraInfo in that bag and writes `astra_factory_camera_info.json`.
  Neither underlying CameraInfo artifact is available locally; these are
  documented provenance claims, not a freshly verified message or lens identity.
- `_tmp_fit_front_floor.py` currently loads the misleadingly named
  `front_floor_sheet_final_640x480.png` using 320x240 K and directly copies its
  corner pixels. Its comment and the profile's comment describe 640-to-320
  conversion, but the current script does **not** perform that conversion.
- That PNG and its `_new` copy are byte-identical **320x240** files. The saved
  annotated image is **640x480**, has different scene framing, and the report
  contains only the four construction pixels, not the full 24 detections.
  Native detection fails on the current 320x240 source, so the saved fit is not
  reproducible from the current script/input pair.

### Actual detector investigation (OpenCV 4.12.0, NumPy 2.2.6)

The native test invokes the bundled robust detector unchanged: normalized,
exhaustive/accurate SB, then adaptive-threshold classic with subpixel refinement.

| Input in parent folder | Actual pixels | Native 6x4 result |
|---|---:|---|
| `front_floor_sheet_20260921.png` | 320x240 | failed |
| `front_floor_sheet_640x480_20260921.png` | 640x480 | 24 corners |
| `front_floor_sheet_final_640x480.png` and `_new` | 320x240 | failed |
| `front_floor_sheet_moved25_320x240.png` | 320x240 | failed |
| `front_floor_sheet_moved25_640x480.png` | 640x480 | failed |
| `front_floor_sheet_detected_640x480.png` | 640x480 | failed; already annotated |
| `r5_front_floor_640x480.png` | 640x480 | 24 corners |
| `r5_front_floor_640x480_new.png` | 640x480 | failed |
| `r5_burst_0.png`, `_1.png`, `_2.png` | 640x480 | failed |
| `r5_front_detected_640x480.png` | 640x480 | 24 corners; already annotated, exclude from fitting |

Evidence: `native_detection/detections.json` and numbered raw-pixel overlays.
The older small board has only about 3.38 px minimum adjacent-row corner distance.
The successful original R5 capture has about 15.40 px; burst recovery has about
7.43-7.47 px. These are Euclidean corner spacings, not square heights.

Recovery experiments, recorded separately:

- Old image: crop `(x=35,y=125,w=240,h=41)`, cubic resize x1/y4 -> 24 corners.
  In exploratory tests, this same crop failed at x1/y1, x2/y2 and x4/y4 but
  succeeded at x2/y6. This supports severe foreshortening as a factor; it does
  not prove a unique detector failure cause.
- Each burst and the newer R5 image: crop `(60,260,565,65)`, cubic resize x2/y2
  -> 24 corners. The burst-0 crop without enlargement failed in exploration.
- Pixels are mapped back by `(p_processed + 0.5)/scale - 0.5 + crop_offset`.
  The actual rounded resize dimensions determine scale. Detection overlays were
  visually inspected on original pixels. No checker texture was painted in or
  synthesized. Parent `test_cv3.py` paints a white rectangle and must not be used
  as calibration evidence.
- Recovered old perimeter pixels differ from the saved report by
  **5.55-9.88 raw pixels**. Replaying the old assumptions on these *new*
  detections gives 1.959 mm RMS / 4.140 mm maximum, not the saved result. This is
  a diagnostic comparison, not a replacement calibration or physical accuracy claim.

### SSH evidence from `sunrise@192.168.137.74`

Hostname verified as `risabot5`. `remote_snapshot/` contains timestamped read-only
copies of capture scripts, driver parameters and both existing board-local R5
profile files. Those profile files are identical and already claim calibrated
primary at 320x240, 50 mm squares, centre `[0.55,0]`, zero distortion and
640x480 K with `fx=fy=570.3422047415297`, `cx=319.5`, `cy=239.5`.

The native detections in the **original** local R5 image, scaled with pixel-centre
conversion, reproduce the remote profile's four pixels **exactly**. Its stated
board assumptions reproduce **1.804 mm RMS / 2.932 mm maximum** on the other
20 corners. See `claim_replay.json`. This verifies the numerical claim conditional
on its inputs. It does not verify the actual 50 mm print, 0.55 m placement, sensor
intrinsics, independent accuracy, or current camera pose.

Remote SHA-256 checks matched the local original R5 image, burst 0 and old final
image. R5 scripts capture `/camera/color/image_raw` in ROS domain 1, but do not
save synchronized CameraInfo, timestamps, sensor serial or measurements. The
640-mode script changes parameters and kills/relaunches applications: it was
**read, never executed**. Installed parameters currently specify 320x240; that
does not prove what any running process loaded.

A 15-second domain-1 subscription received **neither image nor CameraInfo**.
Topic names in the resulting graph can arise from the audit's own subscribers;
they do not prove a publisher existed. No driver was started or restarted.
No complete independent R5 CameraInfo file was found in the limited saved-file
search. New profile remains locked despite the remote file's `calibrated: true`.

## Measurement workflow and completed front session

**Final front-only session update:** the user confirmed rear-wheel hub
alignment to the REAR AXLE line, centreline/forward alignment, and physically
measured both sheet scale bars as correct. The actual PDF specifies front centre
`[0.620, 0]` m, yaw90 and 50mm squares. Fresh driver-published CameraInfo and
320/640 captures are saved in `calibration_risabot5/front_session/`; see its
`README.md`. These supersede the initial lack of a live message, but do not prove
factory-vs-default intrinsic provenance. The user clarified that the lens is
offset within the housing; a camera mount adjustment was not confirmed. The car
was moved right and then explicitly realigned. Only `aligned_current320` and
`aligned_current640`, captured AFTER that confirmation, underpin the final fit.
Extra printed-sheet guide features supply the additional reference checks; tape
marks are an optional separate survey, not an outstanding activation requirement.

Keep the final camera mount and lens focus fixed. The operator/main task owns
driver operation and final mode selection; do not run the historical shell scripts.

1. **Identity and stream:** confirm car 2/Risabot 5, physical front camera and
   topic. Record host, sensor serial if available, UTC time, image header/frame,
   final raw width/height, crop/binning/rotation and driver mode. Prefer a fresh
   capture directly at the final runtime resolution. Current saved 640x480
   evidence cannot establish a different native 320x240 camera mode.
2. **Intrinsics:** save the full R5 `/camera/color/camera_info` message at the same
   mode and session as the raw image: header, width, height, distortion_model,
   k, d, r, p, binning_x/y and complete roi. Do not copy K or zero D from either
   chassis's profile. Verify the intrinsic source and inspect undistorted straight
   lines. If factory values fail, collect at least 15 accepted distinct 9x6
   checkerboard views (target 20-25), covering all nine image cells and several
   tilts/distances; measure its square size. The handover requires <=0.8 px RMS.
   Bundled intrinsic fitting can compare pinhole/fisheye, but `bev_core` cannot
   consume equidistant coefficients without a separately owned runtime change.
3. **Floor board:** physically verify 6x4 inner corners and measure multiple
   square spans along both axes; record actual square size in metres. Nominal
   print is 50 mm per square. Check uniform print scaling, flatness and scale
   bars. Mark the rear-axle midpoint on the ground and the forward centreline.
   Measure this capture's board centre `[forward,left]` and directed yaw. Do not
   reuse 0.765 m, 0.62 m, or the remote 0.55 m claim without its measurement record.
4. **One usable raw floor frame:** include the entire board/white surround and
   **at least two additional, distinct floor reference features** at known `[forward,left]`
   locations spanning useful near/far/lateral coverage. These can be surveyed tape
   markers or printed guide features on the verified-scale/aligned sheet. Record
   which method establishes their positions and its shared assumptions. Save their raw pixel
   centres and measured positions. These must not be any of the 24 board corners.
   One frame with all targets is the minimum; if they cannot fit, use another
   frame with the same fixed camera and unchanged mode. Keep all 24 intersections
   sharp; move the board and remeasure if compressed or cropped. A burst of an
   unchanged board alone is not independent geometric validation.
5. **Ordering:** on the numbered overlay identify the physical corners at indices
   0, 5, 18 and 23, using measured directed board axes/asymmetric margin marks.
   Record how this establishes identity/rot180/flip_rows/flip_cols. An unmarked
   symmetric board cannot decide front/left signs by reprojection error alone.
6. **Acceptance:** four corners construct H; all 20 others and all independent
   markers must have maximum error <=20 mm for this workflow. Inspect undistorted
   straight edges, BEV orientation and the coverage mask over the actual useful
   driving region. The small board does not validate the entire configured
   0.05-1.80 m range. Record measured coverage limits and extend validation if
   the main task needs more range. Apply equivalent steps separately to secondary;
   there is no secondary R5 calibration evidence here.

## Reproduce locally

Run from the existing repository root. Python dependencies: `opencv-python`,
`numpy`, `PyYAML`; `pytest` for tests. This audit used OpenCV 4.12.0 / NumPy 2.2.6.
The tool imports only the bundled calibration math and `bev_core`; ROS is not
needed. Keep their source hashes with the run. Use `-B` to avoid bytecode files.

```powershell
python -B tools/calibrate_risabot5_camera.py inspect 'calibration_risabot5/inputs/front_floor*.png' 'calibration_risabot5/inputs/r5_*.png' --output calibration_risabot5/replay_native
python -B tools/calibrate_risabot5_camera.py inspect calibration_risabot5/inputs/front_floor_sheet_final_640x480.png --roi 35 125 240 41 --scale 1 4 --output calibration_risabot5/replay_old_recovery
python -B tools/calibrate_risabot5_camera.py inspect 'calibration_risabot5/inputs/r5_burst_*.png' calibration_risabot5/inputs/r5_front_floor_640x480_new.png --roi 60 260 565 65 --scale 2 2 --output calibration_risabot5/replay_r5_recovery
python -B -m pytest -q -p no:cacheprovider tests/test_risabot5_calibration.py
```

Each inspect run requires a **new** evidence subdirectory and records actual
dimensions, hashes, detector version, preprocessing and all raw-pixel corners.
Already annotated `*_detected_*` inputs are present only for the historical audit;
never use them for a fit. Recovery ROIs above are image-specific diagnostics,
not a default for future captures. Prefer successful native detection.

For a measured session, place raw image and full CameraInfo under a new evidence
input directory, copy `session.template.json` to `session.json`, and fill every
null with measured/source-backed information. Manifest file paths are relative
to the manifest. A JSON or YAML CameraInfo mapping with lowercase ROS keys is
accepted. `capture_provenance` should identify vehicle/camera/topic/time/mode;
`intrinsics_provenance` identifies the saved message or intrinsic fit and sensor;
`measurement_provenance` names the measurement record and operator;
`ordering_evidence` describes verified corner identities. Save measurement photos
or notes beside the manifest. Populate the PNG hash from its detection record.

```powershell
python -B tools/calibrate_risabot5_camera.py inspect calibration_risabot5/inputs/front.png --output calibration_risabot5/session_detection
python -B tools/calibrate_risabot5_camera.py fit calibration_risabot5/session.json --output calibration_risabot5/session_fit
```

Use actual `target_resolution`. If it differs from the capture, the tool permits
conversion only with documented **software resizing of the same raw image**:
`software_resize_only: true` and `resize_provenance`. Point mapping is
`u'=(u+0.5)*sx-0.5`, `v'=(v+0.5)*sy-0.5`; K becomes `A @ K`, D is unchanged.
A 640->320 resize maps 319.5 to 159.5, not 159.75. A changed sensor mode, crop,
rotation or binning requires matching effective intrinsics/captures; matching
aspect ratio alone is not evidence. Cropped/binned and fisheye CameraInfo are
rejected by this preparation tool rather than silently misinterpreted.

`fit` verifies hashes, bounds, convex ordering, finite geometry, supported
intrinsics, 20 held-out errors and independent-marker errors. It also compares
its metric result with the actual `bev_core` undistort/BEV pipeline (<0.1 mm
numerical agreement) and saves `bev.png`, `coverage.png`, `fit_report.json`, and
`candidate_camera_profiles.yaml`. Geometry failure returns exit code 2.
For staged capture review, `fit ... --board-only` permits empty independent
marker lists and saves a preliminary board fit. Its report explicitly records
`independent_markers: null`, `geometry_pass: false`, and a pending-validation
status; exit code 2 means incomplete validation in this case. It cannot enable BEV.
**Every exported candidate remains `calibrated: false`, including a geometry
pass.** A missing measurement never gets a numeric default. The integrating task
must review provenance/orientation/straight lines/coverage before copying measured
fields into the exact new profile path and enabling the appropriate camera. That
review is now complete for the front's local sheet region and its checked-in
profile has been updated; archived tool-generated candidates remain locked.

`calibration_risabot5/read_remote_evidence.py` documents the authorized read-only
snapshot; it refuses to overwrite its output directory. It is optional and was
not part of the offline detector/geometry tests. `audit_saved_evidence.py` records
conditional replays of old saved claims; it is not a route to activate a profile.

## Verification and integration handoff

28 tests passed: 21 new synthetic tests plus seven existing `test_bev_core.py`
tests. The new tests cover distorted pinhole projection, half-pixel
resize/K consistency, crop inverse mapping, board origin/yaw/span, wrong-order
ambiguity exposed by independent markers, held-out error, malformed K/D/ROI,
degenerate/crossed/out-of-frame points, failed validation, hash mismatch, forbidden
output paths, mode-change refusal, the reviewed front profile rendering the actual
capture, and both secondary and a synthetic disabled-primary fixture refusing BEV.
End-to-end tests create only synthetic temporary evidence and do not imply that
R5 is physically calibrated. Test output is saved under `calibration_risabot5/`.
`baseline.json` and `dirty_file_check.json` record the workspace snapshots. Some
pre-existing dirty files changed concurrently in their owning tasks; this task
did not restore or edit them. Repository-wide `git diff --check` reports trailing
whitespace in another task's `dashboard.py`; it was left for that owner.

Created files are limited to the four requested code/config/test/doc paths and
`calibration_risabot5/`. Existing central parameters, control, steering, launch,
dashboard files and car 1 profile remain owned by their existing tasks. The
integrating task must deliberately choose/package the new profile; no setup or
launch integration was made here. The remote existing profile was archived as
evidence only and was not edited or selected.
