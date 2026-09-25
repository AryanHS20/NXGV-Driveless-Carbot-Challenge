# Handover: Carbot camera calibration, steps 2, 3 and 4

From the RISA Bot team (NxGV Driverless CarBot Challenge 26/27).
Code source: our repo, pinned to commit **`d5aa11d`** (Phase 3) so the links never change under you.

| Step | What it gives you | Status in our repo |
|---|---|---|
| 2. Camera identity | which physical camera is front / left / right | **No tool yet** (wizard is a later phase). Done by hand, see below. |
| 3. Camera intrinsics | lens model per camera (fx, fy, cx, cy, distortion) | Tool: `calib_intrinsics.py` |
| 4. 3-camera extrinsics + IPM | each camera's pose on the car + a stitched top-down check image | Tool: `calib_extrinsics.py` |

The math is plain Python + OpenCV + numpy. ROS 2 (`rclpy`, `sensor_msgs`) is only needed for live capture on the car. You do **not** need our other packages, custom messages or a colcon build.

---

## 1. Links

Repo: https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2
Pinned commit: https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/tree/d5aa11d1a90e10dc1307fec14d186efaaec59212

Base URL for every file below:
`https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/`

**Calibration code (`src/carbot_perception/carbot_perception/`)**

| File | Role |
|---|---|
| [calib_intrinsics.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_perception/carbot_perception/calib_intrinsics.py) | Step 3 command-line tool (live or from saved images) |
| [calib_extrinsics.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_perception/carbot_perception/calib_extrinsics.py) | Step 4 command-line tool (live or from saved images) |
| [calib_core.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_perception/carbot_perception/calib_core.py) | The math: chessboard detection, pinhole + fisheye fit, solvePnP with ordering disambiguation, seam error |
| [calib_io.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_perception/carbot_perception/calib_io.py) | Session folders, YAML read/write, ROS frame grabber |
| [camera_model.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_perception/carbot_perception/camera_model.py) | Lens models, mounts, pixel ↔ ground projection |
| [road_mask.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_perception/carbot_perception/road_mask.py) | IPM grid + stitch, used to draw the step-4 check image |
| [ros_image.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_perception/carbot_perception/ros_image.py) | `sensor_msgs/Image` → numpy without cv_bridge |
| [calibration_store.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_common/carbot_common/calibration_store.py) | (in `carbot_common`) timestamped session folders, `summary.yaml`, `ACTIVE` marker |

**Config (`src/carbot_bringup/config/`)**

| File | Role |
|---|---|
| [data/cameras.yaml](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_bringup/config/data/cameras.yaml) | Sensors, topics, **roles** (step 2), nominal mounts (step 4 starting guess) |
| [data/calibration_steps.yaml](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_bringup/config/data/calibration_steps.yaml) | Board sizes, floor board positions, pass thresholds |
| [fastdds/disable_shm.xml](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_bringup/config/fastdds/disable_shm.xml) | Lets a normal-user subscriber see topics from the root `mipi_cam` |

**Printables and pictures**

| File | Role |
|---|---|
| [intrinsics_board_A4.pdf](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/docs/calibration/intrinsics_board_A4.pdf) | Step 3 hand-held board: 9×6 inner corners, 25 mm squares |
| [floor_board_A3.pdf](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/docs/calibration/floor_board_A3.pdf) | Step 4 floor board: 6×4 inner corners, 50 mm squares (print 3) |
| [floor_sheet.pdf](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/docs/calibration/floor_sheet.pdf) | Step 4 alternative: all 3 boards on one large-format sheet, nothing to measure |
| [calib_mat.png](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/docs/images/calib_mat.png) | Picture of the step 4 floor layout |
| [make_boards.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/tools/calibration/make_boards.py) | Regenerates the PDFs from `calibration_steps.yaml` |
| [Camera_Setup.md](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/docs/reference/Camera_Setup.md) | Verified RDK X5 MIPI camera commands and gotchas |

**Laptop sandbox (test before touching the car)**

| File | Role |
|---|---|
| [tools/sandbox/run_calib_intrinsics.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/tools/sandbox/run_calib_intrinsics.py) | Step 3 on a virtual lens (`--synth`) or a laptop webcam (`--webcam 0`) |
| [tools/sandbox/run_calib_extrinsics.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/tools/sandbox/run_calib_extrinsics.py) | Step 4 on virtual floor boards with a known mount error |
| [tools/sandbox/sandbox_common.py](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/tools/sandbox/sandbox_common.py) | Virtual track and cameras |
| [tools/sandbox/requirements.txt](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/blob/d5aa11d1a90e10dc1307fec14d186efaaec59212/tools/sandbox/requirements.txt) | pip packages |
| [src/carbot_perception/test/](https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/tree/d5aa11d1a90e10dc1307fec14d186efaaec59212/src/carbot_perception/test) | Unit tests (`test_calibration.py`, `test_camera_model.py`) |

---

## 2. Get the files

Either unzip `carbot_calibration_2-4.zip` (same folder layout as the repo, only the files above), or clone just those folders:

```bash
git clone --filter=blob:none --no-checkout https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2.git carbot_calib
cd carbot_calib
git sparse-checkout init --no-cone
git sparse-checkout set /src/carbot_perception/ /src/carbot_common/ /src/carbot_bringup/config/ /tools/sandbox/ /tools/calibration/ /docs/calibration/ /docs/images/ /docs/reference/Camera_Setup.md
git checkout d5aa11d1a90e10dc1307fec14d186efaaec59212
```

Keep the folder layout. The tools find `calibration_steps.yaml` and `cameras.yaml` by relative path (`src/carbot_bringup/config/`).

Dependencies: Python 3.8–3.12, `numpy`, `opencv-python` ≥ 4.5.4, `pyyaml` (and `pytest` for the tests):

```bash
pip install -r tools/sandbox/requirements.txt
```

---

## 3. Prove it works on a laptop first (5 min, no car)

```bash
python tools/sandbox/run_calib_intrinsics.py --synth fisheye --headless
python tools/sandbox/run_calib_extrinsics.py --synth --headless
python -m pytest -q src/carbot_perception/test/test_calibration.py src/carbot_perception/test/test_camera_model.py
```

Expected: step 3 prints `focal length error ~0.04 % (OK)`; step 4 prints `extrinsics_ipm: PASS` and each camera recovered within ~2 mm / 0.2°; pytest says `17 passed`. Output images land in `tools/sandbox/out/`. Drop `--headless` to see windows. `--webcam 0` on step 3 lets you practise the board-waving with your laptop camera.

If these fail, the problem is your Python environment, not your car.

---

## 4. Adapt to YOUR car (do this before step 2)

Edit `src/carbot_bringup/config/data/cameras.yaml`:

1. **`sensors:`** one entry per physical camera. The names (`astra`, `ov5647`, `imx219`) are what you pass as `--sensor`. Set `image_topic`, `width`/`height` (or `image_width`/`image_height` for mipi_cam) and `encoding` to what your drivers really publish. Check with `ros2 topic list` and `ros2 topic echo <topic> --once --no-arr`.
2. **`mounts:`** a rough measured guess for each camera in `base_link` = **rear-axle centre on the ground**, +x forward, +y left, +z up; `yaw_deg` 0 = forward, +90 = left; `pitch_down_deg` > 0 = looking down. Step 4 measures the real values; the guess only has to be roughly right (a few cm, ~10–20°) because the solver picks the physically valid solution closest to it. Ours are the RISA Bot CAD values, not yours.
3. **Data folder.** Results go to `$CARBOT_DATA/calibration/<session>/`, default `/home/sunrise/carbot_data`. If your user is not `sunrise`: `export CARBOT_DATA=~/carbot_data`.

---

## 5. Step 2: camera identity (manual)

Goal: know for certain which physical camera is the front, the left and the right, then write it into `cameras.yaml`. On our car the front is the Astra Pro; the two MIPI cameras are the sides, and which is left/right must be confirmed on the car, never assumed.

1. Start each camera in its own terminal **as root** (Camera_Setup.md, verified):
   ```bash
   sudo -i
   source /opt/tros/humble/setup.bash
   ros2 run mipi_cam mipi_cam --ros-args -r __ns:=/cam_ov5647 -p channel:=2 -p image_width:=960 -p image_height:=544
   ```
   (and the same for `/cam_imx219` on `channel:=0`). Always set `image_width`/`image_height`, the default makes the node fail.
2. View one topic at a time with the verified viewer: `hobot_codec` encoder with `codec_sub_topic:=/cam_ov5647/image_raw`, then `websocket`, then open `http://<RDK-IP>:8000` → "Web display". Commands are in Camera_Setup.md. Or `ros2 run rqt_image_view rqt_image_view` if you have a desktop.
3. With a feed on screen, **put your hand over the camera on the car's physical LEFT side** (left when sitting in the car facing forward). The feed that goes dark is your left camera. Repeat for the right.
4. Write it down in `cameras.yaml`:
   ```yaml
   roles:
     front: astra
     left_rear: imx219      # the sensor whose feed went dark for the LEFT hand
     right_rear: ov5647
   roles_confirmed: true
   ```
   Keep the role keys `front` / `left_rear` / `right_rear` exactly; the step 4 tool and the board layout use them.
5. To switch the viewer to the other camera: stop the encoder AND websocket, restart the encoder on the other topic, restart websocket, hard-refresh the browser (Ctrl+Shift+R).

Pass = the covered-lens test matched for both sides and `roles_confirmed: true` is saved.

---

## 6. Step 3: intrinsics (per camera)

**Board:** print `intrinsics_board_A4.pdf` at **100 % / Actual size**, matte paper, glued flat to foam board. Measure the scale bar; if the squares are not 25.0 mm, set the measured size in `calibration_steps.yaml` → step 3 `target.square_m`.

**Run** (camera driver already running; one sensor per run, same `--session` for all steps):

```bash
source /opt/tros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/src/carbot_bringup/config/fastdds/disable_shm.xml
export PYTHONPATH=$PWD/src/carbot_perception:$PWD/src/carbot_common:$PYTHONPATH
export CARBOT_DATA=~/carbot_data            # if not user sunrise

python3 -m carbot_perception.calib_intrinsics --sensor ov5647 --session 20260922_1000
python3 -m carbot_perception.calib_intrinsics --sensor imx219 --session 20260922_1000
python3 -m carbot_perception.calib_intrinsics --sensor astra  --session 20260922_1000
```

(If you do build it as a ROS package, the same tool is `ros2 run carbot_perception calib_intrinsics ...`.)

**How to hold the board:** a view is captured automatically when the board is **held still** in a **new** position, size or tilt. Cover the whole image, especially the **corners and edges**: distortion is only measured where the board has been. Tilt it ±30–45°, near and far. Watch the progress in the terminal and the annotated frame in `<session>/captures/intrinsics/<sensor>/live.jpg`.

**Pass** (from `calibration_steps.yaml`): at least 15 views, reprojection RMS ≤ 0.8 px, board seen in ≥ 9 image cells. It fits both the standard (plumb_bob) and fisheye (equidistant) models and keeps the better one, so you don't need to know your lens type.

**Offline / redo:** the captured images are saved; recalibrate from a folder with `--images <dir>`.

**Output:** `<session>/intrinsics/<sensor>.yaml` (ROS camera_info format), plus `intrinsics_file` filled in in `<session>/data/cameras.yaml`.

---

## 7. Step 4: extrinsics + IPM (all 3 cameras)

Needs step 3 done for all three sensors **in the same session**.

**Floor setup** (picture: `docs/images/calib_mat.png`). Positions come from `calibration_steps.yaml` step 4 `target.boards`, measured from the rear-axle centre on the ground:

| Board | Seen by | Centre (x, y) m | Long side |
|---|---|---|---|
| front | front | (0.62, 0.00) | across the car (yaw 90°) |
| left | left_rear | (0.10, +0.47) | along the car (yaw 0°) |
| right | right_rear | (0.10, −0.47) | along the car (yaw 0°) |

Option A, three A3 boards: tape a cross on the floor under the rear-axle centre and a line for the car's centre line. Lay each `floor_board_A3.pdf` flat at its position. If a board is cut off in its camera, move it, **measure** the new centre and edit `centre_m` / `yaw_deg`. The measurement is what matters, not the default.
Option B, `floor_sheet.pdf`: one large print with all boards, the centre line and the rear-axle line already on it. Tape flat, park the car on the lines, check the scale bars. If you edit any `centre_m` / `yaw_deg`, regenerate with `python tools/calibration/make_boards.py --sheet` and reprint.

Your cameras sit elsewhere than ours, so first check each camera actually sees its whole board, then adjust the positions and reprint/measure.

**Run** (same environment variables as step 3):

```bash
python3 -m carbot_perception.calib_extrinsics --session 20260922_1000
# offline from photos:
python3 -m carbot_perception.calib_extrinsics --session 20260922_1000 --images front=f.png left_rear=l.png right_rear=r.png
# add --activate on the final good run to mark this session ACTIVE
```

It averages the board corners over several frames per camera, solves each camera's pose, and prints per camera: ground error, the measured x/y/z/yaw/pitch/roll, and the difference from your nominal mount.

**Pass:** ground error ≤ 2 cm per camera; seam error ≤ 3 cm (only measured if a board is seen by two cameras). A warning (not a fail) if the result is > 3 cm / 5° from the nominal mount: then either the mount guess in `cameras.yaml` is off, or a board position is wrong.

**Check the picture:** `<session>/captures/extrinsics/ipm_check.png` is the stitched top-down view with each board's true outline drawn in red. The printed squares must sit inside the red rectangles. If one camera's board is shifted or rotated, re-measure that board.

**Output:** measured `mounts:` and `extrinsics_calibrated: true` in `<session>/data/cameras.yaml`; results in `04_extrinsics_ipm.yaml`.

---

## 8. What the session folder looks like

```
~/carbot_data/calibration/
  ACTIVE                         name of the session your stack should load
  20260922_1000/
    summary.yaml                 PASS / FAIL per step
    03_camera_intrinsics.yaml    per-sensor results
    04_extrinsics_ipm.yaml       per-camera results
    intrinsics/<sensor>.yaml     ROS camera_info format
    data/cameras.yaml            full cameras.yaml with your new values
    captures/                    every image used (for redo / offline)
```

Old sessions are never deleted, so you can always roll back by pointing `ACTIVE` at an earlier one. To use the results in your own stack, read `data/cameras.yaml` from the active session.

---

## 9. Troubleshooting

| Symptom | Fix |
|---|---|
| Tool waits forever / no frames | The tool can't see the root `mipi_cam` topics. Export `FASTRTPS_DEFAULT_PROFILES_FILE=.../disable_shm.xml` in **both** terminals (or run the tool as root too). Same `ROS_DOMAIN_ID` everywhere. Check `ros2 topic hz <topic>`. |
| `create_and_run_vflow failed` | `mipi_cam` not run as root. |
| `creat_vse_node failed, ret -10` | `image_width`/`image_height` not set (default 1088×1280 is taller than the sensor). |
| `There are no available host` | That camera port is already in use: `ps -ef \| grep -E "codec\|websocket\|mipi" \| grep -v grep`, then `kill <PID>`. |
| Step 3 never captures | Board not held still, not in a new pose, glossy, or bent. Matte, flat, good even light. |
| Step 3 RMS too high | Board printed scaled (measure squares), board bent, motion blur, or too few edge/corner views. |
| Step 4 "board not found" | Board cut off or too far/oblique in that camera: move it closer to that camera, re-measure, edit `centre_m`. |
| Step 4 ground error > 2 cm | Wrong board position (re-measure from the rear-axle centre, not the car's nose), wrong `square_m`, or step 3 intrinsics wrong for that camera. |
| Step 4 camera "under the floor" / flipped | Nominal mount in `cameras.yaml` too far off (yaw/pitch sign wrong). Fix the guess and rerun. |
| Left/right swapped in `ipm_check.png` | Step 2 roles wrong. Redo the covered-lens test. |
| "get camera calibration parameters failed" | Harmless mipi_cam warning. |

Questions: ask the RISA Bot team.
