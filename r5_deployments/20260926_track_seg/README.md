# R5 track-seg road source (seg replaces the classical HSV mask)

**Status: code complete and unit-tested on the laptop; NOT yet run on the BPU or the car.**
The `.bin` has not been compiled and no board smoke test has been done.

## What it changes

The classical path classifier is `v4_road_mask_shadow` (HSV dark-road threshold on the
BEV image) feeding `trajectory_shadow` -> `motion_executor`. With `road_source:=seg`:

```text
camera 320x240 -> track_seg (BPU, LRASPP 512x288) -> mask warped to BEV with the same
homography -> /v4_experimental/road/primary/seg_candidate -> v4_road_mask_shadow
(replaces the HSV threshold; seed/connected-component, memory fusion, corridor unchanged)
-> trajectory_shadow -> motion_executor (unchanged)
```

Steering and speed logic are untouched. If `track_seg` goes silent for 0.5 s the shadow
falls back to the classical mask automatically (`seg_fallback_to_classical`).

## Revert (three levels)

1. **Instant, no rebuild:** launch with `road_source:=classical` (this is the default).
2. **Automatic:** seg node dead/silent -> classical fallback after 0.5 s.
3. **Full file restore:** `./revert_track_seg.sh` (restores the backup made by the deploy).

## Steps

Compile (needs Docker + OpenExplorer + the shared `bpu_export/mapper.py`, none of which
are on the laptop this was written on): see `../../track_seg_x5_port/README_X5_PORT.md` s.1.
Prefer ~50 **Astra** frames in `cal_images/` (current ones are laptop/phone photos).

```bash
# laptop -> board
scp -r r5_deployments/20260926_track_seg track_seg_x5_port/model_output/track_seg_512x288_nv12.bin \
    track_seg_x5_port/board/verify_track_seg.py track_seg_x5_port/board/test_track.jpg sunrise@<RDK_IP>:~/
# board: smoke test the model alone first; stop here if it fails
cd ~ && python3 verify_track_seg.py --model track_seg_512x288_nv12.bin --image test_track.jpg
mv ~/track_seg_512x288_nv12.bin ~/20260926_track_seg/
# board: install (dry-runs patches, backs up, builds). Behaviour is still classical.
cd ~/20260926_track_seg && ./deploy_track_seg.sh
# board: run seg WITHOUT the systemd service also running
sudo systemctl stop risabot5-track-stack.service
ros2 launch risabot_v4_control track_test.launch.py road_source:=seg
```

Check `systemctl cat risabot5-track-stack.service` to see how the service launches so you
can pass `road_source:=seg` there later.

## Test order

1. MANUAL, car on blocks or hand-pushed: `ros2 topic echo /v4_experimental/road/seg_status`
   (want `bpu_loaded:true`, `fps` near the camera rate, `errors:0`) and
   `/v4_experimental/road/status` (want `mask_source:seg`, `last_frame_source.primary:seg`,
   `seg.fallback` not growing, a sensible `corridor`).
2. AUTO at low duty on a straight, hand on MANUAL.

Live FPS/latency comes from `seg_status` (`fps`, `forward_ms_p50/p95`, `frame_ms_p50/p95`).
