# Stage 1 camera calibration

The repository profiles are intentionally marked `calibrated: false`. Do not
change that flag until the cameras are mounted in their final positions and the
following measurements have been recorded for each sensor.

## Required measurements

1. Confirm the published frame is exactly 960 x 544 and note which physical
   sensor reaches the primary and secondary relay topics.
2. Capture at least 20 sharp checkerboard views across the whole image.
3. Run the offline calibration utility. Dimensions are checkerboard *inner
   corners* and the square size must be measured:

   ```bash
   ros2 run risabot_v4_experimental calibrate_intrinsics \
     'captures/primary/*.png' --columns 9 --rows 6 --square-m 0.024 \
     --camera primary
   ```

   It prints a YAML fragment and deliberately leaves `calibrated: false`.
   Review the RMS pixel error and rejected frames. Reject a result with visibly
   curved straight lines or poor corner fit.
4. Place four clearly visible ground marks at measured `[forward, left]`
   coordinates relative to the vehicle reference point.
5. Record the raw pixel centre of each mark in the same order as its measured
   ground coordinate. List the four correspondences around the perimeter rather
   than crossing the quadrilateral. The implementation undistorts those raw
   pixel points before calculating the homography.
6. Enter the values in `config/camera_profiles.yaml`, set only that camera's
   `calibrated` flag true, and run the tests.

## Shadow validation

```bash
ros2 launch risabot_v4_experimental stage1_bev.launch.py enabled:=true
ros2 topic echo /v4_experimental/bev/status
```

Inspect the primary and secondary debug images and coverage masks. Straight
ground lines must remain straight, the same physical mark must land at its
measured metric location, and the profile must reject frames with a different
resolution. No Stage 1 topic is consumed by the vehicle controller.

## Topics

- `/v4_experimental/bev/primary/image`
- `/v4_experimental/bev/primary/coverage`
- `/v4_experimental/bev/secondary/image`
- `/v4_experimental/bev/secondary/coverage`
- `/v4_experimental/bev/status`
