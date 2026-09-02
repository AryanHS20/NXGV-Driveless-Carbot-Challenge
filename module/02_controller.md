# Module 2: The RISA-bot Controller & Hardware Interface

This module describes the hardware control board used in RISA-bot, its connection with the Horizon RDK X5 developer kit, and how the `servo_controller` node communicates with physical actuators and sensors.

---

## 1. Hardware Architecture

The RISA-bot uses a two-tier brain architecture:
1. **High-Level Processor**: The **Horizon RDK X5** developer board runs Linux (Ubuntu 20.04/22.04), ROS 2 Humble, camera processing, LiDAR processing, and autonomous driving logic.
2. **Low-Level Controller**: The **Yahboom Rosmaster Expansion Board** operates as a coprocessor. It contains dedicated motor driver chips, servo headers, and a built-in Inertial Measurement Unit (IMU).

![RISA-bot Hardware Connection Block Diagram](images/risabot_hardware_block_diagram.png)
<!-- Image Description: Block diagram showing Horizon RDK X5 connected to the Rosmaster Board via serial UART, and the Rosmaster board connected to motors, servo, battery, and controller receiver. -->

---

## 2. Serial Communication Interface

The RDK X5 communicates with the Rosmaster board via a **UART Serial Connection**:
- **Device Port**: `/dev/myserial` (or `/dev/ttyUSB0` / `/dev/ttyTHS1` depending on Linux configuration)
- **Baud Rate**: `115200`

The software interaction is wrapped in the `Rosmaster_Lib` library. The node initializes it as follows:
```python
from Rosmaster_Lib import Rosmaster
self.bot = Rosmaster(com="/dev/myserial")
self.bot.create_receive_threading() # Starts background thread to parse incoming packets
self.bot.set_auto_report_state(True) #MCU reports speed/encoders/IMU raw data every 10ms
```

---

## 3. Capabilities & Kinematics

### 3.1. Motors & Encoders
While the Rosmaster board supports up to 4 independent encoders and motors, **RISA-bot is physically configured with a single rear-drive motor (drive motor index 0)**.
- The other 3 motor channels are undriven.
- **Important Note**: Reading encoders from undriven wheels returns raw sensor noise or floating data that will corrupt your odometry calculation. You must only read the encoder from the driving motor.

In `servo_controller.py`, the speed is set using:
```python
# motor_pwm values range from -255 (full reverse) to 255 (full forward)
self.bot.set_motor(motor_pwm, 0, 0, 0)
```

And ticks are read using:
```python
ticks = self.bot.get_motor_encoder()
# Select the single driven wheel's encoder value
motor_idx = int(self._param_cache['drive_motor_index']) # default: 0
avg_ticks = ticks[motor_idx]
```

### 3.2. Steering Servo
RISA-bot uses an **Ackermann steering mechanism** where a front-mounted servo rotates the front steering wheels left and right.
- **Servo Channel**: `4` (controlled via PWM headers on the board).
- **Physical Angle Limits**: Center is generally at `90°`. The steering limits are set asymmetrically around the center:
  - Turn Left: Decreases angle (typically down to `40°` depending on parameters).
  - Turn Right: Increases angle (typically up to `140°` depending on parameters).

In python code:
```python
# steer_angle is restricted by self.servo_range_left and self.servo_range_right
self.bot.set_pwm_servo(self.servo_steer_id, steer_angle)
```

![Ackermann Steering mechanism servo diagram](images/steering_servo_ackermann.png)
<!-- Image Description: Visual drawing of the front wheels steering layout, highlighting the servo motor, the steering link, and the left/right angle limits. -->

### 3.3. Inertial Measurement Unit (IMU)
The Rosmaster board has an onboard IMU (such as MPU9250 or ICM20948) that reports linear acceleration, angular velocity, and estimated attitude.
- **Topic**: `/imu/rpy` (Roll, Pitch, Yaw in degrees).
- **Attitude Retrieval**:
  ```python
  roll, pitch, yaw = self.bot.get_imu_attitude_data()
  ```
- **Software Calibration**: Raw IMU values can drift or have minor offsets. The `servo_controller` uses parameters (`imu_roll_offset`, `imu_pitch_offset`, `imu_yaw_offset`) to zero the IMU when calibrated and filter noise through an Exponential Moving Average (EMA) and a small deadband filter.

---

## 4. Key Parameters

You can view and set these parameters using `ros2 param` on the `/servo_controller` node:

| Parameter | Default | Type | Description |
|---|---|---|---|
| `drive_motor_index` | `0` | int | Which encoder channel maps to the active drive wheel (0=FL, 1=FR, 2=RL, 3=RR). |
| `ticks_per_meter` | `1050.0` | float | Encoder counts corresponding to 1 meter of distance. |
| `servo_center` | `90` | int | Steering servo neutral angle in degrees. |
| `servo_range_left` | `50` | int | Max steering offset to the left (Center - Range). |
| `servo_range_right` | `50` | int | Max steering offset to the right (Center + Range). |
| `auto_right_steer_boost` | `1.3` | float | Correction multiplier applied only to right turns to balance Ackermann turning asymmetry. |
| `joy_timeout` | `0.8` | float | Safety timer threshold; stops motors if no controller signal is received. |
