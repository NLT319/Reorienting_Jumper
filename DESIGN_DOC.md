# Jumping Robot Flight Controller — Design Doc

## What it is
A flight controller for a spring-launched jumping robot. It detects launch automatically from accelerometer magnitude, then during the ballistic phase drives two reaction wheels to rotate the robot so the base/foot faces the instantaneous velocity vector to prepare for landing.

Runs on an ESP32‑C3, reads a WitMotion WT901 IMU over I2C, and commands two IQ motors over Serial0.

## Repo / file layout
- `src/main.cpp`
  - Thin entrypoint only. Always runs the PD controller.
  - Calls `pd_setup()` and `pd_loop()`.

- `src/PD_controlled.cpp`
  - The PD flight controller implementation.
  - Owns the state machine, quaternion handling, velocity integration, motor mixing, and telemetry.

- `src/pd_controlled.h`
  - Declares `pd_setup()` / `pd_loop()`.

- `extras/motor_test.cpp`
  - Archived motor test firmware (motor mixing + individual control validation).
  - Not compiled by default; kept as a reference/backup.

## Hardware
Microcontroller: ESP32‑C3 (Seeed XIAO ESP32‑C3)

- I2C: SDA=8, SCL=9
- Reset button: GPIO2, active low, `INPUT_PULLUP`
- Motors: two IQ modules on Serial0
  - Motor 0 (ID 0): `roll_control`
  - Motor 1 (ID 1): `pitch_control`
  - IDs are already configured on the motors.

IMU: WitMotion WT901 using the JY901 library (I2C)

Signals used:
- accelerometer (specific force)
- gyroscope
- quaternion (Kalman-filtered) — trusted pre-launch; not trusted in freefall

Scaling (as used in code / module configuration):
- Accel: ±16g → `raw/32768 * 16`
- Gyro: ±2000 deg/s → `raw/32768 * 2000`
- Quaternion: `q=[w,x,y,z]` → `raw/32768`, roughly [-1, 1]

## Coordinate frames and conventions
### Body frame (robot/IMU frame)
- IMU is mounted “Y-up”: robot’s long/thrust axis is +Y in body frame.
- At rest upright, accelerometer reads approximately `(0, +1g, 0)` in body frame (specific force points up).

### World frame
- World frame is defined at power-on by the IMU’s Kalman output: at startup the reported quaternion is near identity, so world≈body initially.
- Only gravity direction matters for the ballistic model; world X/Z are whatever the IMU defines as “horizontal” at startup.

### Gravity vector convention
- `gravity_world` is a **unit vector pointing down** (direction of gravitational acceleration), not what the accelerometer directly reads.
- Pre-launch we estimate it from the quaternion:
  - `up_world = quatRotVec(q_imu, bodyUp)` where `bodyUp = (0, +1, 0)`
  - `gravity_world = -normalize(up_world)`

## Motor geometry and mixing
Both motor axes lie in the XZ plane and are at 45° relative to the IMU X and Z axes.

Body-frame attitude error components:
- `ex`: error about body X axis
- `ez`: error about body Z axis

Gyro rates for D term:
- `wx`, `wz` (deg/s)

Mixing (1/√2 absorbed into gains):
- Motor 1 (ID 1):
  - `cmd1 = KP * ( -ex - ez ) - KD * ( -wx - wz )`
- Motor 0 (ID 0):
  - `cmd0 = KP * (  ex - ez ) - KD * (  wx - wz )`

Per-axis sign flips can be applied in code if a motor is mounted reversed.

## Operating phases

### 1) Idle / pre-launch
Goal: motors off, track gravity direction, and arm the launch detector.

What happens:
- Read accel, gyro, quaternion each loop.
- Update `gravity_world` continuously from quaternion.
- Keep `candidateQuat` updated with the latest quaternion (used later at burnout).
- Motors commanded to zero.
- Wait for launch detection.

### 2) Acceleration phase (spring release / impulse)
Goal: detect the “burn” (high acceleration), and integrate the actual launch velocity.

Launch state machine:
1) Confirm ambient first:
   - `|a| <= 1.15g` for 3 consecutive samples
2) Detect acceleration phase:
   - `|a| >= 3.0g` for 3 consecutive samples
3) Detect end of acceleration phase / start of ballistic:
   - after the spike is seen, wait for `|a| <= 1.15g` for 3 consecutive samples
   - that ambient-confirm transition is ballistic start

During the acceleration phase (after the spike is confirmed and before ballistic start):
- Quaternion continues to be read and stored as `candidateQuat`.
- Integrate velocity in the world frame using accelerometer (specific force) and the captured gravity direction:
  - `accel_body_g = (ax, ay, az)` (specific force, in g)
  - `specific_force_world_g = quatRotVec(candidateQuat, accel_body_g)`
  - Net linear acceleration:
    - `linear_accel_world = (specific_force_world_g + gravity_world) * 9.80665` (m/s²)
  - Integrate:
    - `vel_world += linear_accel_world * dt`

Motors remain off throughout this phase.

### 3) Ballistic phase
Triggered at the end-of-accel ambient confirmation.

Initialization:
- Freeze `gravity_world` (stop updating it from the IMU).
- Set `q_body = candidateQuat` (orientation snapshot at burnout).
- `vel_world` already contains integrated initial ballistic velocity.
- Initialize timestamps for gyro and velocity integration.

Each loop:
1) Propagate attitude using gyro only:
   - IMU quaternion isn’t trusted in freefall (no gravity reference), so integrate:
   - `q_body = integrateGyro(q_body, wx, wy, wz, dt)`

2) Evolve velocity under constant gravity:
   - `vel_world += gravity_world * 9.80665 * dt`
   - `velDir_world = normalize(vel_world)`

3) Compute target attitude:
   - Target: “foot points into velocity”
     - body `-Y` aligns with `velDir_world`
     - equivalently body `+Y` aligns with `-velDir_world`
   - Compute shortest-path quaternion that rotates current body +Y (in world) onto target +Y:
     - `currentBodyY = quatRotVec(q_body, (0,1,0))`
     - `targetBodyY = -velDir_world`
     - `q_err_world = quatFromVecToVec(currentBodyY, targetBodyY)`
   - Express error in body frame:
     - `q_err_body = q_body* ⊗ q_err_world ⊗ q_body`
   - Convert quaternion error into axis-angle and extract X/Z error components:
     - `ex = err_deg * axis_x`
     - `ez = err_deg * axis_z`

4) PD + deadband + mixing:
- Apply deadband per axis (1.5°).
- PD uses:
  - P on `ex` and `ez`
  - D directly on raw gyro rates `wx` and `wz`
- Mix into motor commands using the mapping above.
- Clamp to `MAX_MOTOR_VELOCITY`.

## Tunable parameters
- `KP`: proportional gain (motor units per degree)
- `KD`: derivative gain (motor units per deg/s)
- `MAX_MOTOR_VELOCITY`: clamp
- `ANGLE_DEADBAND_DEG`: per-axis deadband
- Launch detection:
  - `LAUNCH_ACCEL_THRESHOLD_G` (default 3.0g)
  - `AMBIENT_ACCEL_THRESHOLD_G` (default 1.15g)
  - `LAUNCH_CONFIRM_SAMPLES` (default 3)
  - `AMBIENT_CONFIRM_SAMPLES` (default 3)
- Button debounce: 50 ms

## Button behavior
A debounced press on GPIO2 (active low) resets everything back to idle:
- launch state machine flags/counters
- velocity integrator (`vel_world`)
- quaternion/gyro state (`q_body`, timestamps)
- candidate values

## Telemetry (Serial, 200 ms)
Idle / pre-launch:
- `WAITING: accel=Xg grav=x,y,z`

Ballistic:
- `BALLISTIC: velDir=x,y,z ex=X ez=X -> m1=X m0=X`

## Known limitations and tuning notes
- **Gyro drift:** gyro integration during the ballistic phase accumulates bias error over time. For short jumps (a few seconds) this is usually negligible, but longer flights may require bias correction (e.g., average gyro output at rest pre-launch and subtract).
- **Acceleration-phase velocity integration accuracy:** net linear acceleration is computed by rotating body-frame accel into world frame and compensating gravity. This is only as accurate as the quaternion estimate during the high‑g acceleration phase — the WT901 Kalman filter may lag or distort under rapid dynamics. The result is still a better initialization than assuming unit speed, but it should be treated as an estimate.
- **No aerodynamic drag:** velocity vector evolution assumes pure gravity (no drag). For a small dense robot this is usually reasonable.
- **KP/KD tuning order:** set `KD=0` first, tune `KP` until you get crisp response without oscillation, then increase `KD` to damp overshoot.
- **Motor sign verification:** if a tilt/error in one direction drives the robot further off rather than correcting, negate the sign on that motor’s command in `updateMotorCommand`.
- **Quaternion order verification:** at startup in the `WAITING` telemetry, `grav` should read approximately `(0, −1, 0)` when the robot is upright and still. If it doesn’t, confirm quaternion ordering (`w,x,y,z`) and/or the rotate convention for your WT901 firmware.

