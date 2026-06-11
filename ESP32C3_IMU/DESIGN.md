# Reorienting Jumper: ESP32C3 IMU Flight Controller - Design Document

## Project Overview
An ESP32C3-based autonomous flight controller for a jumping robot that uses an IMU (JY901 9-DOF sensor) to detect launch, estimate flight trajectory, and autonomously reorient the robot mid-air using two steerable motors. The system aims to align the robot's body with its velocity vector during the ballistic phase.

---

## Onboarding Summary
This document is primarily for developers working on the flight controller and telemetry system. A new contributor should know:
- The core behavior is launch detection, burn-phase velocity integration, and ballistic-phase attitude control.
- `src/PD_controlled.cpp` contains the main state machine, physics, and motor command logic.
- `src/telemetry.cpp` provides the WiFi TCP telemetry server; `tools/telemetry_client.py` receives and plots telemetry.
- `DESIGN.md` is the current system architecture and debugging reference.
- Current instrumentation includes high-rate `burn` JSON logs and periodic `ballistic` logs.
- The biggest remaining issue is likely the handoff between burn integration and ballistic attitude control, not raw IMU sign/frame handling.
- The best first task is to capture the last `burn` records and the first `ballistic` records around the transition.

---

## System Architecture

### Hardware Components
- **Microcontroller**: ESP32C3 (Seeed XIAO variant)
- **IMU Sensor**: JY901 (9-DOF: 3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer, quaternion output)
- **Communication**: I2C (IMU data), Serial (motor commands via IQ module)
- **Motors**: 2x IQ modules controlled via serial protocol (MultiTurnAngleControlClient)
- **Input**: Digital push-button (pin 2) for launch reset

### Key I/O Pins
- **I2C SDA**: GPIO 8
- **I2C SCL**: GPIO 9
- **Button**: GPIO 2 (input pullup)

---

## Software Architecture

### Main Entry Points
```
main.cpp
  ├── setup() → pd_setup()
  └── loop()  → pd_loop()
```

**pd_setup()** (PD_controlled.cpp):
- Initialize serial communication (115200 baud)
- Configure I2C for IMU
- Initialize IQ module serial communication
- Configure button GPIO
- Reset launch state machine

**pd_loop()** (PD_controlled.cpp):
- Runs continuously at MCU loop frequency (~1 kHz)
- Executes state machine and flight control loop

---

## Software Modules

### 1. JY901 IMU Library (lib/JY901/)
**Purpose**: Low-level I2C communication with the JY901 IMU sensor.

**Key Classes**:
- `CJY901`: Singleton for IMU access

**Key Methods**:
- `StartIIC()`: Initialize I2C communication
- `GetAcc()`: Read raw 16-bit accelerometer (3 channels)
- `GetGyro()`: Read raw 16-bit gyroscope (3 channels)
- `GetQuaternion()`: Read raw 16-bit quaternion components (q0, q1, q2, q3)

**Data Structures**:
- `SAcc`: 3-axis acceleration + temperature
- `SGyro`: 3-axis angular velocity + temperature
- `SQuater`: Quaternion (w, x, y, z)

**Notes**:
- I2C address: 0x50 (can be customized)
- Raw values are 16-bit signed integers
- Scaling factors applied in main controller

---

### 2. PD Flight Controller (src/PD_controlled.cpp)

#### A. State Machine Architecture

The system operates in **two primary phases**:

```
WAITING PHASE (Pre-Launch)
  └─ Detect Launch → BALLISTIC PHASE (Flight)

WAITING PHASE Sub-States:
  1. accelReturnedToAmbient = false
     └─ Wait for accel ≤ 1.15g for 3 samples (idle detection)
  
  2. accelReturnedToAmbient = true, launchSpikeSeen = false
     └─ Wait for accel ≥ 3.0g for 3 samples (launch spike detection)
     └─ Once detected: initialize velocity integration
  
  3. launchSpikeSeen = true
     └─ Integrate velocity during acceleration phase
     └─ Wait for accel to return ≤ 1.15g for 3 samples (burnout detection)
     └─ Transition to BALLISTIC PHASE

BALLISTIC PHASE:
  └─ Gyro-integrate body orientation (propagate rotation)
  └─ Gravity-integrate velocity trajectory
  └─ Compute attitude error to target velocity direction
  └─ Send PD commands to motors

Reset Trigger:
  └─ Button press → resetLaunchState() → return to WAITING PHASE
```

#### B. Coordinate Frames

- **Body Frame**: Fixed to the robot's body (as used in the code)
  - +X: Right (roll-positive direction)
  - +Y: Up / body "up" direction (primary reference axis)
  - +Z: Forward/back (secondary axis)

- **World Frame**: Inertial reference frame established at rest before launch
  - +Y: World up (at rest body +Y aligns with world +Y)
  - `gravity_world` is a unit vector pointing down (i.e. negative of world +Y)

Note: the implementation treats the body +Y axis as the axis that should align with the velocity direction during control (see `TARGET_BODY_Y_SIGN`).
- `TARGET_BODY_Y_SIGN = +1.0f` enables the reorientation behavior where body +Y is driven to track velocity.
- `TARGET_BODY_Y_SIGN = -1.0f` causes the system to hold attitude instead, effectively inverting the behavior to maintain current orientation.
- In testing, the reorientation mode works and the robot can track the correct behavior, but it tends to overshoot and will likely require further tuning and testing.

#### C. Quaternion Algebra

The system performs 3D rotations using quaternions:

**Quaternion Operations**:
- `quatMul(p, q)`: Quaternion multiplication (compose rotations)
- `quatConj(q)`: Quaternion conjugate (inverse rotation)
- `quatNorm(q)`: Normalize quaternion to unit magnitude
- `quatRotVec(q, v)`: Rotate vector v by quaternion q
- `quatFromVecToVec(a, b)`: Shortest-path rotation from vector a to vector b
- `integrateGyro(q, wx, wy, wz, dt)`: Time-integrate angular velocity to update quaternion

**Key Insight**: 
- IMU quaternion (pre-launch) establishes gravity direction and initial body orientation
- Body quaternion (post-launch) is gyro-integrated forward in time
- Error quaternion = inverse(body) * world_target * body (express error in body frame)

#### D. State Variables

**Calibration (before launch)**:
- `gravity_world`: Unit vector pointing down (gravity direction in world frame)
  - Derived from IMU quaternion at rest: negate the upward direction
  - Updated continuously pre-launch until spike detected

**Launch Detection**:
- `launchDetected`: True once ballistic phase begins
- `launchTimeMs`: Timestamp when ballistic phase started
- `launchSpikeSeen`: True once acceleration spike threshold exceeded
- `accelReturnedToAmbient`: True once return to idle is detected (for state sequencing)
- `candidateQuat`: IMU quaternion snapshot (updated until burnout)

**Flight State (post-launch)**:
- `q_body`: Body orientation in world frame (gyro-integrated)
- `vel_world`: Velocity vector in world frame (gravity-integrated)
- `velDir_world`: Normalized velocity direction (target body +Y)
- `launchAxis_world`: Estimated launch axis (from initial body orientation)

**Timing**:
- `lastGyroUpdateUs`: Timestamp of last gyro integration
- `lastVelUpdateUs`: Timestamp of last velocity update
- `lastTelemetryTime`: Timestamp of last telemetry output

**Button Debounce**:
- `buttonLastChangeMs`, `stableButtonState`, `lastStableButtonState`
  - Filters out mechanical bounce (50 ms debounce)

---

### 3. Physics & Control Logic

#### A. Pre-Launch: Gravity Estimation

Before launch, at rest, the IMU reports:
- Accelerometer = specific force (apparent acceleration)
- At rest: a_specific = -g_world (opposite direction of gravity)

**Algorithm**:
1. Read IMU quaternion (represents body orientation in world frame)
2. Rotate body +Y (0, 1, 0) into world frame: `up_world = q_imu * (0,1,0)`
3. Gravity direction: `gravity_world = -up_world` (unit vector pointing "down")

#### B. Launch Detection

Three-stage state machine for robust detection:

**Stage 1: Idle Detection**
- Threshold: accel ≤ 1.15g
- Confirm: 3 consecutive samples
- Purpose: Ensure stable baseline before launch detection

**Stage 2: Launch Spike**
- Threshold: accel ≥ 3.0g
- Confirm: 3 consecutive samples
- Action: Initialize velocity integrator, record launch axis

**Stage 3: Burnout (Acceleration Phase Ends)**
- Threshold: accel ≤ 1.15g (back to free-fall)
- Confirm: 3 consecutive samples
- Action: Transition to ballistic phase (enabled PD control)
- Snapshot: Body orientation from IMU at burnout → `q_body`

#### C. Acceleration Phase: Velocity Integration

**Duration**: From launch spike to burnout (while accel ≥ 1.15g)

**Velocity Integration**:
```
d(v_world)/dt = a_specific_world + g_world
```

Where:
- `a_specific_world = q_body_burnout * a_body * G_MPS2`
- `g_world = gravity_world * G_MPS2`

**Notes**:
- The implementation integrates `q_body` (gyro) during the burn; the rotated specific force uses the current, gyro-updated `q_body`.
- Accounts for actual acceleration phase dynamics
- If integrated velocity is too low, use `launchAxis_world` as fallback

#### D. Ballistic Phase: Flight Control

Once `launchDetected = true`:

**Step 1: Propagate Body Orientation**
```
q_body ← integrateGyro(q_body, wx, wy, wz, dt)
```
- Integrate gyroscope angular velocity into body quaternion
- Updates continuously based on IMU gyro readings

**Step 2: Evolve Velocity Under Gravity**
```
v_world ← v_world + g_world * G_MPS2 * dt
velDir_world ← v_world / |v_world|  (normalized)
```
- Ballistic trajectory
- Recompute normalized velocity direction for control target

**Step 3: Compute Attitude Error**

**Target**: Body +Y should align with velocity direction
```
targetBodyY = TARGET_BODY_Y_SIGN * velDir_world
```
- `TARGET_BODY_Y_SIGN = +1.0f` drives the reorientation behavior.
- `TARGET_BODY_Y_SIGN = -1.0f` would instead hold the current attitude by aiming body -Y along the velocity vector.

**Current**: Current body +Y
```
currentBodyY = q_body * (0, 1, 0)
```

**World-Frame Error Quaternion**: Shortest path from current to target
```
q_err_world = quatFromVecToVec(currentBodyY, targetBodyY)
```

**Body-Frame Error**: Express error in body coordinates
```
q_err_body = conj(q_body) * q_err_world * q_body
```

**Error Extraction**: Convert quaternion error to axis-angle
```
half_angle = arccos(q_err_body.w)
sin_half = sin(half_angle)
error_deg = 2 * half_angle * (180/π)

axis_x = q_err_body.x / sin_half  (if sin_half > 1e-6)
axis_z = q_err_body.z / sin_half

ex_err = error_deg * axis_x
ez_err = error_deg * axis_z
```

**Step 4: Apply Deadband & Send Commands**
```
if |ex_err| >= 1.5°:  cmd_ex = ex_err
if |ez_err| >= 1.5°:  cmd_ez = ez_err
```

**Step 5: Compute Motor Commands (PD Control)**

System has two motors:
- Motor 0: Roll control (axis between -X and +Z)
- Motor 1: Pitch control (axis between -X and -Z)

**Motor Command Mapping**:
```
cmd_pitch = KP * (-ex - ez) - KD * (-wx - wz)
cmd_roll  = KP * (ex - ez) - KD * (wx - wz)
```

Clamped to: [-199, +199] motor units

**Tunable Gains**:
- `KP = 1.0`: Proportional gain (motor units per degree) — implemented value in `PD_controlled.cpp`
- `KD = 0.08`: Derivative gain (motor units per deg/s) — implemented value in `PD_controlled.cpp`
- `ANGLE_DEADBAND_DEG = 1.5°`: Minimum error to command

---

### 4. Scaling Factors & Constants

**IMU Raw to SI Conversion**:
- Accel: `16g / 32768 = 0.4883 mg/LSB`
- Gyro: `2000°/s / 32768 = 0.0610°/s/LSB`
- Quaternion: `1 / 32768 = 3.05e-5 /LSB`
- Gravity: `9.80665 m/s²`

**Launch Detection**:
- Spike threshold: 3.0g
- Ambient threshold: 1.15g (allows for noise at rest)
- Confirmation: 3 samples per state (debounce)
- Button debounce: 50 ms

**Control Limits**:
- Max motor velocity: 199 units
- Angle deadband: 1.5°
- Min launch speed (fallback): 0.25 m/s
- Telemetry rate: 200 ms (5 Hz)

---

## Control Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    WAITING PHASE (Pre-Launch)                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  [Idle]                                                          │
│    └─ Read IMU quat → update gravity_world                       │
│    └─ Read accel                                                 │
│    └─ accel ≤ 1.15g? (3x) → accelReturnedToAmbient = true      │
│                                                                  │
│  [Spike Detection]                                               │
│    └─ accel ≥ 3.0g? (3x) → launchSpikeSeen = true              │
│    └─ Initialize: vel_world = 0, launchAxis_world, lastVelTime │
│                                                                  │
│  [Acceleration Phase]                                            │
│    └─ Integrate velocity: v ← v + a_world * dt                 │
│    └─ accel ≤ 1.15g? (3x) → TRANSITION TO BALLISTIC            │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│              BALLISTIC PHASE (Flight Control)                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  [Propagate Orientation]                                        │
│    └─ q_body ← integrateGyro(q_body, wx, wy, wz, dt)           │
│                                                                  │
│  [Evolve Trajectory]                                            │
│    └─ v_world ← v_world + gravity * dt                         │
│    └─ velDir_world ← normalize(v_world)                        │
│                                                                  │
│  [Compute Error]                                                │
│    └─ targetBodyY = TARGET_BODY_Y_SIGN * velDir_world          │
│    └─ currentBodyY = q_body * (0, 1, 0)                        │
│    └─ q_err_body = q_body⁻¹ * quatFromVecToVec(...) * q_body  │
│    └─ Extract ex, ez from error quaternion                      │
│                                                                  │
│  [Send Motor Commands]                                          │
│    └─ PD law: cmd = KP*err - KD*rate                           │
│    └─ Send to motors via IQ module serial                       │
│                                                                  │
│  [Telemetry] (every 200 ms)                                     │
│    └─ Print state vectors for debugging/logging                 │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
                            ↑
                    Button Press ↓
                  resetLaunchState()
                   → Back to WAITING
```

---

## Data Flow: Motor Commands

```
PD_controlled.cpp::updateMotorCommand()
  ├─ Input: ex, ez (attitude error), wx, wz (angular rates)
  ├─ Compute motor commands via PD law
  │   ├─ cmd_pitch = KP*(-ex-ez) - KD*(-wx-wz)
  │   └─ cmd_roll  = KP*(ex-ez)  - KD*(wx-wz)
  └─ Output: ser.set(pitch_control.ctrl_velocity_, cmd_pitch)
             ser.set(roll_control.ctrl_velocity_, cmd_roll)
                ↓
         IqSerial (IQ Module Communication Library)
                ↓
         Serial0 (UART to IQ Motor Controllers)
```

---

## Edge Cases & Safety

1. **Quaternion Singularities**:
   - Handled with epsilon checks (1e-6)
   - 180-degree case: Use orthogonal axis

2. **Velocity Underflow**:
   - If `|v_world| < 0.25 m/s`: Use `launchAxis_world` as fallback

3. **Button Debounce**:
   - 50 ms filter prevents bounce effects (debouncing)

4. **Time Overflow**:
   - Uses `micros()` (32-bit → overflow every ~71 minutes)
   - Not an issue for testing and current scope

5. **Zero Vectors**:
   - Division checks: `if (len < 1e-6)` before normalizing

---

## Telemetry & Debugging

**WAITING Phase Output**:
```
WAITING: accel=0.98g grav=-0.01,-1.00,0.02
```

**Launch Transition**:
```
Burn detected: integrating velocity until accel returns to ambient for ballistic launch.
```

**Burn Phase Output** (high-rate per loop during accel integration):
```
{"t":<ms>,"type":"burn","launch_candidate_ms":<ms>,"quat":[w,x,y,z],"accelBody":[ax,ay,az],"specificForceWorld":[sf_x,sf_y,sf_z],"linearAccel_mps2":[a_x,a_y,a_z],"launchAccel_mps2":<proj>,'"deltaVel_mps":[dv_x,dv_y,dv_z],"launchVel_mps":<v>,"dt":<s>}
```
- `accelBody`: raw IMU acceleration in g, body frame
- `specificForceWorld`: rotated specific force in world frame
- `linearAccel_mps2`: net world acceleration after gravity (m/s²)
- `launchAccel_mps2`: projection of net acceleration along `launchAxis_world`
- `deltaVel_mps`: velocity increment applied for this loop
- `launchVel_mps`: integrated launch-axis velocity magnitude so far

**Ballistic Start**:
```
Ballistic start at 1523 ms. launchAxis=0.00,1.00,0.02 velDir=0.00,1.00,0.02 gravity=0.01,-1.00,-0.00 launchVel=0.95 launchAccel=24.5
```

**Ballistic State** (every 200 ms):
```
BALLISTIC: dt=0.0021 vel=-0.07,-0.26,0.92 | speed=0.96 dir=-0.07,-0.27,0.96 grav=0.25,-0.69,-0.68 ex=58.8 ez=-1.9 -> pitch=118.6 roll=-119.8
```

**Python Telemetry Client**:
- Connects to `192.168.4.1:3333`
- Prints raw received lines with `RECV:` prefix for easier capture of burn and transitional messages
- Writes ballistic and waiting records to `telemetry_log.csv`
- Plots body attitude vs. target velocity direction continuously
- Important diagnostic burn and ballistic JSON messages are also echoed to Serial for local capture

---

## Diagnostic Findings

- Recent burn-phase logs show a real acceleration event: `launchAccel_mps2` is positive and large, and `launchVel_mps` rises from ~0.05 m/s to ~1.18 m/s during the burn.
- The IMU-specific-force path appears consistent; raw accelerometer sign/frame no longer appears to be the primary bug.
- The likely remaining failure mode is in the transition from accel integration to ballistic flight or in the ballistic attitude-control path rather than in raw burn detection and velocity integration.
- Important debugging focuses now on the `Ballistic start` state and the first few `type":"ballistic"` records immediately after burn.

## Current Issues

- The primary remaining issue is a mismatch in the immediate post-burn velocity/velocity-direction state.
- Evidence from logs suggests that `vel_world` and `velDir_world` do not evolve smoothly from the last burn sample into the first ballistic sample.
- The implementation may be corrupting the state during the handoff, either through a timing issue (`lastVelUpdateUs` / `lastGyroUpdateUs`) or by incorrectly applying the fallback `launchAxis_world` velocity.
- Because the burn path now looks sound, the most likely problem area is the ballistic phase logic that propagates velocity under gravity and computes the target attitude.
- The attitude-control PD path can amplify this issue if `velDir_world` is wrong, which would produce large `ex`/`ez` commands even when the robot itself has only small attitude errors.

### Hypotheses

1. `vel_world` is being reset or replaced incorrectly at the moment of transition to `BALLISTIC`.
2. The ballistic update timing is wrong, causing an unexpectedly large velocity jump when integrating with gravity.
3. The fallback velocity assignment using `launchAxis_world` is being triggered incorrectly in marginal burn cases.
4. The telemetry output may be misaligned with the actual state due to timing of the 200 ms reporting interval, so the first few ballistic samples need to be captured carefully.

### Diagnostics Recommended

- Capture the last few `burn` records and the first few `ballistic` records in one sequence.
- Inspect the `Ballistic start` line and compare the reported `vel_world` at burnout to the first ballistic sample.
- Confirm that `gravity_world` remains stable through transition.
- Confirm that `velDir_world` changes gradually, not discontinuously.

---

## Known Limitations & Future Improvements

1. **No Magnetometer Use**: System doesn't use the magnetometer (gravity-only attitude)
2. **No Sensor Fusion**: Could implement extended Kalman filter for better estimates
3. **Fixed Motor Axes**: Currently hardcoded; could be made configurable
4. **Open-Loop Launch Axis**: Doesn't adapt if launch direction deviates
5. **No Trajectory Prediction**: Could predict future velocity for smoother control
6. **Tuning Dependent**: KP, KD, thresholds are hand-tuned constants

---

## Testing & Validation Points

- [ ] IMU reads correctly (verify accel/gyro/quat values in serial output)
- [ ] Button debounce works (no false resets)
- [ ] Gravity detection accurate at different orientations (pre-launch)
- [ ] Launch detection triggers reliably (varies by drop height)
- [ ] Velocity integration correct (compare vs. expected v = at)
- [ ] Motor commands scale with attitude error
- [ ] No numerical instabilities (NaN/Inf in output)
- [ ] Timing constraints met (loop runs fast enough)

---

**Generated**: June 4, 2026
**Status**: Current implementation analysis (pre-review)
