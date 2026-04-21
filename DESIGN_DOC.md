# Ballistic Jumper Control System - Design Document

## Executive Summary

This document describes the firmware architecture for the **Ballistic Jumper** robotics platform. The system uses a reaction wheel (flywheel) to induce a controlled rotation during flight, allowing the jumper to land at a predetermined orientation (180 deg flip from launch).

The control strategy is straightforward:
1. **Spin-up phase**: Motor accelerates the flywheel to a target velocity (3 seconds)
2. **Free flight**: Jumper launches at a known angle and velocity
3. **Braking phase**: At the precise moment, the flywheel is mechanically braked (coils shorted), transferring angular momentum to the body
4. **Landing**: Jumper lands inverted due to the induced rotation

---

## System Architecture

### Hardware Components

- **ESP32C3 Microcontroller**: Real-time control, button monitoring, motor communication
- **IQ Module Motor Controller**: Brushless motor with integrated control (velocity, torque, brake modes)
- **Launch Mechanism**: Manual mechanical launcher with variable spring tension
- **Button Input (GPIO 2)**: Initiates launch sequence (3-second countdown)
- **Flywheel System**: Reaction wheel with known inertia properties

### Communication Protocol

- **Serial0 (UART)**: Communication with IQ Module using proprietary `iq_module_communication.hpp` library
- **Serial (USB)**: Debug output and logging to host computer

---

## Control Flow Diagram

```
Power On
    ↓
[IDLE] - Waiting for button press
    ↓ (Button pressed)
[COUNTDOWN] - 3 second motor spin-up
    ↓ (3s elapsed)
[BRAKING] - Monitor time, apply brake at calculated moment
    ↓ (Landing detected)
[COMPLETE] - Motor off, awaiting reset
```

### State Descriptions

#### IDLE
- **Duration**: Indefinite
- **Motor Command**: Brake (motor off, coils shorted if applicable)
- **Trigger**: Button press (GPIO2 = LOW with INPUT_PULLUP)
- **Action**: Log event, transition to COUNTDOWN, record launch start time

#### COUNTDOWN
- **Duration**: 3 seconds (configurable via `SPINUP_TIME`)
- **Motor Command**: Velocity control to `TARGET_VELOCITY`
- **Trigger**: Elapsed time >= 3000 ms
- **Action**: Calculate landing and brake times, transition to BRAKING

#### BRAKING
- **Duration**: From t=3s until landing is detected
- **Motor Command**: 
  - Before brake time: Coast at TARGET_VELOCITY
  - At brake time: Full brake (motor coils shorted)
- **Triggers**:
  - Brake application: Elapsed time >= `calculatedBrakeTime`
  - Landing detection: Elapsed time >= `calculatedLandingTime` + 500ms buffer
- **Action**: Apply brake, monitor for landing

#### COMPLETE
- **Motor Command**: Brake (motor off)
- **Duration**: Until power cycle

---

## Physics Model

### Coordinate System & Conventions

- **theta**: Body orientation angle
  - theta = 0 deg: Vertical (upright)
  - theta = 45 deg: Launch angle
  - theta = 135 deg: Landing angle (180 deg flip)
  
- **Body**: Rigid rod of length Lb, mass Mb, initial orientation theta_0 = 45 deg
- **Flywheel**: Inertia Jw, initially spinning at omega_0 = bdphi0 (rad/s)

### Key Physics Equations

#### 1. Ballistic Trajectory (COM Height)
```
y_COM(t) = y0 + v_y0 * t - 0.5 * g * t^2
```

Where:
- y0 = Initial COM height (adjusted for body angle)
- v_y0 = Initial vertical velocity = v0 * sin(theta_0)
- g = 9.81 m/s^2

#### 2. Landing Time Calculation
The jumper lands when the foot (tip of body) touches the ground:

```
y_COM(t_land) - (Lb/2) * sin(theta_final) = 0
```

Solving the quadratic:
```
t_land = max(roots of: -0.5*g*t^2 + v_y0*t + (y0_adjusted - y_COM_required))
```

#### 3. Angular Momentum Conservation
During braking, angular momentum is conserved:
```
H = J_w * omega_wheel = const
```

After the wheel is fully braked:
```
omega_body_post = H / J_total = (J_w * omega_0) / J_total
```

#### 4. Brake Trigger Time
For the body to reach the target angle at landing:
```
Delta_theta = omega_body_post * (t_land - t_brake)

t_brake = t_land - (Delta_theta / omega_body_post)
```

Where:
- Delta_theta = theta_final - theta_initial = (180 deg - 45 deg) = 135 deg

---

## Parameter Configuration

All system parameters are defined in `include/jumper_params.h` as compile-time constants.

### Physical Parameters

| Parameter | Symbol | Value | Units | Notes |
|-----------|--------|-------|-------|-------|
| Body Mass | MB | 0.16 | kg | |
| Body Length | LB | 0.00235 | m | Distance between COM and tip |
| Wheel Mass | MW | 0.02 | kg | |
| Wheel Radius | RW | 0.16 | m | |
| Body Inertia | JB | Auto | kg*m^2 | Calculated: (1/12)*MB*LB^2 |
| Wheel Inertia | JW | Auto | kg*m^2 | Calculated: (1/2)*MW*RW^2 |
| Wheel-Body Offset | LBW | 0 | m | Assumed at COM for simplicity |

### Control Parameters

| Parameter | Symbol | Value | Units | Notes |
|-----------|--------|-------|-------|-------|
| Jump Angle | JUMP_ANGLE | 45 | deg | Launch angle above horizontal |
| Jump Velocity | JUMP_VELOCITY | 20 | m/s | Initial velocity magnitude |
| Spin-up Time | SPINUP_TIME | 3 | s | Countdown before launch |
| Target Motor Velocity | TARGET_VELOCITY | 60 | m/s or motor units | Velocity setpoint during spin-up |
| Initial Flywheel Speed | bdphi0 | 2 | rad/s | Derived from angular momentum requirement |
| Brake Duration | T_BRAKE | 0.1 | s | How long brake torque is applied |
| Gravity | G | 9.81 | m/s^2 | Earth's gravitational acceleration |

### Modifying Parameters

To adjust system behavior for mechanical design iterations:

1. **Mechanical changes**: Update `MB`, `MW`, `RW` in `jumper_params.h`
2. **Jump characteristics**: Adjust `JUMP_VELOCITY`, `JUMP_ANGLE`
3. **Motor tuning**: Modify `TARGET_VELOCITY`, `SPINUP_TIME`
4. **Braking response**: Adjust `T_BRAKE` (note: motor must support this duration)

After any parameter change, recompile and verify with MATLAB simulation before deployment.

---

## Non-Blocking Implementation

### Why Non-Blocking?

The firmware uses a non-blocking state machine rather than `delay()` statements:

- **Responsiveness**: Button presses are detected immediately
- **Sensor polling**: Motor feedback is read continuously without blocking
- **Scalability**: Easy to add parallel features (LEDs, IMU, logging) without refactoring

### Key Timing Mechanisms

#### Elapsed Time Tracking
```cpp
unsigned long launchStartTime;      // When button was pressed
unsigned long elapsedTime = millis() - launchStartTime;
```

#### State Transitions Based on Time
```cpp
if (elapsedTime >= SPINUP_TIME * 1000UL) {
  currentState = BRAKING;
  // Calculate brake and landing times
}
```

#### Periodic Sensor Reads
```cpp
static unsigned long lastVoltLog = 0;
if (millis() - lastVoltLog > 500) {
  // Read sensor, log every 500ms instead of every loop
  lastVoltLog = millis();
}
```

### Loop Timing
- **Main loop delay**: 50 ms (20 Hz loop rate)
- **Serial communication**: Asynchronous via IQ library
- **Sensor reads**: As-available (IQ module responds when data is ready)

---

## Serial Output & Debugging

### Initialization Block
On power-up, the system prints:
```
[startup info]
Jump Angle: 45 deg
Jump Velocity: 20 m/s
Body Inertia (Jb): 0.000001 kg*m^2
...
System Ready. Press button to launch.
```

### Event Logging
State changes and key events are timestamped:
```
[1245 ms] STATE: COUNTDOWN
[1250 ms] EVENT: Button pressed - starting 3s spin-up
[4245 ms] STATE: BRAKING
[4250 ms] EVENT: Calculated landing time: 1234.5 ms
[4255 ms] EVENT: Calculated brake trigger time: 456.7 ms
```

### Periodic Sensor Output (During Braking)
```
V: 12.35V
V: 12.34V
```

### Advantages for Development
- **Timestamps**: Correlate with external logging (video, IMU) for debugging
- **State visibility**: Know exactly what the controller is doing
- **Parameter validation**: Verify calculations before deployment

---

## Motor Control Strategy

### IQ Module Command Set

The code uses the following IQ Module clients:

| Client | Command | Effect |
|--------|---------|--------|
| `multi_control.ctrl_velocity_` | Set value | Ramp motor to target velocity |
| `multi_control.ctrl_brake_` | Execute | Apply maximum braking (short coils) |
| `power.volts_` | Get value | Read system voltage |
| `brushless_drive.obs_velocity_` | Get value | Observe motor velocity |

### State-Based Control

- **IDLE**: `ctrl_brake_` → Motor off
- **COUNTDOWN**: `ctrl_velocity_(TARGET_VELOCITY)` → Ramp to target
- **BRAKING (pre-brake)**: `ctrl_velocity_(TARGET_VELOCITY)` → Hold velocity
- **BRAKING (post-brake)**: `ctrl_brake_` → Apply coil shorting
- **COMPLETE**: `ctrl_brake_` → Motor off

### Brake Behavior

When `ctrl_brake_` is applied:
- Motor coils are shorted together
- Back-EMF creates a resistive torque proportional to velocity
- Wheel decelerates rapidly
- Momentum is transferred to the body (angular momentum conservation)

**Important**: The IQ motor must support continuous braking for the full brake duration (~0.1s). Verify thermal limits during prototyping.

---

## Future Enhancements

### Planned Features

1. **LED Status Indicators**
   - Red: IDLE (waiting)
   - Yellow: COUNTDOWN (spinning up)
   - Green: BRAKING (flight)
   - Blue: COMPLETE (landed)
   - Possible: Countdown LEDs showing T-3, T-2, T-1 seconds

2. **Onboard IMU**
   - Verify actual jump angle matches expected
   - Detect landing event instead of relying on timer
   - Adjust brake time dynamically if launch angle is off

3. **Parameter Adjustment**
   - EEPROM storage for rapid tuning
   - Serial command interface to adjust parameters without recompiling
   - Store multiple "flight profiles" for different mechanical configurations

4. **Motor Feedback Tuning**
   - Closed-loop velocity control during spin-up
   - Adaptive ramp rate to reach TARGET_VELOCITY smoothly
   - Verify motor is ready before launch countdown

5. **Energy Optimization**
   - Calculate minimum flywheel velocity needed (not always bdphi0)
   - Reduce spin-up time/power consumption
   - Trade-off: Requires more precise mechanics

### Code Refactoring Considerations

- **Separate files**: Move physics calculations to `jumper_physics.cpp`
- **LED library**: Create `jumper_leds.h` for status indicators
- **Configuration object**: Build dynamic `JumperConfig` from EEPROM
- **Unit tests**: MATLAB validation for each physics function

---

## Testing & Validation Protocol

### Pre-Flight Checklist

1. **Parameter Validation**
   - [ ] Compare MATLAB simulation results with calculated landing/brake times
   - [ ] Print results via serial during COUNTDOWN to BRAKING transition
   - [ ] Verify times are reasonable (t_brake < t_land)

2. **Hardware Verification**
   - [ ] Motor spins smoothly during COUNTDOWN
   - [ ] Brake engages when triggered (listen for coil shorting)
   - [ ] Button input is stable (no spurious triggers)
   - [ ] Serial communication is working

3. **Dry Run (No Actual Jump)**
   - [ ] Press button, observe full state sequence
   - [ ] Log timing data from serial output
   - [ ] Verify brake is applied at expected time

4. **Post-Flight Analysis**
   - [ ] Record video at high FPS (240+ fps)
   - [ ] Measure actual landing angle visually
   - [ ] Compare with calculated angle
   - [ ] Iterate if error exceeds tolerance (+/- 5 deg acceptable for prototype)

---

## Troubleshooting

### Symptoms & Solutions

| Symptom | Possible Cause | Solution |
|---------|---|---|
| Button press not detected | GPIO pin mismatch, pull-up issue | Verify INPUT_PULLUP works with multimeter |
| Motor doesn't spin up | Serial communication broken, velocity command ignored | Check IQ library initialization, serial baud rate |
| Landing angle is wrong | Incorrect parameter values, wrong formula | Validate against MATLAB, print intermediate calculations |
| Brake not applied | State machine bug, command not sent | Add debug logs around `brakeActive` flag |
| Serial output glitchy | Baud rate mismatch, USB cable quality | Try different cable, check `Serial.begin(115200)` |

---

## Code Organization

```
Reorienting_Jumper/
├── ESP32C3/
│   ├── platformio.ini              # PlatformIO configuration
│   ├── include/
│   │   └── jumper_params.h         # System parameters & initialization
│   └── src/
│       └── main.cpp                # Main control firmware
├── DESIGN_DOC.md                   # This file
├── MATLAB/                         # Simulation code (not included here)
└── README.md                       # Project overview
```

---

## References

- **Angular Momentum Conservation**: Goldstein, H. *Classical Mechanics* (3rd Ed.), Chapter 5
- **Ballistic Trajectory**: Standard kinematic equations with gravity
- **IQ Module API**: Included in `iq_module_communication.hpp`
- **MATLAB Simulations**: `braking_controller.m`, `landing_time_tip.m`

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-04-17 | Control System | Initial design document and firmware |

---

## Document Maintenance

This design document should be updated whenever:
- System parameters are changed
- New features are added
- Control algorithm is modified
- Hardware changes are made
- Testing reveals new insights