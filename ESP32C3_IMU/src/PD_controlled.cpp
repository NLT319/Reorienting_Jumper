#include <Arduino.h>
#include <Wire.h>
#include <iq_module_communication.hpp>
#include <JY901.h>
#include "pd_controlled.h"
#include "telemetry.h"

namespace {

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define BUTTON_PIN 2

// --- Tunable gains ---
const float KP                    = 2.0f;    // proportional: motor units per degree of attitude error
const float KD                    = 0.08f;   // derivative: motor units per deg/s of body rate
const float MAX_MOTOR_VELOCITY    = 199.0f;
const float ANGLE_DEADBAND_DEG    = 1.5f;

// flip to + for body flip behavior
const float TARGET_BODY_Y_SIGN     = 1.0f;   // +1: body +Y points along velocity, -1: body -Y points along velocity
const float MIN_INTEGRATED_LAUNCH_SPEED_MPS = 0.25f;
const unsigned long TELEMETRY_INTERVAL_MS = 200;

// --- IMU scaling ---
const float ACCEL_SCALE_G         = 16.0f   / 32768.0f;
const float GYRO_SCALE_DPS        = 2000.0f / 32768.0f;
const float QUAT_SCALE            = 1.0f    / 32768.0f;
const float G_MPS2                = 9.80665f;

// --- Launch detection ---
const float LAUNCH_ACCEL_THRESHOLD_G  = 3.0f;
const float AMBIENT_ACCEL_THRESHOLD_G = 1.15f;
const uint8_t LAUNCH_CONFIRM_SAMPLES  = 3;
const uint8_t AMBIENT_CONFIRM_SAMPLES = 3;
const unsigned long BUTTON_DEBOUNCE_MS = 50;

IqSerial ser(Serial0);
MultiTurnAngleControlClient pitch_control(0);  // Motor axis between -Z and -X
MultiTurnAngleControlClient roll_control(1);   // Motor axis between -Z and +X

// --- Math types ---
struct Vec3 { float x, y, z; };
struct Quat { float w, x, y, z; };

static float vecDot(Vec3 a, Vec3 b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

static float vecNorm(Vec3 v) {
  return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

static Vec3 vecScale(Vec3 v, float s) {
  return {v.x*s, v.y*s, v.z*s};
}

static Vec3 vecNormalizeOrFallback(Vec3 v, Vec3 fallback) {
  float n = vecNorm(v);
  if (n < 1e-6f) return fallback;
  return {v.x/n, v.y/n, v.z/n};
}

static Quat quatMul(Quat p, Quat q) {
  return {
    p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z,
    p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y,
    p.w*q.y - p.x*q.z + p.y*q.w + p.z*q.x,
    p.w*q.z + p.x*q.y - p.y*q.x + p.z*q.w
  };
}
static Quat quatConj(Quat q) { return { q.w, -q.x, -q.y, -q.z }; }
static Quat quatNorm(Quat q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-6f) return {1,0,0,0};
  return { q.w/n, q.x/n, q.y/n, q.z/n };
}
static Vec3 quatRotVec(Quat q, Vec3 v) {
  float tx = 2*(q.y*v.z - q.z*v.y);
  float ty = 2*(q.z*v.x - q.x*v.z);
  float tz = 2*(q.x*v.y - q.y*v.x);
  return {
    v.x + q.w*tx + q.y*tz - q.z*ty,
    v.y + q.w*ty + q.z*tx - q.x*tz,
    v.z + q.w*tz + q.x*ty - q.y*tx
  };
}
// Shortest-path quaternion rotating unit vector a onto unit vector b
static Quat quatFromVecToVec(Vec3 a, Vec3 b) {
  float dot = constrain(a.x*b.x + a.y*b.y + a.z*b.z, -1.0f, 1.0f);
  Vec3 cross = { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
  float crossLen = sqrtf(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
  if (crossLen < 1e-6f) {
    if (dot > 0.9999f) return {1,0,0,0};
    // 180-degree case: pick orthogonal axis
    Vec3 orth = (fabsf(a.x) < 0.9f) ? Vec3{1,0,0} : Vec3{0,1,0};
    Vec3 axis = { a.y*orth.z - a.z*orth.y,
                  a.z*orth.x - a.x*orth.z,
                  a.x*orth.y - a.y*orth.x };
    float len = sqrtf(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
    return { 0, axis.x/len, axis.y/len, axis.z/len };
  }
  Quat q = { 1.0f + dot, cross.x, cross.y, cross.z };
  return quatNorm(q);
}
// Integrate quaternion by body-frame angular velocity (deg/s) over dt (s)
static Quat integrateGyro(Quat q, float wx, float wy, float wz, float dt) {
  const float D2R = M_PI / 180.0f;
  float wx_r = wx*D2R, wy_r = wy*D2R, wz_r = wz*D2R;
  float h = 0.5f * dt;
  Quat dq = {
    -(q.x*wx_r + q.y*wy_r + q.z*wz_r)*h,
     (q.w*wx_r + q.y*wz_r - q.z*wy_r)*h,
     (q.w*wy_r - q.x*wz_r + q.z*wx_r)*h,
     (q.w*wz_r + q.x*wy_r - q.y*wx_r)*h
  };
  return quatNorm(Quat{ q.w+dq.w, q.x+dq.x, q.y+dq.y, q.z+dq.z });
}

// --- State ---
float currentAccelG = 0.0f;

// World-frame gravity unit vector, updated pre-launch from IMU quaternion
Vec3 gravity_world = {0, -1, 0};

// Body orientation in world frame, gyro-integrated from burnout
Quat q_body = {1,0,0,0};
unsigned long lastGyroUpdateUs = 0;

// World-frame velocity vector, gravity-integrated from burnout
Vec3 vel_world   = {0,0,0};
Vec3 velDir_world = {0,1,0};
Vec3 launchAxis_world = {0,1,0};
float lastBallisticDt = 0.0f;
float lastBallisticSpeed = 0.0f;
unsigned long lastVelUpdateUs = 0;
// Diagnostic: last measured accel and specific force (g)
Vec3 lastAccelBody_g = {0,0,0};
Vec3 lastSpecificForceWorld_g = {0,0,0};
// Diagnostic: last applied linear acceleration (m/s^2) and delta-velocity (m/s)
Vec3 lastLinearAccel_mps2 = {0,0,0};
Vec3 lastDeltaVel_mps = {0,0,0};

bool accelReturnedToAmbient = false;
bool launchSpikeSeen = false;
uint8_t ambientConfirmCount = 0;
unsigned long lastTelemetryTime = 0;

bool launchDetected = false;
uint8_t launchConfirmCount = 0;
unsigned long launchTimeMs = 0;
unsigned long candidateLaunchTimeMs = 0;
Quat candidateQuat = {1,0,0,0};

bool lastButtonState = HIGH;
bool stableButtonState = HIGH;
bool lastStableButtonState = HIGH;
unsigned long buttonLastChangeMs = 0;

void updateMotorCommand(float ex, float ez, float wx, float wz);
void computeMotorCommands(float ex, float ez, float wx, float wz, float &pitchCmd, float &rollCmd);

static inline void resetLaunchState() {
  launchDetected        = false;
  launchConfirmCount    = 0;
  accelReturnedToAmbient = false;
  launchSpikeSeen       = false;
  ambientConfirmCount   = 0;
  candidateLaunchTimeMs = 0;
  candidateQuat         = {1,0,0,0};
  q_body                = {1,0,0,0};
  vel_world             = {0,0,0};
  velDir_world          = {0,1,0};
  launchAxis_world      = {0,1,0};
  lastGyroUpdateUs      = 0;
  lastVelUpdateUs       = 0;
  updateMotorCommand(0.0f, 0.0f, 0.0f, 0.0f);
}

void pd_setup_impl() {
  Serial.begin(115200);
  resetLaunchState();
  delay(500);
  Serial.println("ESP32C3 WT901 jumping robot starting...");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  JY901.StartIIC();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastButtonState      = digitalRead(BUTTON_PIN);
  stableButtonState    = lastButtonState;
  lastStableButtonState = stableButtonState;
  buttonLastChangeMs   = millis();

  ser.begin();
  Serial.println("IQ serial initialized");
  telemetryStartAP();
  updateMotorCommand(0.0f, 0.0f, 0.0f, 0.0f);
}

void pd_loop_impl() {
  // --- Read accelerometer ---
  JY901.GetAcc();
  float ax = (float)JY901.stcAcc.a[0] * ACCEL_SCALE_G;
  float ay = (float)JY901.stcAcc.a[1] * ACCEL_SCALE_G;
  float az = (float)JY901.stcAcc.a[2] * ACCEL_SCALE_G;
  currentAccelG = sqrtf(ax*ax + ay*ay + az*az);

  // --- Read gyro ---
  JY901.GetGyro();
  float wx = (float)JY901.stcGyro.w[0] * GYRO_SCALE_DPS;
  float wy = (float)JY901.stcGyro.w[1] * GYRO_SCALE_DPS;
  float wz = (float)JY901.stcGyro.w[2] * GYRO_SCALE_DPS;

  // --- Read IMU quaternion (idle + acceleration phase, for gravity capture & burnout snapshot) ---
  // Quaternion: q[0]=w, q[1]=x, q[2]=y, q[3]=z, scaled /32768
  if (!launchDetected && !launchSpikeSeen) {
    JY901.GetQuaternion();
    Quat q_imu = quatNorm(Quat{
      (float)JY901.stcQuater.q0 * QUAT_SCALE,
      (float)JY901.stcQuater.q1 * QUAT_SCALE,
      (float)JY901.stcQuater.q2 * QUAT_SCALE,
      (float)JY901.stcQuater.q3 * QUAT_SCALE
    });

    // At rest, accelerometer reads body +Y ≈ +1g (specific force, "up").
    // Rotate that into world, then negate to get gravity_world (unit "down" direction).
    Vec3 bodyUp = {0, 1, 0};
    Vec3 up_world = quatRotVec(q_imu, bodyUp);
    float uLen = sqrtf(up_world.x*up_world.x + up_world.y*up_world.y + up_world.z*up_world.z);
    if (uLen > 1e-6f) {
      up_world = {up_world.x/uLen, up_world.y/uLen, up_world.z/uLen};
      gravity_world = {-up_world.x, -up_world.y, -up_world.z};
    }

    // Snapshot the quaternion continuously as launch candidate
    candidateQuat = q_imu;
  }

  // --- Button debounce & launch reset ---
  bool buttonState = digitalRead(BUTTON_PIN);
  unsigned long buttonNow = millis();
  if (buttonState != lastButtonState) {
    lastButtonState = buttonState;
    buttonLastChangeMs = buttonNow;
  }
  if ((buttonNow - buttonLastChangeMs) > BUTTON_DEBOUNCE_MS && buttonState != stableButtonState) {
    stableButtonState = buttonState;
    if (stableButtonState == LOW && lastStableButtonState == HIGH) {
      resetLaunchState();
      Serial.println("Launch reset: waiting for next launch event.");
    }
    lastStableButtonState = stableButtonState;
  }

  // --- Launch detection state machine (unchanged from working version) ---
  if (!launchDetected) {
    if (!accelReturnedToAmbient) {
      if (currentAccelG <= AMBIENT_ACCEL_THRESHOLD_G) {
        ambientConfirmCount++;
        if (ambientConfirmCount >= AMBIENT_CONFIRM_SAMPLES) {
          accelReturnedToAmbient = true;
          ambientConfirmCount = 0;
        }
      } else {
        ambientConfirmCount = 0;
      }
    } else if (!launchSpikeSeen) {
      if (currentAccelG >= LAUNCH_ACCEL_THRESHOLD_G) {
        launchConfirmCount++;
        if (launchConfirmCount >= LAUNCH_CONFIRM_SAMPLES) {
          launchSpikeSeen = true;
          launchConfirmCount = 0;
          ambientConfirmCount = 0;
          // Begin integrating velocity over the acceleration phase only.
          vel_world = {0,0,0};
          q_body = candidateQuat;
          launchAxis_world = vecNormalizeOrFallback(quatRotVec(candidateQuat, Vec3{0,1,0}), vecScale(gravity_world, -1.0f));
          if (vecDot(launchAxis_world, gravity_world) > 0.0f) {
            launchAxis_world = vecScale(launchAxis_world, -1.0f);
          }
          lastGyroUpdateUs = micros();
          lastVelUpdateUs = lastGyroUpdateUs;
          Serial.println("Burn detected: integrating velocity until accel returns to ambient for ballistic launch.");
        }
      } else {
        launchConfirmCount = 0;
      }
    } else {
      // --- Acceleration phase: integrate body orientation and world-frame velocity ---
      // accel_* is specific force (in g). Linear acceleration = specific_force_world + gravity_world.
      // gravity_world is unit "down". At rest: specific_force_world ≈ -gravity_world -> linear ≈ 0.
      unsigned long nowUs = micros();
      float dt = (nowUs - lastVelUpdateUs) * 1e-6f;
      float dt_gyro = 0.0f;
      if (lastGyroUpdateUs != 0) {
        dt_gyro = (nowUs - lastGyroUpdateUs) * 1e-6f;
      }
      if (dt_gyro > 0.0f && dt_gyro < 0.05f) {
        q_body = integrateGyro(q_body, wx, wy, wz, dt_gyro);
      }
      lastGyroUpdateUs = nowUs;
      lastVelUpdateUs = nowUs;
      if (dt > 0.0f && dt < 0.05f) {
        // Use raw accelerometer specific force as provided by IMU (in g), rotate into world.
        Vec3 accel_body_g = {ax, ay, az};
        Vec3 specific_force_world_g = quatRotVec(q_body, accel_body_g);
        // store for telemetry diagnosis
        lastAccelBody_g = accel_body_g;
        lastSpecificForceWorld_g = specific_force_world_g;
        Vec3 linear_accel_world_mps2 = {
          (specific_force_world_g.x + gravity_world.x) * G_MPS2,
          (specific_force_world_g.y + gravity_world.y) * G_MPS2,
          (specific_force_world_g.z + gravity_world.z) * G_MPS2
        };
        // apply and record
        lastLinearAccel_mps2 = linear_accel_world_mps2;
        lastDeltaVel_mps = { linear_accel_world_mps2.x * dt, linear_accel_world_mps2.y * dt, linear_accel_world_mps2.z * dt };
        vel_world.x += lastDeltaVel_mps.x;
        vel_world.y += lastDeltaVel_mps.y;
        vel_world.z += lastDeltaVel_mps.z;
      }

      // Emit high-rate burn telemetry while in the acceleration phase to aid capture
      {
        unsigned long nowMs = millis();
        float launchAccel = vecDot(lastLinearAccel_mps2, launchAxis_world);
        float launchVel = vecDot(vel_world, launchAxis_world);
        char burnBuf[640];
        snprintf(burnBuf, sizeof(burnBuf), "{\"t\":%lu,\"type\":\"burn\",\"launch_candidate_ms\":%lu,\"quat\":[%.6f,%.6f,%.6f,%.6f],\"accelBody\":[%.6f,%.6f,%.6f],\"specificForceWorld\":[%.6f,%.6f,%.6f],\"linearAccel_mps2\":[%.6f,%.6f,%.6f],\"launchAccel_mps2\":%.6f,\"deltaVel_mps\":[%.6f,%.6f,%.6f],\"launchVel_mps\":%.6f,\"dt\":%.6f}",
                 nowMs, candidateLaunchTimeMs,
                 q_body.w, q_body.x, q_body.y, q_body.z,
                 lastAccelBody_g.x, lastAccelBody_g.y, lastAccelBody_g.z,
                 lastSpecificForceWorld_g.x, lastSpecificForceWorld_g.y, lastSpecificForceWorld_g.z,
                 lastLinearAccel_mps2.x, lastLinearAccel_mps2.y, lastLinearAccel_mps2.z,
                 launchAccel,
                 lastDeltaVel_mps.x, lastDeltaVel_mps.y, lastDeltaVel_mps.z,
                 launchVel,
                 dt);
        Serial.println(burnBuf);
        telemetrySend(burnBuf);
      }

      if (currentAccelG <= AMBIENT_ACCEL_THRESHOLD_G) {
        if (ambientConfirmCount == 0) {
          candidateLaunchTimeMs = millis();
          // candidateQuat is already being updated every loop above
        }
        ambientConfirmCount++;
        if (ambientConfirmCount >= AMBIENT_CONFIRM_SAMPLES) {
          launchDetected = true;
          launchTimeMs   = candidateLaunchTimeMs;

          // Keep the current integrated body quaternion at burnout.
          // q_body has been integrated during the burn phase.

          float vLen = vecNorm(vel_world);
          if (vLen < MIN_INTEGRATED_LAUNCH_SPEED_MPS || vecDot(vel_world, launchAxis_world) < 0.0f) {
            float launchSpeed = (vLen > MIN_INTEGRATED_LAUNCH_SPEED_MPS) ? vLen : MIN_INTEGRATED_LAUNCH_SPEED_MPS;
            vel_world = vecScale(launchAxis_world, launchSpeed);
            vLen = vecNorm(vel_world);
          }
          velDir_world = (vLen > 1e-6f) ? Vec3{vel_world.x/vLen, vel_world.y/vLen, vel_world.z/vLen} : launchAxis_world;

          lastGyroUpdateUs = micros();
          lastVelUpdateUs  = micros();

          float launchProjVel = vecDot(vel_world, launchAxis_world);
          float launchProjAccel = vecDot(lastLinearAccel_mps2, launchAxis_world);
          Serial.printf("Ballistic start at %lu ms. launchAxis=%.2f,%.2f,%.2f velDir=%.2f,%.2f,%.2f grav=%.2f,%.2f,%.2f launchVel=%.3f launchAccel=%.3f\n",
                        launchTimeMs,
                        launchAxis_world.x, launchAxis_world.y, launchAxis_world.z,
                        velDir_world.x, velDir_world.y, velDir_world.z,
                        gravity_world.x, gravity_world.y, gravity_world.z,
                        launchProjVel,
                        launchProjAccel);
        }
      } else {
        ambientConfirmCount = 0;
      }
    }
  }

  // --- Ballistic phase: attitude control toward velocity vector ---
  float cmd_ex = 0.0f;
  float cmd_ez = 0.0f;

  if (launchDetected) {
    unsigned long nowUs = micros();

    // 1. Propagate body orientation via gyro integration
    float dt_gyro = (nowUs - lastGyroUpdateUs) * 1e-6f;
    lastGyroUpdateUs = nowUs;
    q_body = integrateGyro(q_body, wx, wy, wz, dt_gyro);

    // 2. Evolve velocity vector under constant gravity
    float dt_vel = (nowUs - lastVelUpdateUs) * 1e-6f;
    lastVelUpdateUs = nowUs;
    // gravity-only evolution during ballistic
    Vec3 gravityAccel = { gravity_world.x * G_MPS2, gravity_world.y * G_MPS2, gravity_world.z * G_MPS2 };
    lastLinearAccel_mps2 = gravityAccel;
    lastDeltaVel_mps = { gravityAccel.x * dt_vel, gravityAccel.y * dt_vel, gravityAccel.z * dt_vel };
    vel_world.x += lastDeltaVel_mps.x;
    vel_world.y += lastDeltaVel_mps.y;
    vel_world.z += lastDeltaVel_mps.z;

    // 3. Normalize to get current velocity direction
    float vLen = sqrtf(vel_world.x*vel_world.x + vel_world.y*vel_world.y + vel_world.z*vel_world.z);
    if (vLen > 1e-3f) {
      velDir_world = {vel_world.x/vLen, vel_world.y/vLen, vel_world.z/vLen};
    }
    lastBallisticDt = dt_vel;
    lastBallisticSpeed = vLen;

    // 4. Target: selected body Y direction points along velocity direction.
    Vec3 targetBodyY = {
      TARGET_BODY_Y_SIGN * velDir_world.x,
      TARGET_BODY_Y_SIGN * velDir_world.y,
      TARGET_BODY_Y_SIGN * velDir_world.z
    };

    // 5. Current body +Y in world frame
    Vec3 currentBodyY = quatRotVec(q_body, Vec3{0,1,0});

    // 6. Shortest-path quaternion error in world frame
    Quat q_err_world = quatFromVecToVec(currentBodyY, targetBodyY);

    // 7. Express error in body frame
    Quat q_err_body = quatMul(quatConj(q_body), quatMul(q_err_world, q_body));
    q_err_body = quatNorm(q_err_body);

    // 8. Extract X and Z error components (axis * angle in degrees)
    float half_angle = acosf(constrain(q_err_body.w, -1.0f, 1.0f));
    float sin_half   = sinf(half_angle);
    float err_deg    = 2.0f * half_angle * 180.0f / M_PI;
    float axis_x = (sin_half > 1e-6f) ? q_err_body.x / sin_half : 0.0f;
    float axis_z = (sin_half > 1e-6f) ? q_err_body.z / sin_half : 0.0f;

    float ex_err = err_deg * axis_x;
    float ez_err = err_deg * axis_z;

    cmd_ex = (fabs(ex_err) >= ANGLE_DEADBAND_DEG) ? ex_err : 0.0f;
    cmd_ez = (fabs(ez_err) >= ANGLE_DEADBAND_DEG) ? ez_err : 0.0f;
  }

  if (launchDetected) {
    updateMotorCommand(cmd_ex, cmd_ez, wx, wz);
  } else {
    updateMotorCommand(0.0f, 0.0f, 0.0f, 0.0f);
  }

  // --- Telemetry ---
  unsigned long now = millis();
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = now;
    if (launchDetected) {
      float pitchCmd = 0.0f;
      float rollCmd = 0.0f;
      computeMotorCommands(cmd_ex, cmd_ez, wx, wz, pitchCmd, rollCmd);
      Serial.printf("BALLISTIC: dt=%.4f vel=%.2f,%.2f,%.2f | speed=%.2f dir=%.2f,%.2f,%.2f grav=%.2f,%.2f,%.2f ex=%.1f ez=%.1f -> pitch=%.1f roll=%.1f\n",
                    lastBallisticDt,
                    vel_world.x, vel_world.y, vel_world.z,
                    lastBallisticSpeed,
                    velDir_world.x, velDir_world.y, velDir_world.z,
                    gravity_world.x, gravity_world.y, gravity_world.z,
                    cmd_ex, cmd_ez, pitchCmd, rollCmd);
      char buf[1024];
      // Include launchAxis, lastBallisticDt, and diagnostic accel/specific-force for debugging
      snprintf(buf, sizeof(buf), "{\"t\":%lu,\"type\":\"ballistic\",\"launch_ms\":%lu,\"quat\":[%.6f,%.6f,%.6f,%.6f],\"vel\":[%.6f,%.6f,%.6f],\"velDir\":[%.6f,%.6f,%.6f],\"launchAxis\":[%.6f,%.6f,%.6f],\"gravity\":[%.6f,%.6f,%.6f],\"accelBody\":[%.6f,%.6f,%.6f],\"specificForceWorld\":[%.6f,%.6f,%.6f],\"linearAccel_mps2\":[%.6f,%.6f,%.6f],\"deltaVel_mps\":[%.6f,%.6f,%.6f],\"dt\":%.6f}",
           now, launchTimeMs,
           q_body.w, q_body.x, q_body.y, q_body.z,
           vel_world.x, vel_world.y, vel_world.z,
           velDir_world.x, velDir_world.y, velDir_world.z,
           launchAxis_world.x, launchAxis_world.y, launchAxis_world.z,
           gravity_world.x, gravity_world.y, gravity_world.z,
           lastAccelBody_g.x, lastAccelBody_g.y, lastAccelBody_g.z,
           lastSpecificForceWorld_g.x, lastSpecificForceWorld_g.y, lastSpecificForceWorld_g.z,
           lastLinearAccel_mps2.x, lastLinearAccel_mps2.y, lastLinearAccel_mps2.z,
           lastDeltaVel_mps.x, lastDeltaVel_mps.y, lastDeltaVel_mps.z,
           lastBallisticDt);
      Serial.println(buf);
      telemetrySend(buf);
    } else {
      Serial.printf("WAITING: accel=%.2fg grav=%.2f,%.2f,%.2f\n",
                    currentAccelG,
                    gravity_world.x, gravity_world.y, gravity_world.z);
      char buf[128];
      snprintf(buf, sizeof(buf), "{\"t\":%lu,\"type\":\"waiting\",\"accel\":%.6f}", now, currentAccelG);
      telemetrySend(buf);
    }
  }
}

void updateMotorCommand(float ex, float ez, float wx, float wz) {
  // Motor 1 axis: (-X -Z)/sqrt2  →  cmd1 = -KP*(ex+ez) - KD*(wx+wz) projected
  // Motor 0 axis: (+X -Z)/sqrt2  →  cmd0 =  KP*(ex-ez) - KD*(wx-wz) projected
  // sqrt(2) absorbed into KP/KD
  float pitchCmd = 0.0f;
  float rollCmd = 0.0f;
  computeMotorCommands(ex, ez, wx, wz, pitchCmd, rollCmd);
  ser.set(pitch_control.ctrl_velocity_, pitchCmd);
  ser.set(roll_control.ctrl_velocity_, rollCmd);
}

void computeMotorCommands(float ex, float ez, float wx, float wz, float &pitchCmd, float &rollCmd) { 
  float cmd1 = constrain(KP*(-ex - ez) - KD*(-wx - wz), -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
  float cmd0 = constrain(KP*( ex - ez) - KD*( wx - wz), -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
  pitchCmd = -cmd1;
  rollCmd = -cmd0;
}

}

void pd_setup() {
  pd_setup_impl();
}

void pd_loop() {
  pd_loop_impl();
}
