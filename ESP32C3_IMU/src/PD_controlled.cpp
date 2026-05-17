#include <Arduino.h>
#include <Wire.h>
#include <iq_module_communication.hpp>
#include <JY901.h>
#include "pd_controlled.h"

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define BUTTON_PIN 2

// --- Tunable gains ---
const float KP                    = 0.1f;    // proportional: motor units per degree of attitude error
const float KD                    = 0.01f;   // derivative: motor units per deg/s of body rate
const float MAX_MOTOR_VELOCITY    = 150.0f;
const float ANGLE_DEADBAND_DEG    = 1.5f;
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
MultiTurnAngleControlClient pitch_control(1);  // Motor 1: axis between -Z and -X
MultiTurnAngleControlClient roll_control(0);   // Motor 0: axis between -Z and +X

// --- Math types ---
struct Vec3 { float x, y, z; };
struct Quat { float w, x, y, z; };

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
unsigned long lastVelUpdateUs = 0;

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
  lastGyroUpdateUs      = 0;
  lastVelUpdateUs       = 0;
}

void pd_setup() {
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
}

void pd_loop() {
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
  if (!launchDetected) {
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
          lastVelUpdateUs = micros();
          Serial.println("Burn detected: integrating velocity until accel returns to ambient for ballistic launch.");
        }
      } else {
        launchConfirmCount = 0;
      }
    } else {
      // --- Acceleration phase: integrate world-frame velocity ---
      // accel_* is specific force (in g). Linear acceleration = specific_force_world + gravity_world.
      // gravity_world is unit "down". At rest: specific_force_world ≈ -gravity_world -> linear ≈ 0.
      unsigned long nowUs = micros();
      float dt = (nowUs - lastVelUpdateUs) * 1e-6f;
      lastVelUpdateUs = nowUs;
      if (dt > 0.0f && dt < 0.05f) {
        Vec3 accel_body_g = {ax, ay, az};
        Vec3 specific_force_world_g = quatRotVec(candidateQuat, accel_body_g);
        Vec3 linear_accel_world_mps2 = {
          (specific_force_world_g.x + gravity_world.x) * G_MPS2,
          (specific_force_world_g.y + gravity_world.y) * G_MPS2,
          (specific_force_world_g.z + gravity_world.z) * G_MPS2
        };
        vel_world.x += linear_accel_world_mps2.x * dt;
        vel_world.y += linear_accel_world_mps2.y * dt;
        vel_world.z += linear_accel_world_mps2.z * dt;
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

          // Initialize body quaternion from IMU snapshot at burnout
          q_body = candidateQuat;

          // vel_world has been integrated over the acceleration phase; use it as initial ballistic velocity.
          float vLen = sqrtf(vel_world.x*vel_world.x + vel_world.y*vel_world.y + vel_world.z*vel_world.z);
          velDir_world = (vLen > 1e-6f) ? Vec3{vel_world.x/vLen, vel_world.y/vLen, vel_world.z/vLen} : Vec3{0,1,0};

          lastGyroUpdateUs = micros();
          lastVelUpdateUs  = micros();

          Serial.printf("Ballistic start at %lu ms. velDir: %.2f,%.2f,%.2f gravity: %.2f,%.2f,%.2f\n",
                        launchTimeMs,
                        velDir_world.x, velDir_world.y, velDir_world.z,
                        gravity_world.x, gravity_world.y, gravity_world.z);
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
    vel_world.x += gravity_world.x * G_MPS2 * dt_vel;
    vel_world.y += gravity_world.y * G_MPS2 * dt_vel;
    vel_world.z += gravity_world.z * G_MPS2 * dt_vel;

    // 3. Normalize to get current velocity direction
    float vLen = sqrtf(vel_world.x*vel_world.x + vel_world.y*vel_world.y + vel_world.z*vel_world.z);
    if (vLen > 1e-3f) velDir_world = {vel_world.x/vLen, vel_world.y/vLen, vel_world.z/vLen};

    // 4. Target: body -Y (base/foot) points along velocity direction
    //    i.e. body +Y points opposite to velocity
    Vec3 targetBodyY = { -velDir_world.x, -velDir_world.y, -velDir_world.z };

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

  updateMotorCommand(cmd_ex, cmd_ez, wx, wz);

  // --- Telemetry ---
  unsigned long now = millis();
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = now;
    if (launchDetected) {
      float cmd1 = constrain(KP*(-cmd_ex - cmd_ez) - KD*(-wx - wz), -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
      float cmd0 = constrain(KP*( cmd_ex - cmd_ez) - KD*( wx - wz), -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
      Serial.printf("BALLISTIC: velDir=%.2f,%.2f,%.2f ex=%.1f ez=%.1f -> m1=%.1f m0=%.1f\n",
                    velDir_world.x, velDir_world.y, velDir_world.z,
                    cmd_ex, cmd_ez, cmd1, cmd0);
    } else {
      Serial.printf("WAITING: accel=%.2fg grav=%.2f,%.2f,%.2f\n",
                    currentAccelG,
                    gravity_world.x, gravity_world.y, gravity_world.z);
    }
  }
}

void updateMotorCommand(float ex, float ez, float wx, float wz) {
  // Motor 1 axis: (-X -Z)/sqrt2  →  cmd1 = -KP*(ex+ez) - KD*(wx+wz) projected
  // Motor 0 axis: (+X -Z)/sqrt2  →  cmd0 =  KP*(ex-ez) - KD*(wx-wz) projected
  // sqrt(2) absorbed into KP/KD
  float cmd1 = constrain(KP*(-ex - ez) - KD*(-wx - wz), -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
  float cmd0 = constrain(KP*( ex - ez) - KD*( wx - wz), -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
  ser.set(pitch_control.ctrl_velocity_, cmd1);  // Motor 1
  ser.set(roll_control.ctrl_velocity_,  cmd0);  // Motor 0
}
