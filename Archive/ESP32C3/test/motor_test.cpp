// Archived motor test firmware (not compiled by PlatformIO by default).
//
// This file contains a previous "tilt -> motor mixing" tester used to validate both motors
// and the IQUART device IDs. It is kept here as reference only.
//
// To run it, copy this file over `src/main.cpp` (or create a build flag / separate env).

#include <Arduino.h>
#include <Wire.h>
#include <iq_module_communication.hpp>
#include <JY901.h>

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define BUTTON_PIN 2

const float ANGLE_TO_VELOCITY_KP = 0.1f;
const float MAX_MOTOR_VELOCITY = 150.0f;
const float ANGLE_DEADBAND_DEG = 1.5f;
const unsigned long TELEMETRY_INTERVAL_MS = 200;

const float ACCEL_SCALE_G = 16.0f / 32768.0f;
const float LAUNCH_ACCEL_THRESHOLD_G = 3.0f;
const float AMBIENT_ACCEL_THRESHOLD_G = 1.15f;
const uint8_t LAUNCH_CONFIRM_SAMPLES = 3;
const uint8_t AMBIENT_CONFIRM_SAMPLES = 3;
const unsigned long BUTTON_DEBOUNCE_MS = 50;

IqSerial ser(Serial0);
MultiTurnAngleControlClient pitch_control(1);  // Motor 1: axis between -Z and -X
MultiTurnAngleControlClient roll_control(0);   // Motor 0: axis between -Z and +X

float currentAccelG = 0.0f;
float exTilt = 0.0f;
float ezTilt = 0.0f;
bool accelReturnedToAmbient = false;
bool launchSpikeSeen = false;
uint8_t ambientConfirmCount = 0;
unsigned long lastTelemetryTime = 0;

bool launchDetected = false;
uint8_t launchConfirmCount = 0;
unsigned long launchTimeMs = 0;
float launchAngle1 = 0.0f;
float launchAngle2 = 0.0f;
unsigned long candidateLaunchTimeMs = 0;
float candidateAngle1 = 0.0f;
float candidateAngle2 = 0.0f;

bool lastButtonState = HIGH;
bool stableButtonState = HIGH;
bool lastStableButtonState = HIGH;
unsigned long buttonLastChangeMs = 0;

void updateMotorCommand(float ex, float ez);

static inline void resetLaunchState() {
  launchDetected = false;
  launchConfirmCount = 0;
  accelReturnedToAmbient = false;
  launchSpikeSeen = false;
  ambientConfirmCount = 0;
  candidateLaunchTimeMs = 0;
  candidateAngle1 = 0.0f;
  candidateAngle2 = 0.0f;
}

void setup() {
  Serial.begin(115200);
  resetLaunchState();
  delay(500);
  Serial.println("ESP32C3 WT901B IMU motor control starting...");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  JY901.StartIIC();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastButtonState = digitalRead(BUTTON_PIN);
  stableButtonState = lastButtonState;
  lastStableButtonState = stableButtonState;
  buttonLastChangeMs = millis();

  ser.begin();
  Serial.println("IQ serial initialized");
}

void loop() {
  JY901.GetAcc();
  float ax = (float)JY901.stcAcc.a[0] * ACCEL_SCALE_G;
  float ay = (float)JY901.stcAcc.a[1] * ACCEL_SCALE_G;
  float az = (float)JY901.stcAcc.a[2] * ACCEL_SCALE_G;
  currentAccelG = sqrtf(ax * ax + ay * ay + az * az);

  // Derive tilt directly from accelerometer in Y-up frame.
  float xTilt = atan2f(ax, ay) * 180.0f / M_PI;
  float zTilt = atan2f(az, ay) * 180.0f / M_PI;

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

  // --- Launch detection state machine ---
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
          Serial.println("Burn detected: waiting for accel to return to ambient for ballistic launch.");
        }
      } else {
        launchConfirmCount = 0;
      }
    } else {
      if (currentAccelG <= AMBIENT_ACCEL_THRESHOLD_G) {
        if (ambientConfirmCount == 0) {
          candidateLaunchTimeMs = millis();
          candidateAngle1 = xTilt;
          candidateAngle2 = zTilt;
        }
        ambientConfirmCount++;
        if (ambientConfirmCount >= AMBIENT_CONFIRM_SAMPLES) {
          launchDetected = true;
          launchTimeMs = candidateLaunchTimeMs;
          launchAngle1 = candidateAngle1;
          launchAngle2 = candidateAngle2;
          Serial.printf("Launch (ballistic) detected at %lu ms: X-tilt=%.2f Z-tilt=%.2f accel=%.2f g\n",
                        launchTimeMs, launchAngle1, launchAngle2, currentAccelG);
        }
      } else {
        ambientConfirmCount = 0;
      }
    }
  }

  exTilt = 0.0f;
  ezTilt = 0.0f;
  if (launchDetected) {
    exTilt = (fabs(xTilt) >= ANGLE_DEADBAND_DEG) ? xTilt : 0.0f;
    ezTilt = (fabs(zTilt) >= ANGLE_DEADBAND_DEG) ? zTilt : 0.0f;
  }
  updateMotorCommand(exTilt, ezTilt);

  unsigned long now = millis();
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = now;
    float cmd1 = constrain(ANGLE_TO_VELOCITY_KP * (-exTilt - ezTilt), -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
    float cmd0 = constrain(ANGLE_TO_VELOCITY_KP * ( exTilt - ezTilt), -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
    Serial.printf("IMU: X-tilt=%.2f Z-tilt=%.2f accel=%.2f g -> motor1=%.1f motor0=%.1f %s\n",
                  xTilt, zTilt, currentAccelG,
                  cmd1, cmd0,
                  launchDetected ? "[LAUNCHED]" : "[WAITING]");
  }
}

void updateMotorCommand(float ex, float ez) {
  float cmd1 = constrain(ANGLE_TO_VELOCITY_KP * (-ex - ez), -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
  float cmd0 = constrain(ANGLE_TO_VELOCITY_KP * ( ex - ez), -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
  ser.set(pitch_control.ctrl_velocity_, cmd1);  // Motor 1
  ser.set(roll_control.ctrl_velocity_,  cmd0);  // Motor 0
}

