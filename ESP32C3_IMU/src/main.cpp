#include <Arduino.h>
#include <Wire.h>
#include <iq_module_communication.hpp>
#include <JY901.h>

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define BUTTON_PIN 0

// Motor control mapping
const float ANGLE_TO_VELOCITY_KP = 2.0f;    // motor units per degree
const float MAX_MOTOR_VELOCITY = 80.0f;     // motor units
const float ANGLE_DEADBAND_DEG = 1.5f;      // deadband around level
const unsigned long TELEMETRY_INTERVAL_MS = 200;

const float ACCEL_SCALE_G = 16.0f / 32768.0f;   // WT901 raw accel to g
const float LAUNCH_ACCEL_THRESHOLD_G = 2.0f;    // absolute accel threshold for launch burn
const float AMBIENT_ACCEL_THRESHOLD_G = 1.15f;  // return to near ambient after burn
const uint8_t LAUNCH_CONFIRM_SAMPLES = 3;
const uint8_t AMBIENT_CONFIRM_SAMPLES = 3;
const unsigned long BUTTON_DEBOUNCE_MS = 50;

IqSerial ser(Serial0);
MultiTurnAngleControlClient multi_control(0);

float currentRoll = 0.0f;
float currentPitch = 0.0f;
float currentYaw = 0.0f;
float currentAccelG = 0.0f;
bool accelReturnedToAmbient = false;
bool launchSpikeSeen = false;
uint8_t ambientConfirmCount = 0;
unsigned long lastTelemetryTime = 0;

bool launchDetected = false;
uint8_t launchConfirmCount = 0;
unsigned long launchTimeMs = 0;
float launchRoll = 0.0f;
float launchPitch = 0.0f;
float launchYaw = 0.0f;
unsigned long candidateLaunchTimeMs = 0;
float candidateLaunchRoll = 0.0f;
float candidateLaunchPitch = 0.0f;
float candidateLaunchYaw = 0.0f;

bool lastButtonState = HIGH;
unsigned long lastButtonTime = 0;

float computeMotorVelocity(float angleDeg);
void updateMotorCommand(float velocityCommand);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32C3 WT901B IMU motor control starting...");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  JY901.StartIIC();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastButtonState = digitalRead(BUTTON_PIN);

  ser.begin();
  Serial.println("IQ serial initialized");
}

void loop() {
  JY901.GetAngle();
  currentRoll  = (float)JY901.stcAngle.Angle[0] / 32768.0f * 180.0f;
  currentPitch = (float)JY901.stcAngle.Angle[1] / 32768.0f * 180.0f;
  currentYaw   = (float)JY901.stcAngle.Angle[2] / 32768.0f * 180.0f;

  JY901.GetAcc();
  float ax = (float)JY901.stcAcc.a[0] * ACCEL_SCALE_G;
  float ay = (float)JY901.stcAcc.a[1] * ACCEL_SCALE_G;
  float az = (float)JY901.stcAcc.a[2] * ACCEL_SCALE_G;
  currentAccelG = sqrtf(ax * ax + ay * ay + az * az);

  bool buttonState = digitalRead(BUTTON_PIN);
  unsigned long buttonNow = millis();
  if (buttonState != lastButtonState && (buttonNow - lastButtonTime) > BUTTON_DEBOUNCE_MS) {
    lastButtonTime = buttonNow;
    lastButtonState = buttonState;
    if (buttonState == LOW) {
      launchDetected = false;
      launchConfirmCount = 0;
      accelReturnedToAmbient = false;
      launchSpikeSeen = false;
      ambientConfirmCount = 0;
      Serial.println("Launch reset: waiting for next launch event.");
    }
  }

  if (!launchDetected) {
    // State machine:
    // 1) Require ambient first (sitting on pad / not accelerating)
    // 2) Detect a sustained high-accel spike (burn)
    // 3) Declare "launch" at the instant accel returns to ambient (start of ballistic)

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
      // Burn already detected: wait for return-to-ambient (ballistic start).
      if (currentAccelG <= AMBIENT_ACCEL_THRESHOLD_G) {
        if (ambientConfirmCount == 0) {
          candidateLaunchTimeMs = millis();
          candidateLaunchRoll = currentRoll;
          candidateLaunchPitch = currentPitch;
          candidateLaunchYaw = currentYaw;
        }

        ambientConfirmCount++;
        if (ambientConfirmCount >= AMBIENT_CONFIRM_SAMPLES) {
          launchDetected = true;
          launchTimeMs = candidateLaunchTimeMs;
          launchRoll = candidateLaunchRoll;
          launchPitch = candidateLaunchPitch;
          launchYaw = candidateLaunchYaw;
          Serial.printf("Launch (ballistic) detected at %lu ms: roll=%.2f pitch=%.2f yaw=%.2f accel=%.2f g\n",
                        launchTimeMs, launchRoll, launchPitch, launchYaw, currentAccelG);
        }
      } else {
        ambientConfirmCount = 0;
      }
    }
  }

  float velocityCommand = 0.0f;
  if (launchDetected) {
    velocityCommand = computeMotorVelocity(currentPitch);
  }
  updateMotorCommand(velocityCommand);

  unsigned long now = millis();
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = now;
    Serial.printf("IMU angles: roll=%.2f pitch=%.2f yaw=%.2f accel=%.2f g -> target vel=%.1f %s\n",
                  currentRoll, currentPitch, currentYaw, currentAccelG,
                  velocityCommand, launchDetected ? "[LAUNCHED]" : "[WAITING]" );
  }
}

float computeMotorVelocity(float angleDeg) {
  if (fabs(angleDeg) < ANGLE_DEADBAND_DEG) {
    return 0.0f;
  }
  float velocity = ANGLE_TO_VELOCITY_KP * angleDeg;
  velocity = constrain(velocity, -MAX_MOTOR_VELOCITY, MAX_MOTOR_VELOCITY);
  return velocity;
}

void updateMotorCommand(float velocityCommand) {
  ser.set(multi_control.ctrl_velocity_, velocityCommand);
}
