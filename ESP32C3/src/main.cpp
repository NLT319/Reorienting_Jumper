#include <Arduino.h>
#include <iq_module_communication.hpp>
#include "jumper_params.h"

// ============================================================================
// SYSTEM STATE MACHINE
// ============================================================================
enum SystemState {
  IDLE,        // Waiting for button press
  COUNTDOWN,   // Spinning up motor for 3 seconds
  BRAKING,     // Brake applied, waiting for landing
  COMPLETE     // Jump completed (landed)
};

// ============================================================================
// GLOBAL STATE AND TIMING VARIABLES
// ============================================================================
SystemState currentState = IDLE;
unsigned long launchTime = 0;         // Actual launch time (ms)
float calculatedBrakeTime = 0.0f;       // Time from launch to apply brake (ms)
float calculatedLandingTime = 0.0f;     // Time from launch to landing (ms)
bool brakeActive = false;               // Track if brake has been applied
bool brakeHasBeenApplied = false;        // Whether braking has been engaged in this cycle
unsigned long brakeApplyTime = 0;        // When brake was engaged
float observedFlywheelVelocity = 0.0f;   // Latest flywheel velocity reading
bool nextJumpTimingsPrepared = false;    // Whether next jump timings have been prepared in IDLE

JumperParams jumper;                    // System parameters
uint32_t lastStateChangeTime = 0;       // For debug timing
bool buttonPreviouslyPressed = false;   // Track button edge transitions
uint32_t lastButtonEventTime = 0;       // For button debounce

const unsigned long BUTTON_DEBOUNCE_MS = 50;   // Debounce and cooldown timing
const unsigned long RESET_COOLDOWN_MS = 300;

// ============================================================================
// IQ MODULE CLIENTS
// ============================================================================
IqSerial ser(Serial0);
PowerMonitorClient power(0);
PropellerMotorControlClient prop_control(0);
BrushlessDriveClient brushless_drive(0);
MultiTurnAngleControlClient multi_control(0);

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initializeSystem();
void updateStateMachine();
void handleIdle();
void handleCountdown();
void handleBraking();
void updateMotorCommand();
float calculateLandingTime();
float calculateBrakeTime();
void prepareNextJumpTimings();
void printCalculatedTimings(const char* contextLabel);
void logStateChange(const char* newState);
void logEvent(const char* event);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Initialize serial communication
  ser.begin();
  Serial.begin(115200);
  
  // Initialize button input
  pinMode(2, INPUT_PULLUP);
  
  // Initialize system parameters
  jumper = initJumperParams();
  
  // Initial logging
  delay(500);  // Allow serial to stabilize
  Serial.println("\n========================================");
  Serial.println("BALLISTIC JUMPER CONTROL SYSTEM");
  Serial.println("========================================");
  Serial.print("Jump Angle: ");
  Serial.print(JUMP_ANGLE * 180.0f / M_PI);
  Serial.println(" deg");
  Serial.print("Jump Velocity: ");
  Serial.print(JUMP_VELOCITY);
  Serial.println(" m/s");
  Serial.print("Body Inertia (Jb): ");
  Serial.print(jumper.Jb, 6);
  Serial.println(" kg*m^2");
  Serial.print("Wheel Inertia (Jw): ");
  Serial.print(jumper.Jw, 6);
  Serial.println(" kg*m^2");
  Serial.print("Initial Flywheel Velocity (bdphi0): ");
  Serial.print(jumper.bdphi0);
  Serial.println(" rad/s");
  Serial.println("========================================");
  Serial.println("System Ready. Press button to launch.");
  Serial.println("========================================\n");
  
  logStateChange("IDLE");
}

// ============================================================================
// MAIN LOOP (NON-BLOCKING)
// ============================================================================
void loop() {
  // Update state machine based on elapsed time and button state
  updateStateMachine();
  
  // Handle current state logic
  switch (currentState) {
    case IDLE:
      handleIdle();
      break;
    case COUNTDOWN:
      handleCountdown();
      break;
    case BRAKING:
      handleBraking();
      break;
    case COMPLETE:
      // Do nothing, wait for system reset
      break;
  }
  
  // Send motor commands based on current state
  updateMotorCommand();
  
  // Small delay to avoid overwhelming serial/motor comms
  delay(50);
}

// ============================================================================
// STATE MACHINE LOGIC
// ============================================================================

void updateStateMachine() {
  unsigned long currentTime = millis();
  int buttonState = digitalRead(2);
  bool buttonPressed = (buttonState == LOW);
  bool buttonJustPressed = buttonPressed && !buttonPreviouslyPressed;
  buttonPreviouslyPressed = buttonPressed;

  if (buttonJustPressed && currentTime - lastButtonEventTime < BUTTON_DEBOUNCE_MS) {
    buttonJustPressed = false;
  }
  if (buttonJustPressed) {
    lastButtonEventTime = currentTime;
  }

  long timeFromLaunchMs = (long)currentTime - (long)launchTime;

  // Allow reset from COMPLETE back to IDLE on button press
  if (currentState == COMPLETE && buttonJustPressed) {
    currentState = IDLE;
    launchTime = 0;
    nextJumpTimingsPrepared = false;
    logStateChange("IDLE");
    logEvent("Button pressed - reset to IDLE");
    calculatedBrakeTime = 0.0f;
    calculatedLandingTime = 0.0f;
    brakeActive = false;
    brakeHasBeenApplied = false;
    return;
  }

  bool stateTransitionAllowed = (lastStateChangeTime == 0) || (currentTime - lastStateChangeTime >= RESET_COOLDOWN_MS);

  switch (currentState) {
    case IDLE:
      // Button pressed (LOW when using INPUT_PULLUP)
      if (buttonJustPressed && stateTransitionAllowed) {
        launchTime = currentTime + (unsigned long)(SPINUP_TIME * 1000.0f);
        currentState = COUNTDOWN;
        logStateChange("COUNTDOWN");
        logEvent("Button pressed - starting 3s spin-up");
      }
      break;
      
    case COUNTDOWN:
      // When countdown reaches launch, transition to braking
      if (timeFromLaunchMs >= 0) {
        currentState = BRAKING;
        
        // Calculate landing and brake times
        calculatedLandingTime = calculateLandingTime() * 1000.0f;  // Convert to ms
        calculatedBrakeTime = calculateBrakeTime() * 1000.0f;      // Convert to ms
        brakeActive = false;
        brakeHasBeenApplied = false;
        
        logStateChange("BRAKING");
        logEvent("Launch event - countdown complete");
        Serial.println("LAUNCH: countdown complete, entering BRAKING state");
        Serial.print("Braking time (ms): ");
        Serial.println(calculatedBrakeTime, 1);
        Serial.print("Landing time (ms): ");
        Serial.println(calculatedLandingTime, 1);
        if (calculatedLandingTime <= 0.0f || calculatedBrakeTime <= 0.0f) {
          Serial.println("WARNING: invalid timing values computed");
        }
      }
      break;
      
    case BRAKING:
      // Apply brake at the calculated time after launch to allow body rotation
      if (!brakeActive && !brakeHasBeenApplied && timeFromLaunchMs >= (long)calculatedBrakeTime) {
        brakeActive = true;
        brakeHasBeenApplied = true;
        logEvent("Brake applied at calculated time");
      }

      // Release brake once the flywheel has stopped
      if (brakeActive && observedFlywheelVelocity <= BRAKE_VELOCITY_THRESHOLD) {
        brakeActive = false;
        logEvent("Brake released after flywheel stopped");
      }
      
      // Check if we've passed landing time (simulation end)
      if (timeFromLaunchMs >= (long)(calculatedLandingTime + 500.0f)) {  // Add 500ms buffer
        currentState = COMPLETE;
        logStateChange("COMPLETE");
        logEvent("Landing event detected");
      }
      break;
      
    case COMPLETE:
      // Stay in this state until restart/reset via button
      break;
  }
}

void prepareNextJumpTimings() {
  calculatedLandingTime = calculateLandingTime() * 1000.0f;
  calculatedBrakeTime = calculateBrakeTime() * 1000.0f;
  nextJumpTimingsPrepared = true;
}

void handleIdle() {
  if (!nextJumpTimingsPrepared) {
    prepareNextJumpTimings();
    printCalculatedTimings("Next jump timings");
  }
  // Idle state: motor off, monitoring button
  // No active control
}

void handleCountdown() {
  // Countdown state: spin motor to target velocity
  // Motor will ramp based on velocity setpoint
}

void handleBraking() {
  // Braking state: apply brake torque when triggered
  // Monitor sensor voltages for diagnostics
  float volts_now = 0.0f;
  float velocity_now = 0.0f;
  
  // Read system voltages for diagnostics (no serial output)
  if (ser.get(power.volts_, volts_now)) {
    // voltage data available if needed later
  }
  
  // Read flywheel velocity
  if (ser.get(brushless_drive.obs_velocity_, velocity_now)) {
    observedFlywheelVelocity = velocity_now;
  }
}

// ============================================================================
// MOTOR CONTROL COMMAND
// ============================================================================

void updateMotorCommand() {
  switch (currentState) {
    case IDLE:
      // Motor not engaged: coast
      ser.set(multi_control.ctrl_coast_);
      break;
      
    case COUNTDOWN:
      // Ramp to target velocity
      ser.set(multi_control.ctrl_velocity_, TARGET_VELOCITY);
      break;
      
    case BRAKING:
      if (brakeActive) {
        // Short the motor coils (maximum braking)
        ser.set(multi_control.ctrl_brake_);
      } else if (brakeHasBeenApplied) {
        // Brake has been applied and released; coast
        ser.set(multi_control.ctrl_coast_);
      } else {
        // Hold current velocity until brake time
        ser.set(multi_control.ctrl_velocity_, TARGET_VELOCITY);
      }
      break;
      
    case COMPLETE:
      // Motor off / allow freewheel after landing
      ser.set(multi_control.ctrl_coast_);
      break;
  }
}

// ============================================================================
// PHYSICS CALCULATIONS
// ============================================================================

/**
 * @brief Calculate landing time using ballistic equations
 * 
 * Solves: y_final = y0 + vy0*t - 0.5*g*t^2
 * where y_final accounts for the rotated body geometry at landing
 * 
 * @return Landing time in seconds
 */
float calculateLandingTime() {
  // At landing, the body has rotated by (FINAL_THETA - JUMP_ANGLE)
  // The foot touches ground when: COM_y - (Lb/2)*sin(final_theta) = 0
  
  float y_com_required = (LB / 2.0f) * sinf(FINAL_THETA);
  
  // Quadratic formula for ballistic trajectory
  // y_com_landing = y0 + vy0*t - 0.5*g*t^2
  // 0 = (-0.5*g)*t^2 + vy0*t + (y0_adjusted - y_com_required)
  
  float a = -0.5f * G;
  float b = jumper.vy0;
  float c = jumper.y0_adjusted - y_com_required;
  
  float discriminant = b * b - 4.0f * a * c;
  if (discriminant < 0) {
    Serial.println("ERROR: No real solution for landing time!");
    return 0.0f;
  }
  
  float t1 = (-b + sqrtf(discriminant)) / (2.0f * a);
  float t2 = (-b - sqrtf(discriminant)) / (2.0f * a);
  
  // Return the positive root
  float t_land = max(t1, t2);
  if (t_land < 0) t_land = min(t1, t2);
  
  return t_land;
}

/**
 * @brief Calculate brake trigger time
 * 
 * Uses momentum conservation to determine when braking should start
 * such that the jumper reaches final angle exactly at landing.
 * 
 * @return Time (in seconds) when brake should be applied
 */
float calculateBrakeTime() {
  // Initial angular momentum (conserved during braking)
  float H0 = jumper.Jw * jumper.bdphi0;
  
  // Post-brake body angular velocity (from momentum conservation)
  // After brake: dtheta_post = H0 / Jt
  float dtheta_post = H0 / jumper.Jt;
  
  // Landing time
  float t_land = calculateLandingTime();
  
  // Current body angle at launch
  float theta_0 = JUMP_ANGLE;
  
  // Required rotation to reach final angle
  float delta_theta = FINAL_THETA - theta_0;
  
  // Brake must be applied such that:
  // delta_theta = dtheta_post * (t_land - t_brake)
  // Therefore: t_brake = t_land - (delta_theta / dtheta_post)
  
  if (fabsf(dtheta_post) < 1e-6) {
    Serial.println("ERROR: dtheta_post too small!");
    return t_land * 0.5f;  // Fallback to mid-point
  }
  
  float t_brake = t_land - (delta_theta / dtheta_post);
  
  // Safety check: ensure brake time is positive and before landing
  if (t_brake < 0) t_brake = 0.1f;
  if (t_brake > t_land) t_brake = t_land * 0.9f;
  
  return t_brake;
}

// ============================================================================
// DEBUGGING AND LOGGING
// ============================================================================

void logStateChange(const char* newState) {
  unsigned long now = millis();
  lastStateChangeTime = now;
  Serial.print("[");
  Serial.print(now);
  Serial.print(" ms] STATE: ");
  Serial.println(newState);
}

void printCalculatedTimings(const char* contextLabel) {
  if (calculatedBrakeTime > 0.0f || calculatedLandingTime > 0.0f) {
    Serial.print("--- ");
    Serial.print(contextLabel);
    Serial.println(" ---");
    Serial.print("Spin-up time (ms): ");
    Serial.println(SPINUP_TIME * 1000.0f, 1);
    Serial.print("Brake trigger time (ms): ");
    Serial.println(calculatedBrakeTime, 1);
    Serial.print("Landing time (ms): ");
    Serial.println(calculatedLandingTime, 1);
    Serial.println("-------------------------");
  }
}

void logEvent(const char* event) {
  unsigned long now = millis();
  Serial.print("[");
  Serial.print(now);
  Serial.print(" ms] EVENT: ");
  Serial.println(event);
}


/*
// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}*/