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
unsigned long launchStartTime = 0;      // Time when button was pressed (ms)
float calculatedBrakeTime = 0.0f;       // Time from launch to apply brake (ms)
float calculatedLandingTime = 0.0f;     // Time from launch to landing (ms)
bool brakeActive = false;               // Track if brake has been applied

JumperParams jumper;                    // System parameters
uint32_t lastStateChangeTime = 0;       // For debug timing

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
void updateStateachine();
void handleIdle();
void handleCountdown();
void handleBraking();
void updateMotorCommand();
float calculateLandingTime();
float calculateBrakeTime();
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
  int buttonState = digitalRead(2);
  unsigned long elapsedTime = millis() - launchStartTime;
  
  switch (currentState) {
    case IDLE:
      // Button pressed (LOW when using INPUT_PULLUP)
      if (buttonState == LOW) {
        launchStartTime = millis();
        currentState = COUNTDOWN;
        logStateChange("COUNTDOWN");
        logEvent("Button pressed - starting 3s spin-up");
      }
      break;
      
    case COUNTDOWN:
      // After 3 seconds, transition to braking
      if (elapsedTime >= SPINUP_TIME * 1000UL) {
        currentState = BRAKING;
        
        // Calculate landing and brake times
        calculatedLandingTime = calculateLandingTime() * 1000.0f;  // Convert to ms
        calculatedBrakeTime = calculateBrakeTime() * 1000.0f;      // Convert to ms
        brakeActive = false;
        
        logStateChange("BRAKING");
        Serial.print("Calculated landing time: ");
        Serial.print(calculatedLandingTime, 1);
        Serial.println(" ms");
        Serial.print("Calculated brake trigger time: ");
        Serial.print(calculatedBrakeTime, 1);
        Serial.println(" ms");
      }
      break;
      
    case BRAKING:
      // Check if we've reached brake trigger time
      if (!brakeActive && elapsedTime >= calculatedBrakeTime) {
        brakeActive = true;
        logEvent("Brake applied");
      }
      
      // Check if we've passed landing time (simulation end)
      if (elapsedTime >= calculatedLandingTime + 500UL) {  // Add 500ms buffer
        currentState = COMPLETE;
        logStateChange("COMPLETE");
        logEvent("Landing event detected - brake disengaged");
      }
      break;
      
    case COMPLETE:
      // Stay in this state until power cycle or system reset
      break;
  }
}

void handleIdle() {
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
  
  // Read system voltages
  if (ser.get(power.volts_, volts_now)) {
    // Only log periodically to avoid serial spam
    static unsigned long lastVoltLog = 0;
    if (millis() - lastVoltLog > 500) {
      Serial.print("V: ");
      Serial.print(volts_now, 2);
      Serial.println("V");
      lastVoltLog = millis();
    }
  }
  
  // Read flywheel velocity
  if (ser.get(brushless_drive.obs_velocity_, velocity_now)) {
    // Velocity reading available
    // Could be used for feedback in future refinements
  }
}

// ============================================================================
// MOTOR CONTROL COMMAND
// ============================================================================

void updateMotorCommand() {
  switch (currentState) {
    case IDLE:
      // Motor off
      ser.set(multi_control.ctrl_brake_);
      break;
      
    case COUNTDOWN:
      // Ramp to target velocity
      ser.set(multi_control.ctrl_velocity_, TARGET_VELOCITY);
      break;
      
    case BRAKING:
      // Apply brake when triggered
      if (brakeActive) {
        // Short the motor coils (maximum braking)
        ser.set(multi_control.ctrl_brake_);
      } else {
        // Hold current velocity (coast) until brake time
        // This allows smoother transition to braking
        ser.set(multi_control.ctrl_velocity_, TARGET_VELOCITY);
      }
      break;
      
    case COMPLETE:
      // Motor off
      ser.set(multi_control.ctrl_brake_);
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
  Serial.print("[");
  Serial.print(now);
  Serial.print(" ms] STATE: ");
  Serial.println(newState);
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