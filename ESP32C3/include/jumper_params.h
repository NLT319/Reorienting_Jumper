#ifndef JUMPER_PARAMS_H
#define JUMPER_PARAMS_H

/**
 * @file jumper_params.h
 * @brief System parameters for the ballistic jumper
 * 
 * These parameters define the physical properties and control parameters
 * for the jumping robot. Easily modifiable for rapid prototyping and iteration.
 */

// ============================================================================
// PHYSICAL PARAMETERS
// ============================================================================

// Body properties
const float MB = 0.16f;              // Body mass (kg)
const float LB = 0.00235f;           // Body length (m)
const float JB = 0.0f;               // Body moment of inertia (kg*m^2)
                                      // Pre-calculate: JB = (1/12)*MB*LB^2

// Wheel properties  
const float MW = 0.02f;              // Wheel mass (kg)
const float RW = 0.16f;              // Wheel radius (m)
const float JW = 0.0f;               // Wheel moment of inertia (kg*m^2)
                                      // Pre-calculate: JW = (1/2)*MW*RW^2

// Geometric relationship
const float LBW = 0.0f;              // Distance from body COM to wheel axis (m)
                                      // Currently 0 (wheel at body COM)

// ============================================================================
// CONTROL PARAMETERS
// ============================================================================

// Jump trajectory
const float JUMP_ANGLE = 45.0f * M_PI / 180.0f;  // Jump angle in radians
const float JUMP_VELOCITY = 20.0f;               // Jump velocity (m/s)
const float FINAL_THETA = M_PI - JUMP_ANGLE;     // Target landing angle (180 deg - jump angle)

// Ballistic trajectory
const float G = 9.81f;               // Gravitational acceleration (m/s^2)
const float Y0 = 0.0f;               // Initial height of COM relative to goal (m)

// Braking
const float T_BRAKE = 0.1f;          // Brake application duration (s)
const float BRAKE_VELOCITY_THRESHOLD = 1e-4f;  // Wheel velocity threshold to detect stop (rad/s)

// Spin-up
const float SPINUP_TIME = 3.0f;      // Time allowed for motor spin-up (s)
const float TARGET_VELOCITY = 60.0f; // Target motor velocity during spin-up (motor units)
                                      // This corresponds to desired flywheel angular velocity

// ============================================================================
// DERIVED PARAMETERS (calculated at runtime)
// ============================================================================
// These are computed in setup() based on the above values

struct JumperParams {
    // Inertias
    float Jb;
    float Jw;
    float Jt;           // Total body inertia: Jb + mw*Lbw^2
    
    // Ballistic properties
    float vx0;          // Initial horizontal velocity (m/s)
    float vy0;          // Initial vertical velocity (m/s)
    float y0_adjusted;  // Initial height adjusted for COM offset (m)
    
    // Control derived values
    float H0;           // Initial angular momentum (Jw * bdphi0)
    float bdphi0;       // Initial flywheel angular velocity (rad/s) - calculated from params
};

/**
 * @brief Initialize all derived parameters
 * @return Fully populated JumperParams struct
 */
JumperParams initJumperParams() {
    JumperParams p;
    
    // Calculate inertias (if not pre-calculated)
    p.Jb = (1.0f / 12.0f) * MB * LB * LB;
    p.Jw = 0.5f * MW * RW * RW;
    p.Jt = p.Jb + MW * LBW * LBW;
    
    // Calculate initial velocities from jump angle and velocity
    p.vx0 = JUMP_VELOCITY * cosf(JUMP_ANGLE);
    p.vy0 = JUMP_VELOCITY * sinf(JUMP_ANGLE);
    
    // COM offset due to initial body angle
    p.y0_adjusted = Y0 + (LB / 2.0f) * sinf(JUMP_ANGLE);
    
    // Calculate required initial flywheel angular velocity from momentum conservation
    // H0 = Jw * bdphi0 is conserved (rigid connection during braking)
    // For proper rotation: dtheta_post = H0 / Jt
    // This must match the final angle requirement (180 deg flip)
    // For now, calculate based on energy and momentum balance
    // bdphi0 = (some fraction of) final angular velocity requirement
    // Simplified: use a calculated value based on system inertias
    p.bdphi0 = 2.0f;  // This would be refined based on MATLAB validation
    p.H0 = p.Jw * p.bdphi0;
    
    return p;
}

#endif // JUMPER_PARAMS_H
