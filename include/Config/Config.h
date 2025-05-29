#ifndef CONFIG_H
#define CONFIG_H

//* ************************************************************************
//* ************************ SYSTEM CONFIGURATION ************************
//* ************************************************************************
// Configuration constants for the Automated Table Saw - Stage 1
// Motor settings, servo positions, timing, and operational parameters

//* ************************************************************************
//* ************************ SERVO CONFIGURATION **************************
//* ************************************************************************
// Catcher servo position settings
extern const int CATCHER_SERVO_HOME_POSITION;     // Home position (degrees)
extern const int CATCHER_SERVO_ACTIVE_POSITION;   // Position when activated (degrees)

// Catcher servo early activation offset
extern const float CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES; // Early activation offset for servo rotation

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION **************************
//* ************************************************************************
// Motor pulse width settings
extern const int CUT_MOTOR_MIN_PULSE_WIDTH;
extern const int POSITION_MOTOR_MIN_PULSE_WIDTH;

// Motor step calculations and travel distances
extern const int CUT_MOTOR_STEPS_PER_INCH;  // 4x increase from 38
extern const int POSITION_MOTOR_STEPS_PER_INCH; // Steps per inch for position motor
extern const float CUT_TRAVEL_DISTANCE; // inches
extern const float POSITION_TRAVEL_DISTANCE; // inches
extern const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES; // Inches for incremental reverse
extern const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES; // Max inches for incremental reverse before error

// Motor homing direction constants
extern const int CUT_HOMING_DIRECTION;
extern const int POSITION_HOMING_DIRECTION;

// Motor homing distance constants
extern const long CUT_MOTOR_HOMING_DISTANCE;      // Maximum distance to travel during homing
extern const long POSITION_MOTOR_HOMING_DISTANCE; // Maximum distance to travel during homing

// Calculated motor positions
extern const long CUT_MOTOR_CUT_POSITION;  // = CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH
extern const long POSITION_MOTOR_TRAVEL_POSITION;  // = POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH
extern const float STEPS_PER_INCH_POSITION;  // = POSITION_MOTOR_STEPS_PER_INCH

//* ************************************************************************
//* ************************ CUT MOTOR SPEED SETTINGS ********************
//* ************************************************************************
// Normal Cutting Operation (Cutting State)
extern const float CUT_MOTOR_NORMAL_SPEED;      // Speed for the cutting pass (steps/sec)
extern const float CUT_MOTOR_NORMAL_ACCELERATION; // Acceleration for the cutting pass (steps/sec^2)

// Return Stroke (Returning State / End of Cutting State)
extern const float CUT_MOTOR_RETURN_SPEED;     // Speed for returning after a cut (steps/sec)

// Homing Operation (Homing State)
extern const float CUT_MOTOR_HOMING_SPEED;      // Speed for homing the cut motor (steps/sec)

//* ************************************************************************
//* ************************ POSITION MOTOR SPEED SETTINGS ***************
//* ************************************************************************
// Normal Positioning Operation (Positioning State / Parts of Cutting State)
extern const float POSITION_MOTOR_NORMAL_SPEED;    // Speed for normal positioning moves (steps/sec)
extern const float POSITION_MOTOR_NORMAL_ACCELERATION; // Acceleration for normal positioning (steps/sec^2)

// Return to Home/Start (Returning State / End of Cutting State / Homing after initial move)
extern const float POSITION_MOTOR_RETURN_SPEED;    // Speed for returning to home or start position (steps/sec)
extern const float POSITION_MOTOR_RETURN_ACCELERATION; // Acceleration for return moves (steps/sec^2)

// Homing Operation (Homing State)
extern const float POSITION_MOTOR_HOMING_SPEED;     // Speed for homing the position motor (steps/sec)

//* ************************************************************************
//* ************************ TIMING CONFIGURATION *************************
//* ************************************************************************
// Servo timing configuration
extern const unsigned long CATCHER_SERVO_ACTIVE_HOLD_DURATION_MS;

// Catcher clamp timing
extern const unsigned long CATCHER_CLAMP_ENGAGE_DURATION_MS; // 1.5 seconds

// Cylinder timing constants
extern const unsigned long CYLINDER_EXTEND_TIME;
extern const unsigned long CYLINDER_RETRACT_TIME;

// Wood caught error check timing
extern const unsigned long WOOD_CAUGHT_CHECK_DELAY_MS; // 1 second delay to check if wood was caught

// Cut motor homing timeout
extern const unsigned long CUT_HOME_TIMEOUT; // 5 seconds timeout

// Position motor homing timeout
extern const unsigned long POSITION_HOME_TIMEOUT; // Timeout for position motor homing

// Signal timing
extern const unsigned long TA_SIGNAL_DURATION; // Duration for Transfer Arm signal (ms)

//* ************************************************************************
//* ************************ OPERATIONAL CONSTANTS ***********************
//* ************************************************************************
// Catcher clamp early activation offset
extern const float CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;

#endif // CONFIG_H 