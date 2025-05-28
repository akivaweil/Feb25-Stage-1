#include "Config/system_config.h"

//* ************************************************************************
//* ************************ SYSTEM CONFIGURATION ************************
//* ************************************************************************
// Configuration constants for the Automated Table Saw - Stage 1
// Motor settings, servo positions, timing, and operational parameters

//* ************************************************************************
//* ************************ SERVO CONFIGURATION **************************
//* ************************************************************************
// Catcher servo position settings
const int CATCHER_SERVO_HOME_POSITION = 24;     // Home position (degrees)
const int CATCHER_SERVO_ACTIVE_POSITION = 90;   // Position when activated (degrees)

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION **************************
//* ************************************************************************
// Motor step calculations and travel distances
const int CUT_MOTOR_STEPS_PER_INCH = 500;  // 4x increase from 38
const int POSITION_MOTOR_STEPS_PER_INCH = 1000; // Steps per inch for position motor
const float CUT_TRAVEL_DISTANCE = 9.0; // inches
const float POSITION_TRAVEL_DISTANCE = 3.4; // inches
const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES = 0.1; // Inches for incremental reverse
const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES = 0.4; // Max inches for incremental reverse before error

// Motor homing direction constants
const int CUT_HOMING_DIRECTION = -1;
const int POSITION_HOMING_DIRECTION = -1;

//* ************************************************************************
//* ************************ CUT MOTOR SPEED SETTINGS ********************
//* ************************************************************************
// Normal Cutting Operation (Cutting State)
const float CUT_MOTOR_NORMAL_SPEED = 700;      // Speed for the cutting pass (steps/sec)
const float CUT_MOTOR_NORMAL_ACCELERATION = 10000; // Acceleration for the cutting pass (steps/sec^2)

// Return Stroke (Returning State / End of Cutting State)
const float CUT_MOTOR_RETURN_SPEED = 20000;     // Speed for returning after a cut (steps/sec)

// Homing Operation (Homing State)
const float CUT_MOTOR_HOMING_SPEED = 1000;      // Speed for homing the cut motor (steps/sec)

//* ************************************************************************
//* ************************ POSITION MOTOR SPEED SETTINGS ***************
//* ************************************************************************
// Normal Positioning Operation (Positioning State / Parts of Cutting State)
const float POSITION_MOTOR_NORMAL_SPEED = 20000;    // Speed for normal positioning moves (steps/sec)
const float POSITION_MOTOR_NORMAL_ACCELERATION = 30000; // Acceleration for normal positioning (steps/sec^2)

// Return to Home/Start (Returning State / End of Cutting State / Homing after initial move)
const float POSITION_MOTOR_RETURN_SPEED = 30000;    // Speed for returning to home or start position (steps/sec)
const float POSITION_MOTOR_RETURN_ACCELERATION = 30000; // Acceleration for return moves (steps/sec^2)

// Homing Operation (Homing State)
const float POSITION_MOTOR_HOMING_SPEED = 2000;     // Speed for homing the position motor (steps/sec)

//* ************************************************************************
//* ************************ TIMING CONFIGURATION *************************
//* ************************************************************************
// Servo timing configuration
const unsigned long CATCHER_SERVO_ACTIVE_HOLD_DURATION_MS = 2000;

// Catcher clamp timing
const unsigned long CATCHER_CLAMP_ENGAGE_DURATION_MS = 1500; // 1.5 seconds

// Wood caught error check timing
const unsigned long WOOD_CAUGHT_CHECK_DELAY_MS = 1000; // 1 second delay to check if wood was caught

// Cut motor homing timeout
const unsigned long CUT_HOME_TIMEOUT = 5000; // 5 seconds timeout

// Signal timing
const unsigned long TA_SIGNAL_DURATION = 150; // Duration for Transfer Arm signal (ms)

//* ************************************************************************
//* ************************ OPERATIONAL CONSTANTS ***********************
//* ************************************************************************
// Catcher clamp early activation offset
const float CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = 1.2; 