#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <ESP32Servo.h> // For Servo object
#include <FastAccelStepper.h> // For FastAccelStepper objects
#include <Bounce2.h> // <<< ADDED for Bounce type

class Bounce; // Forward declaration for linter

// System States Enum Definition
enum SystemState {
  STARTUP,
  HOMING,
  READY,
  CUTTING,
  RETURNING,
  POSITIONING,
  ERROR,
  ERROR_RESET,
  SUCTION_ERROR_HOLD
};

// Include the main cpp file for Pin Definitions. This is a workaround.
// A better solution is to convert #defines to const int variables.
// #include "../src/Stage 1 Feb25.cpp" // This is problematic, causes redefinitions.

// Extern declarations for Pin Definitions from "Stage 1 Feb25.cpp"
// These will be defined in Stage 1 Feb25.cpp as const int
extern const int CUT_MOTOR_PULSE_PIN;
extern const int CUT_MOTOR_DIR_PIN;
extern const int POSITION_MOTOR_PULSE_PIN;
extern const int POSITION_MOTOR_DIR_PIN;
extern const int SERVO_PIN;
extern const int CUT_MOTOR_HOMING_SWITCH;
extern const int POSITION_MOTOR_HOMING_SWITCH;
extern const int RELOAD_SWITCH;
extern const int START_CYCLE_SWITCH;
extern const int WOOD_SENSOR;
extern const int WAS_WOOD_SUCTIONED_SENSOR;
extern const int POSITION_CLAMP;
extern const int WOOD_SECURE_CLAMP;
extern const int CATCHER_CLAMP_PIN;
extern const int STAGE2_SIGNAL_OUT_PIN; // Consider renaming if it's for TA
extern const int RED_LED;
extern const int YELLOW_LED;
extern const int GREEN_LED;
extern const int BLUE_LED;

// Extern declarations for global variables from "Stage 1 Feb25.cpp"
extern SystemState currentState;
extern Servo servoMotor;
extern unsigned long servoAt90StartTime;
extern bool servoIsAt90AndTiming;
extern unsigned long catcherClampEngageTime;
extern bool catcherClampIsEngaged;
extern unsigned long signalStage2StartTime; // Consider renaming
extern bool signalStage2Active; // Consider renaming

// Extern declarations for motor objects
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;

// Extern declarations for motor configuration constants
extern const int CUT_MOTOR_STEPS_PER_INCH;
extern const int POSITION_MOTOR_STEPS_PER_INCH;
extern const float CUT_TRAVEL_DISTANCE;
extern const float POSITION_TRAVEL_DISTANCE;

// Extern declarations for speed and acceleration settings
extern const float CUT_MOTOR_NORMAL_SPEED;
extern const float CUT_MOTOR_NORMAL_ACCELERATION;
extern const float CUT_MOTOR_RETURN_SPEED;
extern const float CUT_MOTOR_HOMING_SPEED;

extern const float POSITION_MOTOR_NORMAL_SPEED;
extern const float POSITION_MOTOR_NORMAL_ACCELERATION;
extern const float POSITION_MOTOR_RETURN_SPEED;
extern const float POSITION_MOTOR_RETURN_ACCELERATION;
extern const float POSITION_MOTOR_HOMING_SPEED;

// Constants
extern const unsigned long SERVO_HOLD_AT_90_DURATION_MS;
extern const unsigned long CATCHER_CLAMP_ENGAGE_DURATION_MS;
extern const unsigned long STAGE2_SIGNAL_DURATION; // Consider renaming

// Switch objects
extern Bounce cutHomingSwitch;
extern Bounce positionHomingSwitch;
extern Bounce reloadSwitch;
extern Bounce startCycleSwitch;

// System flags
extern bool isReloadMode;
extern bool woodPresent; // Read in main loop, used in conditions
extern bool woodSuctionError;
extern bool errorAcknowledged;
extern bool cuttingCycleInProgress;
extern bool continuousModeActive;
extern bool startSwitchSafe;

// Timers for LEDs/Errors
extern unsigned long lastBlinkTime;
extern unsigned long lastErrorBlinkTime;
// errorStartTime is used in main, but not directly by these planned refactored functions yet.
// positionMoveStartTime not directly used by these.

// LED states for blinking
extern bool blinkState;
extern bool errorBlinkState;

// Function Prototypes

//* ************************************************************************
//* *********************** SIGNALING FUNCTIONS ****************************
//* ************************************************************************
// Contains functions related to signaling other stages or components.
void sendSignalToStage2(); // Consider renaming if it's for TA
void handleStage2SignalTiming(); // For point 5, if pursued

//* ************************************************************************
//* ************************* CLAMP FUNCTIONS ******************************
//* ************************************************************************
// Contains functions for controlling various clamps.
void extendPositionClamp();
void retractPositionClamp();
void extendWoodSecureClamp();
void retractWoodSecureClamp();
void extendCatcherClamp();
void retractCatcherClamp();
void handleCatcherClampDisengage(); // Point 4

//* ************************************************************************
//* *************************** LED FUNCTIONS ******************************
//* ************************************************************************
// Contains functions for controlling LEDs.
void turnRedLedOn();
void turnRedLedOff();
void turnYellowLedOn();
void turnYellowLedOff();
void turnGreenLedOn();
void turnGreenLedOff();
void turnBlueLedOn();
void turnBlueLedOff();
void allLedsOff();
// Point 1: LED Blinking Logic
void handleHomingLedBlink();
void handleErrorLedBlink();
void handleSuctionErrorLedBlink(unsigned long& lastBlinkTimeRef, bool& blinkStateRef);

//* ************************************************************************
//* *********************** MOTOR CONTROL FUNCTIONS ************************
//* ************************************************************************
// Contains functions for controlling motor movements and configurations.

void configureCutMotorForCutting();
void configureCutMotorForReturn();
void configurePositionMotorForNormalOperation();
void configurePositionMotorForReturn();
void moveCutMotorToCut();
void moveCutMotorToHome();
void movePositionMotorToTravel();
void movePositionMotorToHome();
void movePositionMotorToPosition(float targetPositionInches);
void stopCutMotor();
void stopPositionMotor();
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout);
void homePositionMotorBlocking(Bounce& homingSwitch);
void movePositionMotorToInitialAfterHoming();
// Point 3: Complex conditional logic
bool checkAndRecalibrateCutMotorHome(int attempts);

//* ************************************************************************
//* ************************* SWITCH LOGIC FUNCTIONS ***********************
//* ************************************************************************
// Point 2: Switch handling
void handleReloadMode();
void handleErrorAcknowledgement(); // Combined error ack from main loop and cutting
void handleStartSwitchSafety(); // Safety check from main loop setup
void handleStartSwitchContinuousMode(); // Continuous mode from main loop

//* ************************************************************************
//* ************************* STATE LOGIC HELPERS **************************
//* ************************************************************************
// Point 3: Complex conditional logic
bool shouldStartCycle();
// Point 4
void handleServoReturn();

#endif // FUNCTIONS_H 