#ifndef CUT_MOTOR_HOME_ERROR_HANDLER_H
#define CUT_MOTOR_HOME_ERROR_HANDLER_H

#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include "Functions.h"

//* ************************************************************************
//* ************* CUT MOTOR HOME ERROR HANDLER HEADER *********************
//* ************************************************************************
// Header file for cut motor home position error detection and recovery system.

// External constants that need to be defined in main.cpp
extern const int CUT_MOTOR_STEPS_PER_INCH;

// Structure to hold the result of cut motor home error detection
struct CutMotorHomeErrorResult {
    bool wasHomeDetected;                    // True if home position was successfully detected
    bool shouldTransitionToError;            // True if system should enter ERROR state
    bool shouldAttemptSlowRecovery;          // True if slow recovery at homing speed should be attempted
    bool shouldContinueWithWarning;          // True if operation should continue with warning only
    String errorMessage;                     // Descriptive message about the error or success
};

// Constants for recovery system
extern const unsigned long CUT_MOTOR_HOME_RECOVERY_TIMEOUT_MS;
extern const float CUT_MOTOR_HOME_RECOVERY_SPEED;

// Result creation helper functions
CutMotorHomeErrorResult createSuccessResult();
CutMotorHomeErrorResult createErrorTransitionResult(const String& message);
CutMotorHomeErrorResult createWarningOnlyResult(const String& message);

// Real-time home sensor monitoring with controlled deceleration (called from main loop)
// Uses quarter-inch controlled deceleration and 30ms verification delay for reliable detection
void performCutMotorRealTimeHomeSensorCheck(
    FastAccelStepper* cutMotor, 
    Bounce& cutHomingSwitch, 
    bool& cutMotorInYesWoodReturn
);

// Home error detection with slow recovery capability
CutMotorHomeErrorResult handleCutMotorHomeError(
    Bounce& cutHomingSwitch, 
    FastAccelStepper* cutMotor, 
    const String& contextDescription,
    bool allowSlowRecovery
);

// Execute error state transition with all necessary safety actions
void executeCutMotorErrorStateTransition(
    FastAccelStepper* cutMotor,
    FastAccelStepper* positionMotor,
    SystemState& currentState,
    int& cuttingStep,
    int& cuttingSubStep7,
    int& fixPositionStep,
    int& fixPositionSubStep2,
    unsigned long& errorStartTime,
    bool engageWoodSecureClamp = true
);

// Utility function to log the results of error detection
void logCutMotorHomeErrorResult(const CutMotorHomeErrorResult& result);

#endif // CUT_MOTOR_HOME_ERROR_HANDLER_H 