#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include "Config/Config.h"
#include "Config/Pins_Definitions.h"
#include <AccelStepper.h>
#include <Bounce2.h>

//* ************************************************************************
//* ************************ STATE MACHINE ***************************
//* ************************************************************************
//! State machine header file
//! Defines all states and function prototypes for the system

// Motor object declarations
extern AccelStepper cutMotor;
extern AccelStepper positionMotor;

// Clamp Types Enum (using _TYPE suffix)
enum ClampType {
    POSITION_CLAMP_TYPE,
    WOOD_SECURE_CLAMP_TYPE,
    CATCHER_CLAMP_TYPE
};

// Motor Types Enum
enum MotorType {
    CUT_MOTOR,
    POSITION_MOTOR
};

// Sensor Types Enum
enum SensorType {
    WOOD_SENSOR_TYPE,
    WOOD_SUCTION_SENSOR_TYPE
};

// Switch Types Enum
enum SwitchType {
    CUT_MOTOR_HOMING_SWITCH_TYPE,
    POSITION_MOTOR_HOMING_SWITCH_TYPE
};

// State Definitions
enum SystemState {
    STARTUP,
    IDLE,
    HOMING,
    CUTTING,
    YESWOOD,
    NOWOOD,
    pushWoodForwardOne,
    RELOAD,
    ERROR,
    ERROR_RESET
};

// Global State Variables
extern SystemState currentState;
extern SystemState previousState;
extern bool stateChanged;

// Function Prototypes
void initializeStateMachine();
void updateStateMachine();
void changeState(SystemState newState);
void transitionToState(SystemState newState);

// State Functions
void executeIDLE();
void executeHOMING();
void executeCUTTING();
void executeYESWOOD();
void executeNOWOOD();
void executePUSHWOODFORWARDONE();
void executeRELOAD();

// State Execution Functions (from FUNCTIONS files)
void executeIdleMonitoring();
void executeCuttingSequence();
void executeYeswoodSequence();
void executeNowoodSequence();
void executePushWoodForwardSequence();
void executeReloadSequence();

// Transition Functions
bool checkTransitionConditions();
bool areAllSystemsReady();
bool isHomingComplete();

// Utility Functions
void printStateChange();
void updateStatusLED();
void checkCatcherServoEarlyActivation();
void checkCatcherClampEarlyActivation();
void movePositionMotorToTravelWithEarlyActivation();

// Homing Functions
void executeCompleteHomingSequence();
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout);
void homePositionMotorBlocking(Bounce& homingSwitch);
bool checkAndRecalibrateCutMotorHome(int attempts);

// Motor Control Functions
void moveMotorTo(MotorType motor, float position, float speed);
void stopCutMotor();
void stopPositionMotor();
void movePositionMotorToTravelWithEarlyActivation();
void movePositionMotorToInitialAfterHoming();

// Clamp Control Functions
void extendClamp(ClampType clamp);
void retractClamp(ClampType clamp);
void retractAllCylinders();
void extendAllCylinders();

// Sensor Reading Functions
bool readSensor(SensorType sensor);
bool readLimitSwitch(SwitchType switchType);
bool readHomeSwitch(SwitchType switchType);
bool readWoodSensor();
bool readWoodSuctionSensor();
bool readCutMotorHomingSwitch();
bool readPositionMotorHomingSwitch();

// LED Control Functions
void turnRedLedOn();
void turnRedLedOff();
void turnYellowLedOn();
void turnYellowLedOff();
void turnGreenLedOn();
void turnGreenLedOff();
void turnBlueLedOn();
void turnBlueLedOff();
void allLedsOff();
void handleHomingLedBlink();
void handleErrorLedBlink();

// Error Functions
void forceTriggerWoodSuctionError();
bool isCutMotorHomeErrorActive();
void triggerCutMotorHomeError();

// IDLE State Functions
bool shouldStartCycle();
void handleReloadMode();
void handleStartSwitchSafety();
void handleStartSwitchContinuousMode();

// Signaling Functions
void sendSignalToTA();
void handleTASignalTiming();
void activateCatcherServo();
void handleCatcherServoReturn();

// Timing Functions
void handleCatcherClampDisengage();

// Error Functions
void handleErrorAcknowledgement();

#endif // STATEMACHINE_H 