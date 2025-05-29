#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include "Config/Config.h"
#include "Config/Pins_Definitions.h"
#include <FastAccelStepper.h>

//* ************************************************************************
//* ************************ STATE MACHINE ***************************
//* ************************************************************************
//! State machine header file
//! Defines all states and function prototypes for the system

// Motor object declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;

// Clamp Types Enum
enum ClampType {
    POSITION_CLAMP_TYPE,
    WOOD_SECURE_CLAMP_TYPE,
    CATCHER_CLAMP_TYPE
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
void executepushWoodForwardOne();
void executeRELOAD();

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
void startHomingSequence();
void updateHomingSequence();
bool isHomingSequenceComplete();
void homeCutMotor();
bool isCutMotorHoming();
void homePositionMotor();
bool isPositionMotorHoming();
void movePositionMotorToTravelAfterHoming();
bool isPositionMotorMovingToTravel();
bool isCutMotorHomingComplete();
bool isPositionMotorHomingComplete();
bool isPositionMotorAtTravelPosition();
void resetHomingFlags();

// Motor Control Functions
void stopCutMotor();
void stopPositionMotor();
void configureCutMotorForCutting();
void configureCutMotorForReturn();
void configurePositionMotorForNormalOperation();
void configurePositionMotorForReturn();
void moveCutMotorToCut();
void moveCutMotorToHome();
void movePositionMotorToTravel();
void movePositionMotorToHome();
void movePositionMotorToPosition(float targetPositionInches);

// Stepper Motor Functions
void stepperStepCutMotor(bool direction);
void stepperStepPositionMotor(bool direction);
void moveCutMotorSteps(float steps, bool direction, float delayBetweenSteps);
void movePositionMotorSteps(float steps, bool direction, float delayBetweenSteps);

// Clamp Control Functions
void extendClamp(ClampType clamp);
void retractClamp(ClampType clamp);
void retractAllCylinders();
void extendAllCylinders();

// Sensor Reading Functions
bool readWoodSensor();
bool readWoodSuctionSensor();
bool readSensor(float sensorNumber);
bool readCutMotorHomingSwitch();
bool readPositionMotorHomingSwitch();
bool readLimitSwitch(float switchNumber);
bool readHomeSwitch(float switchNumber);

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

// Error Functions
void forceTriggerWoodSuctionError();
bool isCutMotorHomeErrorActive();
void triggerCutMotorHomeError();

#endif // STATEMACHINE_H 