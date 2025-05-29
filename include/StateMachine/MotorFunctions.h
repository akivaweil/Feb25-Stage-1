#ifndef MOTOR_FUNCTIONS_H
#define MOTOR_FUNCTIONS_H

#include <Arduino.h>

//* ************************************************************************
//* ************************ MOTOR FUNCTIONS HEADER ***************************
//* ************************************************************************
//! Header file for motor and pneumatic control functions
//! Function prototypes for stepper motors and pneumatic cylinders
//! THIS FILE CONTAINS ONLY MOTOR-RELATED FUNCTION PROTOTYPES

// Forward declaration for ClampType enum
enum ClampType {
    POSITION_CLAMP_TYPE,
    WOOD_SECURE_CLAMP_TYPE,
    CATCHER_CLAMP_TYPE
};

// Motor control function prototypes
// Basic stepper motor operations

// Individual Motor Step Functions
void stepperStepCutMotor(bool direction);
void stepperStepPositionMotor(bool direction);

// Multi-Step Motor Movement Functions
void moveCutMotorSteps(float steps, bool direction, float delayBetweenSteps);
void movePositionMotorSteps(float steps, bool direction, float delayBetweenSteps);

// Pneumatic Clamp Functions - Unified Parameter-Based System
void extendClamp(ClampType clamp);
void retractClamp(ClampType clamp);

// Collective Operations
void retractAllCylinders();
void extendAllCylinders();

#endif // MOTOR_FUNCTIONS_H 