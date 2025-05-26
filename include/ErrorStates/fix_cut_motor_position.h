#ifndef FIX_CUT_MOTOR_POSITION_H
#define FIX_CUT_MOTOR_POSITION_H

#include <Arduino.h>
#include "Functions.h"

// Function declarations for fix cut motor position functionality
void handleFixCutMotorPositionState();
void initFixCutMotorPosition();

// External variable declaration
extern int fixCutMotorPositionStep;

#endif // FIX_CUT_MOTOR_POSITION_H 