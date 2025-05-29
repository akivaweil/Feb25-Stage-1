#ifndef MOTOR_FUNCTIONS_H
#define MOTOR_FUNCTIONS_H

#include <Arduino.h>

//* ************************************************************************
//* ************************ MOTOR FUNCTIONS HEADER ***************************
//* ************************************************************************
//! Header file for motor and pneumatic control functions
//! Function prototypes for stepper motors and pneumatic cylinders
//! THIS FILE CONTAINS ONLY MOTOR-RELATED FUNCTION PROTOTYPES
//! Note: Actual enums are defined in StateMachine.h

// Collective Operations
void retractAllCylinders();
void extendAllCylinders();

#endif // MOTOR_FUNCTIONS_H 