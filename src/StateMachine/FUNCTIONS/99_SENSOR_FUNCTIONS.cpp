#include "StateMachine/StateMachine.h"

//* ************************************************************************
//* ************************ SENSOR FUNCTIONS ***************************
//* ************************************************************************
//! Sensor and switch reading functions
//! Functions for reading all sensors and limit switches

//* ************************************************************************
//* ************************ SENSOR READING FUNCTIONS ********************
//* ************************************************************************

bool readWoodSensor() {
    // Wood sensor is active LOW with input pullup
    return digitalRead((int)WOOD_SENSOR) == LOW;
}

bool readWoodSuctionSensor() {
    // Wood suction sensor is active LOW with input pullup
    return digitalRead((int)WAS_WOOD_SUCTIONED_SENSOR) == LOW;
}

//* ************************************************************************
//* ************************ SWITCH READING FUNCTIONS ********************
//* ************************************************************************

bool readCutMotorHomingSwitch() {
    // Cut motor homing switch is active HIGH with input pulldown
    return digitalRead((int)CUT_MOTOR_HOMING_SWITCH) == HIGH;
}

bool readPositionMotorHomingSwitch() {
    // Position motor homing switch is active HIGH with input pulldown
    return digitalRead((int)POSITION_MOTOR_HOMING_SWITCH) == HIGH;
} 