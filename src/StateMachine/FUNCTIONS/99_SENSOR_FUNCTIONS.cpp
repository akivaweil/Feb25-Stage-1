#include "StateMachine/StateMachine.h"

//* ************************************************************************
//* ************************ SENSOR FUNCTIONS ***************************
//* ************************************************************************
//! Sensor and switch reading functions
//! Functions for reading all sensors and limit switches

//* ************************************************************************
//* ************************ UNIFIED SENSOR READING FUNCTIONS ************
//* ************************************************************************

bool readSensor(SensorType sensor) {
    switch(sensor) {
        case WOOD_SENSOR_TYPE:
            // Wood sensor is active LOW with input pullup
            return digitalRead((int)WOOD_SENSOR) == LOW;
        case WOOD_SUCTION_SENSOR_TYPE:
            // Wood suction sensor is active LOW with input pullup
            return digitalRead((int)WAS_WOOD_SUCTIONED_SENSOR) == LOW;
        default:
            Serial.println("ERROR: Unknown sensor type for readSensor operation");
            return false;
    }
}

bool readLimitSwitch(SwitchType switchType) {
    switch(switchType) {
        case CUT_MOTOR_HOMING_SWITCH_TYPE:
            // Cut motor homing switch is active HIGH with input pulldown
            return digitalRead((int)CUT_MOTOR_HOMING_SWITCH) == HIGH;
        case POSITION_MOTOR_HOMING_SWITCH_TYPE:
            // Position motor homing switch is active HIGH with input pulldown
            return digitalRead((int)POSITION_MOTOR_HOMING_SWITCH) == HIGH;
        default:
            Serial.println("ERROR: Unknown switch type for readLimitSwitch operation");
            return false;
    }
}

bool readHomeSwitch(SwitchType switchType) {
    // For now, home switches are the same as limit switches
    // This function can be expanded if separate home switches are added
    return readLimitSwitch(switchType);
}

//* ************************************************************************
//* ************************ LEGACY SENSOR READING FUNCTIONS *************
//* ************************************************************************
//! Legacy functions maintained for compatibility

bool readWoodSensor() {
    return readSensor(WOOD_SENSOR_TYPE);
}

bool readWoodSuctionSensor() {
    return readSensor(WOOD_SUCTION_SENSOR_TYPE);
}

bool readCutMotorHomingSwitch() {
    return readLimitSwitch(CUT_MOTOR_HOMING_SWITCH_TYPE);
}

bool readPositionMotorHomingSwitch() {
    return readLimitSwitch(POSITION_MOTOR_HOMING_SWITCH_TYPE);
} 