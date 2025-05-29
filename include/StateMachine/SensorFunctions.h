#ifndef SENSOR_FUNCTIONS_H
#define SENSOR_FUNCTIONS_H

#include <Arduino.h>

//* ************************************************************************
//* ************************ SENSOR FUNCTIONS HEADER *********************
//* ************************************************************************
//! Header file for sensor and switch reading functions
//! Function prototypes for all sensors and limit switches

// Sensor Reading Functions
bool readWoodSensor();
bool readWoodSuctionSensor();

// Switch Reading Functions
bool readCutMotorHomingSwitch();
bool readPositionMotorHomingSwitch();

#endif 