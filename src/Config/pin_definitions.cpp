#include "Config/pin_definitions.h"

//* ************************************************************************
//* ************************ PIN DEFINITIONS *****************************
//* ************************************************************************
// Hardware pin assignments for the Automated Table Saw - Stage 1
// ESP32-S3 based system with stepper motors, servo, sensors, and switches

//* ************************************************************************
//* ************************ MOTOR PINS ***********************************
//* ************************************************************************
// Stepper motor control pins
const int CUT_MOTOR_PULSE_PIN = 12;
const int CUT_MOTOR_DIR_PIN = 11;
const int POSITION_MOTOR_PULSE_PIN = 17;
const int POSITION_MOTOR_DIR_PIN = 18;

//* ************************************************************************
//* ************************ SERVO PINS ***********************************
//* ************************************************************************
// Servo control pins
const int CATCHER_SERVO_PIN = 14;

//* ************************************************************************
//* ************************ SWITCH & SENSOR PINS ************************
//* ************************************************************************
// Homing switches (Active HIGH - input pulldown)
const int CUT_MOTOR_HOMING_SWITCH = 3;
const int POSITION_MOTOR_HOMING_SWITCH = 16;

// Control switches (Active HIGH - input pulldown)
const int RELOAD_SWITCH = 6;
const int START_CYCLE_SWITCH = 5;
const int FIX_POSITION_BUTTON = 41;

// Sensors (Active LOW - input pullup)
const int WOOD_SENSOR = 10;
const int WAS_WOOD_SUCTIONED_SENSOR = 39;

//* ************************************************************************
//* ************************ CLAMP PINS ***********************************
//* ************************************************************************
// Pneumatic clamp control pins
const int POSITION_CLAMP = 36;
const int WOOD_SECURE_CLAMP = 48;
const int CATCHER_CLAMP_PIN = 42;

//* ************************************************************************
//* ************************ SIGNAL PINS **********************************
//* ************************************************************************
// Communication pins for external systems
const int TA_SIGNAL_OUT_PIN = 8;  // Transfer Arm signal

//* ************************************************************************
//* ************************ LED PINS *************************************
//* ************************************************************************
// Status indication LEDs
const int RED_LED = 47;
const int YELLOW_LED = 21;
const int GREEN_LED = 37;
const int BLUE_LED = 19; 