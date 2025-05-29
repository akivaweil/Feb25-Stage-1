#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ PIN DEFINITIONS *****************************
//* ************************************************************************
// Hardware pin assignments for the Automated Table Saw - Stage 1
// ESP32-S3 based system with stepper motors, servo, sensors, and switches

//* ************************************************************************
//* ************************ MOTOR PINS ***********************************
//* ************************************************************************
// Stepper motor control pins
const int CUT_MOTOR_PULSE_PIN = 12;              // Cut motor step pulse pin
const int CUT_MOTOR_DIR_PIN = 11;                // Cut motor direction pin
const int POSITION_MOTOR_PULSE_PIN = 17;         // Position motor step pulse pin
const int POSITION_MOTOR_DIR_PIN = 18;           // Position motor direction pin

//* ************************************************************************
//* ************************ SERVO PINS ***********************************
//* ************************************************************************
// Servo control pins
const int CATCHER_SERVO_PIN = 14;                // Servo control pin for catcher

//* ************************************************************************
//* ************************ SWITCH & SENSOR PINS ************************
//* ************************************************************************
// Homing switches (Active HIGH - input pulldown)
const int CUT_MOTOR_HOMING_SWITCH = 3;           // Cut motor homing limit switch
const int POSITION_MOTOR_HOMING_SWITCH = 16;     // Position motor homing limit switch

// Control switches (Active HIGH - input pulldown)
const int RELOAD_SWITCH = 6;                     // Reload/reset switch
const int START_CYCLE_SWITCH = 5;                // Start cutting cycle switch
const int FIX_POSITION_BUTTON = 41;              // Manual position adjustment button

// Sensors (Active LOW - input pullup)
const int WOOD_SENSOR = 10;                      // Reads LOW when wood is detected
const int WAS_WOOD_SUCTIONED_SENSOR = 39;        // Reads LOW when wood suction is confirmed

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
const int STATUS_LED_PIN = 2;  // Additional status LED

// ... existing code ... 