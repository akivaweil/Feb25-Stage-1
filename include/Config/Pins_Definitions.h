#ifndef PINS_DEFINITIONS_H
#define PINS_DEFINITIONS_H

//* ************************************************************************
//* ************************ PIN DEFINITIONS *****************************
//* ************************************************************************
// Hardware pin assignments for the Automated Table Saw - Stage 1
// ESP32-S3 based system with stepper motors, servo, sensors, and switches

//* ************************************************************************
//* ************************ MOTOR PINS ***********************************
//* ************************************************************************
// Stepper motor control pins
extern const int CUT_MOTOR_PULSE_PIN;
extern const int CUT_MOTOR_DIR_PIN;
extern const int POSITION_MOTOR_PULSE_PIN;
extern const int POSITION_MOTOR_DIR_PIN;

//* ************************************************************************
//* ************************ SERVO PINS ***********************************
//* ************************************************************************
// Servo control pins
extern const int CATCHER_SERVO_PIN;

//* ************************************************************************
//* ************************ SWITCH & SENSOR PINS ************************
//* ************************************************************************
// Homing switches (Active HIGH - input pulldown)
extern const int CUT_MOTOR_HOMING_SWITCH;
extern const int POSITION_MOTOR_HOMING_SWITCH;

// Control switches (Active HIGH - input pulldown)
extern const int RELOAD_SWITCH;
extern const int START_CYCLE_SWITCH;
extern const int FIX_POSITION_BUTTON;

// Sensors (Active LOW - input pullup)
extern const int WOOD_SENSOR;
extern const int WAS_WOOD_SUCTIONED_SENSOR;

//* ************************************************************************
//* ************************ CLAMP PINS ***********************************
//* ************************************************************************
// Pneumatic clamp control pins
extern const int POSITION_CLAMP;
extern const int WOOD_SECURE_CLAMP;
extern const int CATCHER_CLAMP_PIN;

//* ************************************************************************
//* ************************ SIGNAL PINS **********************************
//* ************************************************************************
// Communication pins for external systems
extern const int TA_SIGNAL_OUT_PIN;  // Transfer Arm signal

//* ************************************************************************
//* ************************ LED PINS *************************************
//* ************************************************************************
// Status indication LEDs
extern const int RED_LED;
extern const int YELLOW_LED;
extern const int GREEN_LED;
extern const int BLUE_LED;
extern const int STATUS_LED_PIN;

#endif // PINS_DEFINITIONS_H 