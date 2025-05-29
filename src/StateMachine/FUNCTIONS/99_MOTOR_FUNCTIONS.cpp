#include "StateMachine/StateMachine.h"

// External variable declarations for catcher clamp timing
extern unsigned long catcherClampEngageTime;
extern bool catcherClampIsEngaged;

//* ************************************************************************
//* ************************ MOTOR FUNCTIONS ***************************
//* ************************************************************************
//! Motor control functions for steppers and pneumatic cylinders
//! Basic movement functions that can be shared across states
//! THIS FILE CONTAINS ONLY MOTOR-RELATED FUNCTIONS

// Individual Motor Step Functions
void stepperStepCutMotor(bool direction) {
    digitalWrite((int)CUT_MOTOR_DIR_PIN, direction ? HIGH : LOW);
    digitalWrite((int)CUT_MOTOR_PULSE_PIN, HIGH);
    delayMicroseconds(2);  // Correct - stepper drivers need microsecond pulses
    digitalWrite((int)CUT_MOTOR_PULSE_PIN, LOW);
    delayMicroseconds(2);  // Correct - stepper drivers need microsecond pulses
}

void stepperStepPositionMotor(bool direction) {
    digitalWrite((int)POSITION_MOTOR_DIR_PIN, direction ? HIGH : LOW);
    digitalWrite((int)POSITION_MOTOR_PULSE_PIN, HIGH);
    delayMicroseconds(2);  // Correct - stepper drivers need microsecond pulses
    digitalWrite((int)POSITION_MOTOR_PULSE_PIN, LOW);
    delayMicroseconds(2);  // Correct - stepper drivers need microsecond pulses
}

// Multi-Step Motor Movement Functions
void moveCutMotorSteps(float steps, bool direction, float delayBetweenSteps) {
    for (float i = 0.0; i < steps; i += 1.0) {
        stepperStepCutMotor(direction);
        delay((int)delayBetweenSteps);  // Correct - delay between steps in milliseconds
    }
}

void movePositionMotorSteps(float steps, bool direction, float delayBetweenSteps) {
    for (float i = 0.0; i < steps; i += 1.0) {
        stepperStepPositionMotor(direction);
        delay((int)delayBetweenSteps);  // Correct - delay between steps in milliseconds
    }
}

//* ************************************************************************
//* ************************ PNEUMATIC CYLINDER FUNCTIONS ***************
//* ************************************************************************

// Unified Clamp Control Functions
void extendClamp(ClampType clamp) {
    switch(clamp) {
        case POSITION_CLAMP_TYPE:
            digitalWrite((int)POSITION_CLAMP, HIGH);
            Serial.println("Position clamp extended");
            break;
        case WOOD_SECURE_CLAMP_TYPE:
            digitalWrite((int)WOOD_SECURE_CLAMP, HIGH);
            Serial.println("Wood secure clamp extended");
            break;
        case CATCHER_CLAMP_TYPE:
            digitalWrite((int)CATCHER_CLAMP_PIN, HIGH);
            catcherClampEngageTime = millis();
            catcherClampIsEngaged = true;
            Serial.println("Catcher clamp extended");
            break;
        default:
            Serial.println("ERROR: Unknown clamp type for extend operation");
            break;
    }
}

void retractClamp(ClampType clamp) {
    switch(clamp) {
        case POSITION_CLAMP_TYPE:
            digitalWrite((int)POSITION_CLAMP, LOW);
            Serial.println("Position clamp retracted");
            break;
        case WOOD_SECURE_CLAMP_TYPE:
            digitalWrite((int)WOOD_SECURE_CLAMP, LOW);
            Serial.println("Wood secure clamp retracted");
            break;
        case CATCHER_CLAMP_TYPE:
            digitalWrite((int)CATCHER_CLAMP_PIN, LOW);
            catcherClampIsEngaged = false;
            Serial.println("Catcher clamp retracted");
            break;
        default:
            Serial.println("ERROR: Unknown clamp type for retract operation");
            break;
    }
}

// Collective Operations
void retractAllCylinders() {
    retractClamp(POSITION_CLAMP_TYPE);
    retractClamp(WOOD_SECURE_CLAMP_TYPE);
    retractClamp(CATCHER_CLAMP_TYPE);
    Serial.println("All cylinders retracted");
}

void extendAllCylinders() {
    extendClamp(POSITION_CLAMP_TYPE);
    extendClamp(WOOD_SECURE_CLAMP_TYPE);
    extendClamp(CATCHER_CLAMP_TYPE);
    Serial.println("All cylinders extended");
} 