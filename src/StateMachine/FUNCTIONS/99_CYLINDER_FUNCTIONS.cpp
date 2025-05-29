#include "StateMachine/StateMachine.h"

// External variable declarations for catcher clamp timing
extern unsigned long catcherClampEngageTime;
extern bool catcherClampIsEngaged;

//* ************************************************************************
//* ************************ CYLINDER FUNCTIONS ***************************
//* ************************************************************************
//! Pneumatic cylinder control functions
//! Basic pneumatic operations that can be shared across states
//! THIS FILE CONTAINS ONLY CYLINDER-RELATED FUNCTIONS

//* ************************************************************************
//* ************************ PNEUMATIC CYLINDER FUNCTIONS ***************
//* ************************************************************************

// Unified Clamp Control Functions
void extendClamp(ClampType clamp) {
    switch(clamp) {
        case POSITION_CLAMP_TYPE:
            digitalWrite(POSITION_CLAMP, HIGH);
            Serial.println("Position clamp extended");
            break;
        case WOOD_SECURE_CLAMP_TYPE:
            digitalWrite(WOOD_SECURE_CLAMP, HIGH);
            Serial.println("Wood secure clamp extended");
            break;
        case CATCHER_CLAMP_TYPE:
            digitalWrite(CATCHER_CLAMP_PIN, HIGH);
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
            digitalWrite(POSITION_CLAMP, LOW);
            Serial.println("Position clamp retracted");
            break;
        case WOOD_SECURE_CLAMP_TYPE:
            digitalWrite(WOOD_SECURE_CLAMP, LOW);
            Serial.println("Wood secure clamp retracted");
            break;
        case CATCHER_CLAMP_TYPE:
            digitalWrite(CATCHER_CLAMP_PIN, LOW);
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