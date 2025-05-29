#include "StateMachine/StateMachine.h"
#include <ESP32Servo.h>

//* ************************************************************************
//* ************************ TIMING FUNCTIONS ****************************
//* ************************************************************************
//! Functions that handle timing-based operations and early activation
//! Cut motor position-based activation during cutting operations

// External variable declarations
extern AccelStepper cutMotor;
extern Servo catcherServo;
extern unsigned long catcherServoActiveStartTime;
extern bool catcherServoIsActiveAndTiming;
extern unsigned long catcherClampEngageTime;
extern bool catcherClampIsEngaged;

//* ************************************************************************
//* ************************ EARLY ACTIVATION FUNCTIONS ******************
//* ************************************************************************
//! Early activation based on cut motor position during cutting state

void checkCatcherServoEarlyActivation() {
    float currentCutPositionInches = (float)cutMotor.currentPosition() / CUT_MOTOR_STEPS_PER_INCH;
    float targetCutPositionInches = CUT_TRAVEL_DISTANCE;
    float earlyActivationPositionInches = targetCutPositionInches - CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;
    
    if (currentCutPositionInches >= earlyActivationPositionInches && !catcherServoIsActiveAndTiming) {
        catcherServo.write(CATCHER_SERVO_ACTIVE_POSITION);
        catcherServoActiveStartTime = millis();
        catcherServoIsActiveAndTiming = true;
        Serial.print("Catcher servo early activation at cut position ");
        Serial.print(currentCutPositionInches);
        Serial.print(" inches (");
        Serial.print(CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES);
        Serial.println(" inches before cut completion)");
    }
}

void checkCatcherClampEarlyActivation() {
    float currentCutPositionInches = (float)cutMotor.currentPosition() / CUT_MOTOR_STEPS_PER_INCH;
    float targetCutPositionInches = CUT_TRAVEL_DISTANCE;
    float earlyActivationPositionInches = targetCutPositionInches - CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;
    
    if (currentCutPositionInches >= earlyActivationPositionInches && !catcherClampIsEngaged) {
        extendClamp(CATCHER_CLAMP_TYPE);
        catcherClampEngageTime = millis();
        catcherClampIsEngaged = true;
        Serial.print("Catcher clamp early activation at cut position ");
        Serial.print(currentCutPositionInches);
        Serial.print(" inches (");
        Serial.print(CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES);
        Serial.println(" inches before cut completion)");
    }
}

//* ************************************************************************
//* ************************ TIMING COORDINATION **************************
//* ************************************************************************

void handleCatcherClampDisengage() {
    if (catcherClampIsEngaged && (millis() - catcherClampEngageTime >= CATCHER_CLAMP_ENGAGE_DURATION_MS)) {
        retractClamp(CATCHER_CLAMP_TYPE);
        catcherClampIsEngaged = false;
        Serial.println("Catcher clamp disengaged after duration timeout");
    }
} 