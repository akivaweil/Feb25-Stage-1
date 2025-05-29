#include "StateMachine/StateMachine.h"
#include <ESP32Servo.h>

//* ************************************************************************
//* ************************ SIGNALING FUNCTIONS *************************
//* ************************************************************************
//! Functions for communicating with other stages and components
//! Handles Transfer Arm signals and servo coordination

// External variable declarations
extern Servo catcherServo;
extern bool signalTAActive;
extern unsigned long signalTAStartTime;
extern unsigned long catcherServoActiveStartTime;
extern bool catcherServoIsActiveAndTiming;

//* ************************************************************************
//* ************************ TRANSFER ARM SIGNALING **********************
//* ************************************************************************

void sendSignalToTA() {
    // Set the signal pin HIGH to trigger Transfer Arm (active HIGH)
    digitalWrite(TA_SIGNAL_OUT_PIN, HIGH);
    signalTAStartTime = millis();
    signalTAActive = true;
    Serial.println("Signal sent to Transfer Arm (TA)");

    catcherServo.write(CATCHER_SERVO_ACTIVE_POSITION);
    catcherServoActiveStartTime = millis();
    catcherServoIsActiveAndTiming = true;
    Serial.print("Catcher servo moved to ");
    Serial.print(CATCHER_SERVO_ACTIVE_POSITION);
    Serial.println(" degrees with TA signal.");
}

void handleTASignalTiming() { 
    if (signalTAActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
        digitalWrite(TA_SIGNAL_OUT_PIN, LOW); // Return to inactive state (LOW)
        signalTAActive = false;
        Serial.println("Signal to Transfer Arm (TA) completed"); 
    }
}

//* ************************************************************************
//* ************************ SERVO CONTROL ********************************
//* ************************************************************************

void activateCatcherServo() {
    // Move catcher servo to active position
    catcherServo.write(CATCHER_SERVO_ACTIVE_POSITION);
    catcherServoActiveStartTime = millis();
    catcherServoIsActiveAndTiming = true;
    Serial.print("Catcher servo activated to ");
    Serial.print(CATCHER_SERVO_ACTIVE_POSITION);
    Serial.println(" degrees");
}

void handleCatcherServoReturn() {
    // Move catcher servo to home position
    catcherServo.write(CATCHER_SERVO_HOME_POSITION);
    Serial.print("Catcher servo returned to home position (");
    Serial.print(CATCHER_SERVO_HOME_POSITION);
    Serial.println(" degrees).");
} 