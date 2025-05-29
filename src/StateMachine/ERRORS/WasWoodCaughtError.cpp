#include "StateMachine/StateMachine.h"
#include "StateMachine/SensorFunctions.h"
#include "StateMachine/MotorFunctions.h"
#include <Arduino.h>

//* ************************************************************************
//* ************************ WAS WOOD CAUGHT ERROR ***********************
//* ************************************************************************
//! Error handling for wood caught detection failures
//! Functions for detecting, handling, and recovering from wood caught errors

// External variable declarations
extern bool wasWoodCaughtError;
extern bool woodCaughtCheckPending;
extern unsigned long woodCaughtCheckTime;
extern SystemState currentState;

// Error state variables
bool woodCaughtErrorDetected = false;
unsigned long woodCaughtErrorTime = 0;
bool woodCaughtErrorHandled = false;

//* ************************************************************************
//* ************************ ERROR DETECTION ******************************
//* ************************************************************************

void checkWoodCaughtError() {
    if (woodCaughtCheckPending && millis() >= woodCaughtCheckTime) {
        // Check wood caught using the main wood sensor
        bool woodDetected = readWoodSensor();
        
        if (!woodDetected) {
            // Wood was not caught - error condition
            wasWoodCaughtError = true;
            woodCaughtErrorDetected = true;
            woodCaughtErrorTime = millis();
            Serial.println("ERROR: Wood was not caught - WasWoodCaughtError detected");
        } else {
            Serial.println("Wood caught successfully - no error");
        }
        
        woodCaughtCheckPending = false;
    }
}

bool isWoodCaughtErrorActive() {
    return wasWoodCaughtError;
}

//* ************************************************************************
//* ************************ ERROR HANDLING *******************************
//* ************************************************************************

void handleWoodCaughtError() {
    if (wasWoodCaughtError && !woodCaughtErrorHandled) {
        Serial.println("Handling wood caught error...");
        
        // Stop all motors
        stopCutMotor();
        stopPositionMotor();
        
        // Retract all clamps and cylinders to safe positions
        retractAllCylinders();
        
        // Set error LED pattern
        turnRedLedOn();
        
        woodCaughtErrorHandled = true;
        Serial.println("Wood caught error handling complete - system in safe state");
    }
}

void handleWoodCaughtErrorLedBlink(unsigned long& lastBlinkTimeRef, bool& blinkStateRef) {
    if (millis() - lastBlinkTimeRef >= 1000) { // Blink RED once per second
        lastBlinkTimeRef = millis();
        blinkStateRef = !blinkStateRef;
        if (blinkStateRef) {
            turnRedLedOn();
        } else {
            turnRedLedOff();
        }
        // Keep other LEDs off during error
        turnYellowLedOff();
        turnGreenLedOff();
        turnBlueLedOff();
    }
}

//* ************************************************************************
//* ************************ ERROR RECOVERY *******************************
//* ************************************************************************

void resetWoodCaughtError() {
    wasWoodCaughtError = false;
    woodCaughtErrorDetected = false;
    woodCaughtErrorHandled = false;
    woodCaughtCheckPending = false;
    Serial.println("Wood caught error flags reset");
}

void acknowledgeWoodCaughtError() {
    if (wasWoodCaughtError) {
        Serial.println("Wood caught error acknowledged - preparing for recovery");
        resetWoodCaughtError();
        
        // Return to IDLE state for manual intervention
        if (currentState == ERROR) {
            changeState(ERROR_RESET);
        }
    }
}

//* ************************************************************************
//* ************************ ERROR STATUS **********************************
//* ************************************************************************

void printWoodCaughtErrorStatus() {
    Serial.println("=== Wood Caught Error Status ===");
    Serial.print("Error Active: ");
    Serial.println(wasWoodCaughtError ? "YES" : "NO");
    Serial.print("Error Detected: ");
    Serial.println(woodCaughtErrorDetected ? "YES" : "NO");
    Serial.print("Error Handled: ");
    Serial.println(woodCaughtErrorHandled ? "YES" : "NO");
    Serial.print("Check Pending: ");
    Serial.println(woodCaughtCheckPending ? "YES" : "NO");
    Serial.println("=================================");
} 