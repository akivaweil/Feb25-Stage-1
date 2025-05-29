#include "StateMachine/StateMachine.h"
#include <AccelStepper.h>
#include <Bounce2.h>
#include <Arduino.h>

//* ************************************************************************
//* ************************ CUT MOTOR FAILED TO HOME ERROR **************
//* ************************************************************************
//! Error handling for cut motor homing failures
//! Functions for detecting, handling, and recovering from cut motor homing errors

// External variable declarations
extern AccelStepper cutMotor;
extern Bounce cutHomingSwitch;
extern SystemState currentState;
extern bool isHomed;

// Error state variables
bool cutMotorFailedtoHomeError = false;
bool cutMotorHomeErrorDetected = false;
unsigned long cutMotorHomeErrorTime = 0;
bool cutMotorHomeErrorHandled = false;
int cutMotorHomingAttempts = 0;
const int MAX_HOMING_ATTEMPTS = 3;
unsigned long homingStartTime = 0;
const unsigned long HOMING_TIMEOUT_MS = 30000; // 30 seconds timeout

//* ************************************************************************
//* ************************ ERROR DETECTION ******************************
//* ************************************************************************

void checkCutMotorHomingTimeout() {
    if (currentState == HOMING && cutMotor.distanceToGo() != 0) {
        if (millis() - homingStartTime > HOMING_TIMEOUT_MS) {
            cutMotorFailedtoHomeError = true;
            cutMotorHomeErrorDetected = true;
            cutMotorHomeErrorTime = millis();
            Serial.println("ERROR: Cut motor homing timeout - CutMotorFailedtoHomeError detected");
            
            // Stop the motor immediately
            cutMotor.stop();
            cutMotor.setCurrentPosition(cutMotor.currentPosition());
        }
    }
}

void checkCutMotorHomingFailure() {
    if (currentState == HOMING && cutMotor.distanceToGo() == 0) {
        // Motor stopped but home switch not triggered
        cutHomingSwitch.update();
        if (cutHomingSwitch.read() != HIGH) {
            cutMotorFailedtoHomeError = true;
            cutMotorHomeErrorDetected = true;
            cutMotorHomeErrorTime = millis();
            Serial.println("ERROR: Cut motor stopped without reaching home switch - CutMotorFailedtoHomeError detected");
        }
    }
}

bool isCutMotorHomeErrorActive() {
    return cutMotorFailedtoHomeError;
}

void startCutMotorHomingWithErrorDetection() {
    homingStartTime = millis();
    cutMotorHomingAttempts++;
    Serial.print("Starting cut motor homing attempt ");
    Serial.print(cutMotorHomingAttempts);
    Serial.print(" of ");
    Serial.println(MAX_HOMING_ATTEMPTS);
}

//* ************************************************************************
//* ************************ ERROR HANDLING *******************************
//* ************************************************************************

void handleCutMotorHomeError() {
    if (cutMotorFailedtoHomeError && !cutMotorHomeErrorHandled) {
        Serial.println("Handling cut motor homing error...");
        
        // Stop the cut motor immediately
        cutMotor.stop();
        cutMotor.setCurrentPosition(cutMotor.currentPosition());
        
        // Stop position motor as well for safety
        stopPositionMotor();
        
        // Retract all clamps and cylinders to safe positions
        retractAllCylinders();
        
        // Set error LED pattern
        turnRedLedOn();
        
        // Clear the global homed flag
        isHomed = false;
        
        cutMotorHomeErrorHandled = true;
        Serial.println("Cut motor homing error handling complete - system in safe state");
    }
}

void handleCutMotorHomeErrorLedBlink(unsigned long& lastBlinkTimeRef, bool& blinkStateRef) {
    if (millis() - lastBlinkTimeRef >= 500) { // Fast blink RED every 500ms
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

void resetCutMotorHomeError() {
    cutMotorFailedtoHomeError = false;
    cutMotorHomeErrorDetected = false;
    cutMotorHomeErrorHandled = false;
    cutMotorHomingAttempts = 0;
    Serial.println("Cut motor homing error flags reset");
}

void acknowledgeCutMotorHomeError() {
    if (cutMotorFailedtoHomeError) {
        Serial.println("Cut motor homing error acknowledged - preparing for recovery");
        resetCutMotorHomeError();
        
        // Return to HOMING state for retry or IDLE for manual intervention
        if (currentState == ERROR) {
            if (cutMotorHomingAttempts < MAX_HOMING_ATTEMPTS) {
                Serial.println("Retrying cut motor homing...");
                changeState(HOMING);
            } else {
                Serial.println("Maximum homing attempts reached - manual intervention required");
                changeState(ERROR_RESET);
            }
        }
    }
}

void attemptCutMotorHomeRecovery() {
    if (cutMotorFailedtoHomeError && cutMotorHomingAttempts < MAX_HOMING_ATTEMPTS) {
        Serial.println("Attempting cut motor homing recovery...");
        
        // Move motor slightly away from current position
        cutMotor.move(1000); // Move 1000 steps away
        while (cutMotor.distanceToGo() != 0) {
            cutMotor.run();
            delay(10);
        }
        
        // Reset error flags for retry
        resetCutMotorHomeError();
        
        // Start new homing attempt
        startCutMotorHomingWithErrorDetection();
        homeCutMotorBlocking(cutHomingSwitch, 30000); // Use new blocking homing function
    } else if (cutMotorHomingAttempts >= MAX_HOMING_ATTEMPTS) {
        Serial.println("Cut motor homing recovery failed - maximum attempts exceeded");
    }
}

//* ************************************************************************
//* ************************ ERROR STATUS **********************************
//* ************************************************************************

void printCutMotorHomeErrorStatus() {
    Serial.println("=== Cut Motor Homing Error Status ===");
    Serial.print("Error Active: ");
    Serial.println(cutMotorFailedtoHomeError ? "YES" : "NO");
    Serial.print("Error Detected: ");
    Serial.println(cutMotorHomeErrorDetected ? "YES" : "NO");
    Serial.print("Error Handled: ");
    Serial.println(cutMotorHomeErrorHandled ? "YES" : "NO");
    Serial.print("Homing Attempts: ");
    Serial.print(cutMotorHomingAttempts);
    Serial.print(" of ");
    Serial.println(MAX_HOMING_ATTEMPTS);
    Serial.print("Home Switch State: ");
    cutHomingSwitch.update();
    Serial.println(cutHomingSwitch.read() == HIGH ? "ACTIVE" : "INACTIVE");
    Serial.print("Cut Motor Running: ");
    Serial.println(cutMotor.distanceToGo() != 0 ? "YES" : "NO");
    Serial.println("=====================================");
}

//* ************************************************************************
//* ************************ ERROR TRIGGERING *****************************
//* ************************************************************************

void triggerCutMotorHomeError() {
    cutMotorFailedtoHomeError = true;
    cutMotorHomeErrorDetected = true;
    cutMotorHomeErrorTime = millis();
    Serial.println("ERROR: Cut motor homing error triggered manually - CutMotorFailedtoHomeError activated");
    
    // Stop the cut motor immediately
    cutMotor.stop();
    cutMotor.setCurrentPosition(cutMotor.currentPosition());
    
    // Transition to ERROR state
    changeState(ERROR);
}

void forceTriggerCutMotorHomeError() {
    cutMotorFailedtoHomeError = true;
    cutMotorHomeErrorDetected = true;
    cutMotorHomeErrorTime = millis();
    Serial.println("Cut motor homing error manually triggered");
    
    // Stop the cut motor immediately
    cutMotor.stop();
    cutMotor.setCurrentPosition(cutMotor.currentPosition());
} 