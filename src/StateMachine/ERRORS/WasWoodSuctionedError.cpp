#include "StateMachine/StateMachine.h"
#include "StateMachine/MotorFunctions.h"
#include "StateMachine/SensorFunctions.h"
#include <Arduino.h>

//* ************************************************************************
//* ************************ WAS WOOD SUCTIONED ERROR *********************
//* ************************************************************************
//! Error handling for wood suction detection failures
//! Functions for detecting, handling, and recovering from wood suction errors

// External variable declarations
extern bool woodSuctionError;
extern SystemState currentState;

// Error state variables
bool woodSuctionErrorDetected = false;
unsigned long woodSuctionErrorTime = 0;
bool woodSuctionErrorHandled = false;
unsigned long lastSuctionCheck = 0;

//* ************************************************************************
//* ************************ ERROR DETECTION ******************************
//* ************************************************************************

void checkWoodSuctionError() {
    // Check wood suction sensor periodically
    if (millis() - lastSuctionCheck > 100) { // Check every 100ms
        bool woodSuctioned = readWoodSuctionSensor(); // Use proper sensor function
        
        // If wood should be suctioned but sensor doesn't detect it
        if (!woodSuctioned && currentState == YESWOOD) {
            if (!woodSuctionError) {
                woodSuctionError = true;
                woodSuctionErrorDetected = true;
                woodSuctionErrorTime = millis();
                Serial.println("ERROR: Wood suction failed - WasWoodSuctionedError detected");
            }
        }
        
        lastSuctionCheck = millis();
    }
}

bool isWoodSuctionErrorActive() {
    return woodSuctionError;
}

void forceTriggerWoodSuctionError() {
    woodSuctionError = true;
    woodSuctionErrorDetected = true;
    woodSuctionErrorTime = millis();
    Serial.println("Wood suction error manually triggered");
}

//* ************************************************************************
//* ************************ ERROR HANDLING *******************************
//* ************************************************************************

void handleWoodSuctionError() {
    if (woodSuctionError && !woodSuctionErrorHandled) {
        Serial.println("Handling wood suction error...");
        
        // Stop all motors immediately
        stopCutMotor();
        stopPositionMotor();
        
        // Retract safety components
        retractClamp(POSITION_CLAMP_TYPE);
        retractClamp(WOOD_SECURE_CLAMP_TYPE);
        
        // Set error LED pattern
        turnRedLedOn();
        
        woodSuctionErrorHandled = true;
        Serial.println("Wood suction error handling complete - system in safe state");
    }
}

void handleWoodSuctionErrorLedBlink(unsigned long& lastBlinkTimeRef, bool& blinkStateRef) {
    if (millis() - lastBlinkTimeRef >= 1500) { // Blink RED every 1.5 seconds
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

void resetWoodSuctionError() {
    woodSuctionError = false;
    woodSuctionErrorDetected = false;
    woodSuctionErrorHandled = false;
    Serial.println("Wood suction error flags reset");
}

void acknowledgeWoodSuctionError() {
    if (woodSuctionError) {
        Serial.println("Wood suction error acknowledged - preparing for recovery");
        resetWoodSuctionError();
        
        // Return to IDLE state for manual intervention
        if (currentState == ERROR) {
            changeState(ERROR_RESET);
        }
    }
}

void attemptWoodSuctionRecovery() {
    if (woodSuctionError) {
        Serial.println("Attempting wood suction recovery...");
        
        // Recovery would need to be handled by external suction system
        delay(2000); // Wait 2 seconds
        
        // Check if recovery was successful
        if (readWoodSuctionSensor()) {
            Serial.println("Wood suction recovery successful");
            resetWoodSuctionError();
        } else {
            Serial.println("Wood suction recovery failed - manual intervention required");
        }
    }
}

//* ************************************************************************
//* ************************ ERROR STATUS **********************************
//* ************************************************************************

void printWoodSuctionErrorStatus() {
    Serial.println("=== Wood Suction Error Status ===");
    Serial.print("Error Active: ");
    Serial.println(woodSuctionError ? "YES" : "NO");
    Serial.print("Error Detected: ");
    Serial.println(woodSuctionErrorDetected ? "YES" : "NO");
    Serial.print("Error Handled: ");
    Serial.println(woodSuctionErrorHandled ? "YES" : "NO");
    Serial.print("Suction Sensor State: ");
    Serial.println(readWoodSuctionSensor() ? "ACTIVE" : "INACTIVE");
    Serial.println("==================================");
} 