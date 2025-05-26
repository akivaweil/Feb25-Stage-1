#include "ErrorStates/standard_error.h"

// External references to global variables and functions from main.cpp
extern unsigned long lastErrorBlinkTime;
extern bool errorBlinkState;
extern bool errorAcknowledged;
extern SystemState currentState;
extern void turnRedLedOn();
extern void turnRedLedOff();
extern void turnYellowLedOn();
extern void turnYellowLedOff();
extern void stopCutMotor();
extern void stopPositionMotor();

//* ************************************************************************
//* ***************************** ERROR ************************************
//* ************************************************************************
// Handles system error states.
// Step 1: Blink red and yellow LEDs to indicate an error.
// Step 2: Ensure cut and position motors are stopped.
// Step 3: Wait for the reload switch to be pressed (rising edge) to acknowledge the error.
// Step 4: Once error is acknowledged, transition to ERROR_RESET state.
void handleStandardErrorState() {
    // Blink error LEDs
    if (millis() - lastErrorBlinkTime > 250) {
        errorBlinkState = !errorBlinkState;
        if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        lastErrorBlinkTime = millis();
    }
    
    // Keep motors stopped
    stopCutMotor();
    stopPositionMotor();
    
    // Wait for reload switch to acknowledge error
    if (errorAcknowledged) {
        currentState = ERROR_RESET;
        Serial.println("Error acknowledged in standard error state. Transitioning to ERROR_RESET.");
    }
} 