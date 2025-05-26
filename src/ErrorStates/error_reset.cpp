#include "ErrorStates/error_reset.h"

// External references to global variables and functions from main.cpp
extern bool errorAcknowledged;
extern bool woodSuctionError;
extern SystemState currentState;
extern void turnRedLedOff();
extern void turnYellowLedOff();

//* ************************************************************************
//* ************************** ERROR_RESET *********************************
//* ************************************************************************
// Handles the reset sequence after an error has been acknowledged.
// Step 1: Turn off red and yellow error LEDs.
// Step 2: Reset errorAcknowledged and woodSuctionError flags.
// Step 3: Transition to STARTUP state to re-initialize the system (which will lead to HOMING).
void handleErrorResetState() {
    Serial.println("Entering error reset state.");
    
    // Turn off error LEDs
    turnRedLedOff();
    turnYellowLedOff();
    
    // Reset flags
    errorAcknowledged = false;
    woodSuctionError = false;
    
    // Return to homing state to re-initialize
    currentState = STARTUP;
    Serial.println("Error reset complete, restarting system. Transitioning to STARTUP.");
} 