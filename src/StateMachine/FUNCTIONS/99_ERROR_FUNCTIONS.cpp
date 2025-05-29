#include "StateMachine/StateMachine.h"
#include <Bounce2.h>

//* ************************************************************************
//* ************************ ERROR FUNCTIONS ******************************
//* ************************************************************************
//! Functions for handling errors and error acknowledgment
//! Error state management and recovery operations

// External variable declarations
extern Bounce reloadSwitch;
extern bool errorAcknowledged;

//* ************************************************************************
//* ************************ ERROR HANDLING *******************************
//* ************************************************************************

void handleErrorAcknowledgement() {
    // This handles the general error acknowledgement via reloadSwitch
    // It was present in the main loop and also within the CUTTING state's homePositionErrorDetected block.
    if (reloadSwitch.rose() && (currentState == ERROR || currentState == CUTTING)) { // Check if in ERROR or if a cutting error is active
        // For CUTTING state, the homePositionErrorDetected flag logic needs to remain there,
        // but the transition to ERROR_RESET can be centralized if errorAcknowledged is set.
        if (currentState == ERROR) {
            currentState = ERROR_RESET;
            errorAcknowledged = true; // Set flag, main loop will see this for ERROR state
            Serial.println("Error acknowledged by reload switch (from ERROR state). Transitioning to ERROR_RESET.");
        }
        // If in CUTTING, setting errorAcknowledged might be used by the CUTTING state to proceed.
        // The original CUTTING state logic directly transitioned. For now, we set the flag.
        // The calling code in CUTTING will need to check this flag if it relies on it.
        // For direct transition from specific cutting error, that logic is better kept in cutting stage.
        // This function primarily handles the generic ERROR state reset.
    }
} 