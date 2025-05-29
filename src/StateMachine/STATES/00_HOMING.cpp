#include "StateMachine/StateMachine.h"
#include <Arduino.h>

//* ************************************************************************
//* ************************ HOMING STATE ***************************
//* ************************************************************************
//! HOMING state implementation - simplified blocking homing
//! System performs complete homing sequence for both motors

// External variable declarations
extern SystemState currentState;
extern bool stateChanged;
extern bool isHomed;

void executeHOMING() {
    static bool entryExecuted = false;
    
    // Execute entry actions only once when entering this state
    if (stateChanged && !entryExecuted) {
        //! Entry actions for HOMING state
        Serial.println("Entering HOMING state");
        
        // Execute complete homing sequence (blocking)
        executeCompleteHomingSequence();
        
        entryExecuted = true;
        stateChanged = false;
        
        Serial.println("HOMING state complete - transitioning to IDLE");
        changeState(IDLE);
    }
    
    // Reset entry flag when leaving state
    if (currentState != HOMING) {
        entryExecuted = false;
    }
} 