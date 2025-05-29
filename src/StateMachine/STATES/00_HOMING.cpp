#include "StateMachine/StateMachine.h"
#include <Arduino.h>

//* ************************************************************************
//* ************************ HOMING STATE ***************************
//* ************************************************************************
//! HOMING state implementation
//! System performs homing operations for both cut motor and position motor
//! Cut motor: Home in negative direction, stop on switch, set position to 0
//! Position motor: Home in positive direction, stop on switch, set position to (2 + POSITION_TRAVEL_DISTANCE), then move to POSITION_TRAVEL_DISTANCE

// External variable declarations
extern SystemState currentState;
extern bool stateChanged;

void executeHOMING() {
    static bool entryExecuted = false;
    static unsigned long lastStatusMessage = 0;
    
    // Execute entry actions only once when entering this state
    if (stateChanged && !entryExecuted) {
        //! Entry actions for HOMING state
        Serial.println("Entering HOMING state");
        Serial.println("Starting complete homing sequence");
        
        // Start the homing sequence
        startHomingSequence();
        
        entryExecuted = true;
        stateChanged = false;
        
        Serial.println("HOMING state entry complete - homing sequence started");
    }
    
    // Reset entry flag when leaving state
    if (currentState != HOMING) {
        entryExecuted = false;
    }
    
    //! Main HOMING loop actions
    updateHomingSequence();
    
    // Print status messages every 2 seconds
    if (millis() - lastStatusMessage > 2000) {
        Serial.print("HOMING status - Cut motor: ");
        Serial.print(isCutMotorHomingComplete() ? "COMPLETE" : "IN PROGRESS");
        Serial.print(", Position motor: ");
        Serial.print(isPositionMotorHomingComplete() ? "COMPLETE" : "IN PROGRESS");
        Serial.print(", Travel position: ");
        Serial.println(isPositionMotorAtTravelPosition() ? "COMPLETE" : "IN PROGRESS");
        lastStatusMessage = millis();
    }
    
    // Check if homing sequence is complete
    if (isHomingSequenceComplete()) {
        Serial.println("Homing sequence completed successfully - transitioning to IDLE");
        changeState(IDLE);
    }
} 