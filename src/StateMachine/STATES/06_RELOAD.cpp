#include "StateMachine/StateMachine.h"
#include <Bounce2.h>
#include <Arduino.h>

//* ************************************************************************
//* ************************ RELOAD STATE **********************************
//* ************************************************************************
//! RELOAD state implementation
//! System enters safe reload mode for manual wood loading/unloading
//! All clamps retracted, system safe for manual intervention

// External variable declarations
extern SystemState currentState;
extern bool stateChanged;
extern Bounce reloadSwitch;
extern bool isReloadMode;

void executeRELOAD() {
    static bool entryExecuted = false;
    
    //! ************************************************************************
    //! STEP 1: EXECUTE ENTRY ACTIONS (ONLY ONCE WHEN ENTERING STATE)
    //! ************************************************************************
    if (stateChanged && !entryExecuted) {
        Serial.println("=== ENTERING RELOAD STATE ===");
        Serial.println("System entering safe reload mode");
        Serial.println("All clamps retracting for manual intervention");
        
        // Set reload mode flag
        isReloadMode = true;
        
        // Retract clamps for safe manual access (excluding catcher clamp - not needed for reload)
        retractClamp(POSITION_CLAMP_TYPE);
        retractClamp(WOOD_SECURE_CLAMP_TYPE);
        
        // Set entry flags
        entryExecuted = true;
        stateChanged = false;
        
        Serial.println("RELOAD state entry complete - system safe for manual operation");
        Serial.println("Turn OFF reload switch to exit reload mode");
    }
    
    //! ************************************************************************
    //! STEP 2: RESET ENTRY FLAG WHEN LEAVING STATE
    //! ************************************************************************
    if (currentState != RELOAD) {
        entryExecuted = false;
        isReloadMode = false;
        return;
    }
    
    //! ************************************************************************
    //! STEP 3: UPDATE RELOAD SWITCH STATE
    //! ************************************************************************
    reloadSwitch.update();
    
    //! ************************************************************************
    //! STEP 4: MONITOR RELOAD SWITCH FOR EXIT CONDITION
    //! ************************************************************************
    if (reloadSwitch.read() == LOW) {
        Serial.println("Reload switch turned OFF - exiting reload mode");
        Serial.println("Re-engaging clamps");
        
        // Clear reload mode flag
        isReloadMode = false;
        
        // Re-engage clamps for normal operation
        extendClamp(POSITION_CLAMP_TYPE);
        extendClamp(WOOD_SECURE_CLAMP_TYPE);
        
        Serial.println("Exiting reload mode - returning to IDLE state");
        changeState(IDLE);
        return;
    }
} 