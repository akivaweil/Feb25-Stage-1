#include "StateMachine/StateMachine.h"
#include <Bounce2.h>
#include <Arduino.h>
#include "OTA_Manager.h"

//* ************************************************************************
//* ************************ IDLE STATE ***********************************
//* ************************************************************************
//! IDLE state implementation - Machine waiting and monitoring for input
//! 
//! Step-by-step sequence:
//! 1. Check for OTA uploads
//! 2. Handle start switch safety to prevent accidental startup
//! 3. Check start cycle switch - if HIGH and switch is safe, switch to CUTTING state
//! 4. Check reload switch - if HIGH, switch to RELOAD state

// External variable declarations
extern SystemState currentState;
extern bool stateChanged;
extern Bounce startCycleSwitch;
extern Bounce reloadSwitch;
extern bool startSwitchSafe;
extern bool startSwitchSafeAfterNoWood;

void executeIDLE() {
    //! ************************************************************************
    //! STEP 1: CHECK FOR OTA UPLOADS
    //! ************************************************************************
    handleOTA();
    
    //! ************************************************************************
    //! STEP 2: HANDLE START SWITCH SAFETY
    //! ************************************************************************
    handleStartSwitchSafety();
    
    //! ************************************************************************
    //! STEP 3: CHECK START CYCLE SWITCH (WITH SAFETY CHECK)
    //! ************************************************************************
    startCycleSwitch.update();
    if (startCycleSwitch.read() == HIGH) {
        if (startSwitchSafe && startSwitchSafeAfterNoWood) {
            Serial.println("Start cycle switch HIGH and all safety checks passed - transitioning to CUTTING state");
            changeState(CUTTING);
            return;
        } else {
            if (!startSwitchSafe) {
                Serial.println("Start cycle switch HIGH but startup safety NOT MET - Turn switch OFF first to enable safety");
            } else if (!startSwitchSafeAfterNoWood) {
                Serial.println("Start cycle switch HIGH but post-NOWOOD safety NOT MET - Turn switch OFF->ON to acknowledge reload");
            }
        }
    }
    
    //! ************************************************************************
    //! STEP 4: CHECK RELOAD SWITCH
    //! ************************************************************************
    reloadSwitch.update();
    if (reloadSwitch.read() == HIGH) {
        Serial.println("Reload switch HIGH - transitioning to RELOAD state");
        changeState(RELOAD);
        return;
    }
} 