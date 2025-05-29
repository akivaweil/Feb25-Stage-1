#include "StateMachine/StateMachine.h"
#include <Bounce2.h>

//* ************************************************************************
//* ************************ IDLE FUNCTIONS *****************************
//* ************************************************************************
//! IDLE state specific functions
//! Functions that handle operations while the system is in IDLE state

//* ************************************************************************
//* ************************ IDLE STATE OPERATIONS **********************
//* ************************************************************************

bool shouldStartCycle() {
    // Condition from IDLE state to start a cycle
    extern bool continuousModeActive;
    extern bool cuttingCycleInProgress;
    extern bool woodSuctionError;
    extern bool startSwitchSafe;
    extern Bounce startCycleSwitch;
    
    return ((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress))
            && !woodSuctionError && startSwitchSafe);
}

void handleReloadMode() {
    extern bool isReloadMode;
    extern Bounce reloadSwitch;
    
    if (currentState == IDLE) {
        bool reloadSwitchOn = reloadSwitch.read() == HIGH;
        if (reloadSwitchOn && !isReloadMode) {
            isReloadMode = true;
            retractClamp(POSITION_CLAMP_TYPE);
            retractClamp(WOOD_SECURE_CLAMP_TYPE);
            turnYellowLedOn();
            Serial.println("Entered reload mode");
        } else if (!reloadSwitchOn && isReloadMode) {
            isReloadMode = false;
            extendClamp(POSITION_CLAMP_TYPE);
            extendClamp(WOOD_SECURE_CLAMP_TYPE);
            turnYellowLedOff();
            Serial.println("Exited reload mode, ready for operation");
        }
    }
}

void handleStartSwitchSafety() {
    // Updated logic: Once switch goes OFF (LOW), it's safe to use
    // This handles both startup safety and post-NOWOOD safety
    extern bool startSwitchSafe;
    extern bool startSwitchSafeAfterNoWood;
    extern Bounce startCycleSwitch;
    
    // If switch is currently OFF and we haven't marked it safe yet, mark it safe
    if ((!startSwitchSafe || !startSwitchSafeAfterNoWood) && startCycleSwitch.read() == LOW) {
        if (!startSwitchSafe) {
            startSwitchSafe = true;
            Serial.println("Startup safety cleared - switch is OFF");
        }
        if (!startSwitchSafeAfterNoWood) {
            startSwitchSafeAfterNoWood = true;
            Serial.println("Post-NOWOOD safety cleared - switch is OFF, reload acknowledged");
        }
    }
    
    // Also handle the case where switch was ON and then turned OFF
    if ((!startSwitchSafe || !startSwitchSafeAfterNoWood) && startCycleSwitch.fell()) {
        if (!startSwitchSafe) {
            startSwitchSafe = true;
            Serial.println("Startup safety cleared - switch cycled OFF");
        }
        if (!startSwitchSafeAfterNoWood) {
            startSwitchSafeAfterNoWood = true;
            Serial.println("Post-NOWOOD safety cleared - switch cycled OFF, reload acknowledged");
        }
    }
}

void handleStartSwitchContinuousMode(){
    extern bool continuousModeActive;
    extern bool startSwitchSafe;
    extern Bounce startCycleSwitch;
    
    bool startSwitchOn = startCycleSwitch.read() == HIGH;
    if (startSwitchOn != continuousModeActive && startSwitchSafe) {
        continuousModeActive = startSwitchOn;
        if (continuousModeActive) {
            Serial.println("Continuous operation mode activated");
        } else {
            Serial.println("Continuous operation mode deactivated");
        }
    }
} 