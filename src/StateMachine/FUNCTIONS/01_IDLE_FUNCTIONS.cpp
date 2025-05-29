#include "StateMachine/StateMachine.h"
#include "Config/Config.h"
#include "OTA_Manager.h"
#include <Bounce2.h>

// External variable declarations
extern Bounce startCycleSwitch;
extern Bounce reloadSwitch;
extern SystemState currentState;

//* ************************************************************************
//* ************************ IDLE FUNCTIONS ***************************
//* ************************************************************************
//! IDLE-specific functions for system monitoring and state transitions
//! These functions are specifically designed for the IDLE state operations
//! and provide clear, readable implementations for each IDLE task

//* ************************************************************************
//* ************************ OTA AND SYSTEM MONITORING *******************
//* ************************************************************************

void handleOTAInIdle() {
    handleOTA(); // Handle any OTA updates while idle
}

//* ************************************************************************
//* ************************ SWITCH MONITORING FUNCTIONS *****************
//* ************************************************************************

bool checkStartCycleSwitchInIdle() {
    startCycleSwitch.update();
    if (startCycleSwitch.read() == HIGH) {
        Serial.println("IDLE: Start cycle switch activated - transitioning to CUTTING");
        return true;
    }
    return false;
}

bool checkReloadSwitchInIdle() {
    reloadSwitch.update();
    if (reloadSwitch.read() == HIGH) {
        Serial.println("IDLE: Reload switch activated - transitioning to RELOAD");
        return true;
    }
    return false;
}

//* ************************************************************************
//* ************************ STATE TRANSITION FUNCTIONS ******************
//* ************************************************************************

void transitionFromIdleToCutting() {
    Serial.println("IDLE -> CUTTING: Starting cutting sequence");
    currentState = CUTTING;
}

void transitionFromIdleToReload() {
    Serial.println("IDLE -> RELOAD: Starting reload sequence");
    currentState = RELOAD;
}

//* ************************************************************************
//* ************************ MAIN IDLE MONITORING FUNCTION ***************
//* ************************************************************************

void executeIdleMonitoring() {
    // Handle OTA updates
    handleOTAInIdle();
    
    // Check start cycle switch
    if (checkStartCycleSwitchInIdle()) {
        transitionFromIdleToCutting();
        return;
    }
    
    // Check reload switch
    if (checkReloadSwitchInIdle()) {
        transitionFromIdleToReload();
        return;
    }
    
    // Continue monitoring (no action needed - remain in IDLE)
}

// No functions currently needed - all operations use core functions from 99_ files 