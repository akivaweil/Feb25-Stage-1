#include "StateMachine/StateMachine.h"
#include <Bounce2.h>
#include <Arduino.h>
#include "OTA_Manager.h"

//* ************************************************************************
//* ************************ IDLE STATE ***********************************
//* ************************************************************************
//! IDLE state implementation
//! System waits in ready state, monitors switches, and handles OTA updates
//! This is the main waiting state where the system is ready for operation

// External variable declarations
extern SystemState currentState;
extern bool stateChanged;
extern Bounce startCycleSwitch;
extern Bounce reloadSwitch;

void executeIDLE() {
    static bool entryExecuted = false;
    static unsigned long lastStatusMessage = 0;
    
    //! ************************************************************************
    //! STEP 1: EXECUTE ENTRY ACTIONS (ONLY ONCE WHEN ENTERING STATE)
    //! ************************************************************************
    if (stateChanged && !entryExecuted) {
        Serial.println("=== ENTERING IDLE STATE ===");
        Serial.println("System ready for operations");
        Serial.println("Monitoring: Start Cycle Switch | Reload Switch | OTA Updates");
        
        // Turn on green LED to indicate ready state
        turnGreenLedOn();
        
        // Set entry flags
        entryExecuted = true;
        stateChanged = false;
        
        Serial.println("IDLE state entry complete - system ready and monitoring");
    }
    
    //! ************************************************************************
    //! STEP 2: RESET ENTRY FLAG WHEN LEAVING STATE
    //! ************************************************************************
    if (currentState != IDLE) {
        entryExecuted = false;
        return;
    }
    
    //! ************************************************************************
    //! STEP 3: UPDATE SWITCH STATES
    //! ************************************************************************
    startCycleSwitch.update();
    reloadSwitch.update();
    
    //! ************************************************************************
    //! STEP 4: CHECK FOR OTA UPDATES
    //! ************************************************************************
    handleOTA();
    
    //! ************************************************************************
    //! STEP 5: MONITOR START CYCLE SWITCH
    //! ************************************************************************
    if (startCycleSwitch.rose() == HIGH) {
        Serial.println("Start cycle switch detected HIGH - transitioning to CUTTING state");
        changeState(CUTTING);
        return;
    }
    
    //! ************************************************************************
    //! STEP 6: MONITOR RELOAD SWITCH
    //! ************************************************************************
    if (reloadSwitch.read() == HIGH) {
        Serial.println("Reload switch detected HIGH - transitioning to RELOAD state");
        changeState(RELOAD);
        return;
    }
    
    //! ************************************************************************
    //! STEP 7: PERIODIC STATUS MESSAGES
    //! ************************************************************************
    if (millis() - lastStatusMessage > 10000) {  // Print every 10 seconds
        Serial.println("IDLE state active - System ready | Monitoring switches and OTA");
        lastStatusMessage = millis();
    }
} 