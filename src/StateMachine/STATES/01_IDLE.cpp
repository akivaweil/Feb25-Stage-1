//* ************************************************************************
//* ************************ IDLE STATE ***************************
//* ************************************************************************
//
// DESCRIPTION: 
// The IDLE state is the default waiting state where the machine checks for:
// - OTA uploads
// - Start cycle switch activation (HIGH) to enter CUTTING state
// - Reload switch activation (HIGH) to enter RELOAD state
//
// STEP-BY-STEP PROCESS:
// 1. Check for OTA uploads and handle if needed
// 2. Check if start cycle switch is HIGH -> go to CUTTING state
// 3. Check if reload switch is HIGH -> go to RELOAD state
// 4. Continue monitoring in loop
//
//* ************************************************************************

#include "../../../include/StateMachine/StateMachine.h"

void idle_state() {
    //! ************************************************************************
    //! STEP 1: CHECK FOR OTA UPLOADS
    //! ************************************************************************
    
    // TODO: Add OTA upload check functionality
    
    //! ************************************************************************
    //! STEP 2: CHECK START CYCLE SWITCH
    //! ************************************************************************
    
    // TODO: Check if start cycle switch is HIGH
    // TODO: If HIGH, transition to CUTTING state
    
    //! ************************************************************************
    //! STEP 3: CHECK RELOAD SWITCH
    //! ************************************************************************
    
    // TODO: Check if reload switch is HIGH  
    // TODO: If HIGH, transition to RELOAD state
    
    //! ************************************************************************
    //! STEP 4: CONTINUE MONITORING LOOP
    //! ************************************************************************
    
    // TODO: Continue monitoring switches in main loop
}
