#include "../../../include/StateMachine/StateMachine.h"

//* ************************************************************************
//* ************************ RELOAD STATE ***************************
//* ************************************************************************
//
// DESCRIPTION: 
// The RELOAD state provides a safe mode for manual wood loading/unloading:
// - Retracts all clamps for safe manual access
// - Monitors reload switch for exit condition
// - Returns to IDLE when reload switch is turned OFF
//
// STEP-BY-STEP PROCESS:
// 1. Retract all clamps for safe manual access
// 2. Set reload mode flag
// 3. Monitor reload switch continuously
// 4. When reload switch turns OFF, re-engage clamps and return to IDLE
//
//* ************************************************************************

void reload_state() {
    //! ************************************************************************
    //! STEP 1: RETRACT ALL CLAMPS FOR SAFE ACCESS
    //! ************************************************************************
    
    // TODO: Retract position clamp
    // TODO: Retract wood secure clamp
    // TODO: Set reload mode flag
    
    //! ************************************************************************
    //! STEP 2: MONITOR RELOAD SWITCH
    //! ************************************************************************
    
    // TODO: Update reload switch state
    // TODO: Check if reload switch is turned OFF (LOW)
    
    //! ************************************************************************
    //! STEP 3: EXIT RELOAD MODE
    //! ************************************************************************
    
    // TODO: When reload switch turns OFF:
    // TODO: Clear reload mode flag
    // TODO: Re-engage position clamp
    // TODO: Re-engage wood secure clamp
    // TODO: Return to IDLE state
}