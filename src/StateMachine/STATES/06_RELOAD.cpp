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
    executeReloadSequence();
}