#include "StateMachine/StartupState.h"
#include "StateMachine/StateManager.h"
#include "Functions.h"

//* ************************************************************************
//* ************************** STARTUP STATE *******************************
//* ************************************************************************
// Handles the initial startup state, transitioning to HOMING.
// Step 1: Turn on the blue LED to indicate startup/homing.
// Step 2: Transition to the HOMING state.

void StartupState::execute(StateManager& stateManager) {
    turnBlueLedOn();  // Blue LED on during startup/homing
    stateManager.changeState(HOMING);
} 