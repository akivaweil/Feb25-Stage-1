#include "StateMachine/PositioningState.h"
#include "StateMachine/StateManager.h"
#include "Functions.h"

//* ************************************************************************
//* ************************ POSITIONING STATE *****************************
//* ************************************************************************
// Handles the motor positioning sequence (largely integrated into CUTTING).
// This state is now mostly a placeholder as the positioning logic is integrated directly
// within the CUTTING state's stages.
// Step 1: Transition to IDLE state.
// Step 2: Turn off yellow LED.
// Step 3: Reset cuttingCycleInProgress flag.

void PositioningState::execute(StateManager& stateManager) {
    // This function is no longer needed as the positioning operation is now handled
    // directly in the CuttingState (formerly inlined in case CUTTING)
    
    // Transition to IDLE state
stateManager.changeState(IDLE);
    turnYellowLedOff();
    stateManager.setCuttingCycleInProgress(false);
} 