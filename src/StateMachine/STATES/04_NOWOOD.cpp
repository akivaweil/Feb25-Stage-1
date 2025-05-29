//* ************************************************************************
//* ************************ NOWOOD STATE ***************************
//* ************************************************************************
//
// DESCRIPTION: 
// The NOWOOD state handles the sequence when no wood is detected after cutting:
// - Resets clamp positions and motors for next cycle
// - Returns cut motor to home position
// - Prepares machine for next operation
//
// STEP-BY-STEP PROCESS:
// 1. Retract secure wood clamp
// 2. Move position motor to -1 position
// 3. Return cut motor to position 0 (simultaneous with other operations)
// 4. Reset clamp positions (retract position, extend position)
// 5. Move position motor to POSITION_TRAVEL_DISTANCE
// 6. Return to IDLE state
//
//* ************************************************************************

#include "../../../include/StateMachine/StateMachine.h"

void nowood_state() {
    executeNowoodSequence();
}
