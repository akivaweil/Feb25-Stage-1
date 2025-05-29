//* ************************************************************************
//* ************************ YESWOOD STATE ***************************
//* ************************************************************************
//
// DESCRIPTION: 
// The YESWOOD state handles the sequence when wood is successfully cut:
// - Returns cut motor to home while managing clamps
// - Advances wood position for next cut
// - Performs safety checks and prepares for next cycle
//
// STEP-BY-STEP PROCESS:
// 1. Return cut motor to position 0 (simultaneous with other operations)
// 2. Retract secure wood clamp
// 3. Move position motor to POSITION_TRAVEL_DISTANCE - 0.1
// 4. Swap clamp positions (extend secure, retract position)
// 5. Return position motor to 0 and extend position clamp when home
// 6. Wait for cut motor home and verify with sensor check
// 7. Advance position motor to POSITION_TRAVEL_DISTANCE  
// 8. Check run cycle switch and route accordingly
//
//* ************************************************************************

#include "../../../include/StateMachine/StateMachine.h"

void yeswood_state() {
    executeYeswoodSequence();
}
