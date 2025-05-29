//* ************************************************************************
//* ************************ CUTTING STATE ***************************
//* ************************************************************************
//
// DESCRIPTION: 
// The CUTTING state handles the main cutting operation sequence:
// - Extends both clamps to secure wood
// - Moves cut motor through cutting motion with safety checks
// - Activates catcher mechanisms at specified distances
// - Routes to YESWOOD or NOWOOD states based on wood sensor
//
// STEP-BY-STEP PROCESS:
// 1. Extend both clamps simultaneously
// 2. Move cut motor to cut_travel_distance with safety check at 0.3"
// 3. Activate catcher clamp and servo when reaching early activation offsets
// 4. Check wood sensor and route to YESWOOD (sensor LOW) or NOWOOD (sensor HIGH)
//
//* ************************************************************************

#include "../../../include/StateMachine/StateMachine.h"

void cutting_state() {
    executeCuttingSequence();
}
