//* ************************************************************************
//* ************************ PUSHWOODFORWARD STATE ***************************
//* ************************************************************************
//
// DESCRIPTION: 
// The PUSHWOODFORWARD state handles manually advancing wood forward:
// - Retracts position clamp and returns position motor to 0
// - Swaps clamp control to secure wood
// - Advances wood position with timing delays
// - Returns to position control for next operation
//
// STEP-BY-STEP PROCESS:
// 1. Retract position clamp
// 2. Move position motor to 0
// 3. Extend position clamp and retract secure wood clamp
// 4. Wait 300ms
// 5. Move position motor to POSITION_TRAVEL_DISTANCE - 0.1
// 6. Retract position clamp and extend secure wood clamp
// 7. Wait 50ms
// 8. Move position motor to POSITION_TRAVEL_DISTANCE
//
//* ************************************************************************

#include "../../../include/StateMachine/StateMachine.h"

void pushwoodforward_state() {
    //! ************************************************************************
    //! STEP 1: RETRACT POSITION CLAMP
    //! ************************************************************************
    
    // TODO: Retract the position clamp
    
    //! ************************************************************************
    //! STEP 2: MOVE POSITION MOTOR TO 0
    //! ************************************************************************
    
    // TODO: Move position motor to position 0
    
    //! ************************************************************************
    //! STEP 3: SWAP CLAMP CONTROL
    //! ************************************************************************
    
    // TODO: Extend position clamp
    // TODO: Retract secure wood clamp
    
    //! ************************************************************************
    //! STEP 4: WAIT 300MS
    //! ************************************************************************
    
    // TODO: Wait 300ms
    
    //! ************************************************************************
    //! STEP 5: MOVE POSITION MOTOR TO POSITION_TRAVEL_DISTANCE - 0.1
    //! ************************************************************************
    
    // TODO: Move position motor to POSITION_TRAVEL_DISTANCE - 0.1
    
    //! ************************************************************************
    //! STEP 6: SWAP CLAMP CONTROL BACK
    //! ************************************************************************
    
    // TODO: Retract position clamp
    // TODO: Extend secure wood clamp
    
    //! ************************************************************************
    //! STEP 7: WAIT 50MS
    //! ************************************************************************
    
    // TODO: Wait 50ms
    
    //! ************************************************************************
    //! STEP 8: MOVE POSITION MOTOR TO POSITION_TRAVEL_DISTANCE
    //! ************************************************************************
    
    // TODO: Move position motor to POSITION_TRAVEL_DISTANCE
}
