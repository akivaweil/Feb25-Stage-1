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
    //! ************************************************************************
    //! STEP 1: RETRACT SECURE WOOD CLAMP
    //! ************************************************************************
    
    // TODO: Retract the secure wood clamp
    
    //! ************************************************************************
    //! STEP 2: MOVE POSITION MOTOR TO -1
    //! ************************************************************************
    
    // TODO: Move position motor to -1 position
    
    //! ************************************************************************
    //! STEP 3: RETURN CUT MOTOR TO 0 (SIMULTANEOUS WITH OTHER OPERATIONS)
    //! ************************************************************************
    
    // TODO: Start returning cut motor to position 0
    
    //! ************************************************************************
    //! STEP 4: RESET CLAMP POSITIONS
    //! ************************************************************************
    
    // TODO: Retract position clamp
    // TODO: Extend position clamp
    
    //! ************************************************************************
    //! STEP 5: MOVE POSITION MOTOR TO POSITION_TRAVEL_DISTANCE
    //! ************************************************************************
    
    // TODO: Move position motor to POSITION_TRAVEL_DISTANCE
    
    //! ************************************************************************
    //! STEP 6: RETURN TO IDLE STATE
    //! ************************************************************************
    
    // TODO: Transition back to IDLE state
}
