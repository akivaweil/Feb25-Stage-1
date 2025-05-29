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
    //! ************************************************************************
    //! STEP 1: RETURN CUT MOTOR TO 0 (SIMULTANEOUS WITH OTHER OPERATIONS)
    //! ************************************************************************
    
    // TODO: Start returning cut motor to position 0
    
    //! ************************************************************************
    //! STEP 2: RETRACT SECURE WOOD CLAMP
    //! ************************************************************************
    
    // TODO: Retract the secure wood clamp
    
    //! ************************************************************************
    //! STEP 3: MOVE POSITION MOTOR TO POSITION_TRAVEL_DISTANCE - 0.1
    //! ************************************************************************
    
    // TODO: Move position motor to POSITION_TRAVEL_DISTANCE - 0.1
    
    //! ************************************************************************
    //! STEP 4: SWAP CLAMP POSITIONS
    //! ************************************************************************
    
    // TODO: Extend secure wood clamp
    // TODO: Retract position clamp
    
    //! ************************************************************************
    //! STEP 5: RETURN POSITION MOTOR TO 0
    //! ************************************************************************
    
    // TODO: Move position motor to position 0
    // TODO: When position motor reaches 0, extend position clamp
    
    //! ************************************************************************
    //! STEP 6: WAIT FOR CUT MOTOR HOME AND VERIFY
    //! ************************************************************************
    
    // TODO: Wait until cut motor is home
    // TODO: 10ms after cut motor reports position 0, check homing sensor
    // TODO: If homing sensor reads LOW, enter cutmotorfailedtohome error state
    
    //! ************************************************************************
    //! STEP 7: FINAL HOME CHECK AND ADVANCE
    //! ************************************************************************
    
    // TODO: Inside if statement: move position motor to POSITION_TRAVEL_DISTANCE
    
    //! ************************************************************************
    //! STEP 8: CHECK RUN CYCLE SWITCH AND ROUTE
    //! ************************************************************************
    
    // TODO: Check if run cycle switch is HIGH
    // TODO: If HIGH -> transition to CUTTING state
    // TODO: If not HIGH -> return to IDLE state
}
