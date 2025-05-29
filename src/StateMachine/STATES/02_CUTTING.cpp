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
    //! ************************************************************************
    //! STEP 1: EXTEND BOTH CLAMPS
    //! ************************************************************************
    
    // TODO: Extend both clamps simultaneously
    
    //! ************************************************************************
    //! STEP 2: MOVE CUT MOTOR WITH SAFETY CHECK
    //! ************************************************************************
    
    // TODO: Start moving cut motor to cut_travel_distance
    // TODO: When 0.3 inches in, check waswoodsuctioned sensor
    // TODO: If sensor reads LOW, stop and enter waswoodsuctioned error state
    
    //! ************************************************************************
    //! STEP 3: ACTIVATE CATCHER MECHANISMS
    //! ************************************************************************
    
    // TODO: When cut motor reaches CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES from end
    // TODO: Activate catcher clamp function
    // TODO: When cut motor reaches CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES from end  
    // TODO: Activate catcher servo function
    
    //! ************************************************************************
    //! STEP 4: CHECK WOOD SENSOR AND ROUTE
    //! ************************************************************************
    
    // TODO: Check wood sensor pin
    // TODO: If sensor reads LOW -> transition to YESWOOD state
    // TODO: If sensor reads HIGH -> transition to NOWOOD state
}
