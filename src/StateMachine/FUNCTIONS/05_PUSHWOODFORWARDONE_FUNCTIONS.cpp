#include "StateMachine/StateMachine.h"
#include "Config/Config.h"
#include <AccelStepper.h>

// External variable declarations
extern AccelStepper positionMotor;
extern SystemState currentState;

//* ************************************************************************
//* ************************ PUSHWOODFORWARDONE FUNCTIONS ****************
//* ************************************************************************
//! PUSHWOODFORWARDONE-specific functions for manual wood advancement
//! These functions handle the precise sequence for manually pushing wood forward

//* ************************************************************************
//* ************************ CLAMP OPERATIONS FOR PUSHWOODFORWARDONE *****
//* ************************************************************************

void retractPositionClampForPushWood() {
    digitalWrite(POSITION_CLAMP, HIGH);
    Serial.println("PUSHWOOD: Position clamp retracted");
}

void swapToSecureControlForPushWood() {
    // Extend position clamp
    digitalWrite(POSITION_CLAMP, LOW);
    Serial.println("PUSHWOOD: Position clamp extended for wood control");
    
    // Retract secure wood clamp
    digitalWrite(WOOD_SECURE_CLAMP, HIGH);
    Serial.println("PUSHWOOD: Secure wood clamp retracted - position clamp taking control");
}

void swapToPositionControlForPushWood() {
    // Retract position clamp
    digitalWrite(POSITION_CLAMP, HIGH);
    Serial.println("PUSHWOOD: Position clamp retracted");
    
    // Extend secure wood clamp
    digitalWrite(WOOD_SECURE_CLAMP, LOW);
    Serial.println("PUSHWOOD: Secure wood clamp extended - securing wood for final positioning");
}

//* ************************************************************************
//* ************************ MOTOR OPERATIONS FOR PUSHWOODFORWARDONE *****
//* ************************************************************************

void movePositionMotorToHomeForPushWood() {
    positionMotor.setSpeed(POSITION_MOTOR_NORMAL_SPEED);
    positionMotor.moveTo(0);
    Serial.println("PUSHWOOD: Position motor moving to home position (0)");
}

void advancePositionMotorForPushWood() {
    // Move to POSITION_TRAVEL_DISTANCE - 0.1 inches
    float targetPosition = (POSITION_TRAVEL_DISTANCE - 0.1) * POSITION_MOTOR_STEPS_PER_INCH;
    positionMotor.setSpeed(POSITION_MOTOR_NORMAL_SPEED);
    positionMotor.moveTo(targetPosition);
    Serial.print("PUSHWOOD: Position motor moving to advance position: ");
    Serial.println(targetPosition);
}

void movePositionMotorToFinalForPushWood() {
    positionMotor.setSpeed(POSITION_MOTOR_NORMAL_SPEED);
    positionMotor.moveTo(POSITION_MOTOR_TRAVEL_POSITION);
    Serial.println("PUSHWOOD: Position motor moving to final travel position");
}

//* ************************************************************************
//* ************************ TIMING FUNCTIONS FOR PUSHWOODFORWARDONE *****
//* ************************************************************************

void waitForPushWoodSwapDelay() {
    delay(300);
    Serial.println("PUSHWOOD: 300ms delay completed");
}

void waitForPushWoodFinalDelay() {
    delay(50);
    Serial.println("PUSHWOOD: 50ms delay completed");
}

//* ************************************************************************
//* ************************ STATE TRANSITION FOR PUSHWOODFORWARDONE *****
//* ************************************************************************

void transitionFromPushWoodToIdle() {
    Serial.println("PUSHWOODFORWARDONE -> IDLE: Wood advancement complete - returning to idle");
    currentState = IDLE;
}

//* ************************************************************************
//* ************************ MAIN PUSHWOODFORWARDONE EXECUTION FUNCTION **
//* ************************************************************************

void executePushWoodForwardSequence() {
    static bool positionClampRetracted = false;
    static bool positionMotorToHome = false;
    static bool clampsSwappedToSecure = false;
    static bool swapDelayCompleted = false;
    static bool positionMotorAdvanced = false;
    static bool clampsSwappedToPosition = false;
    static bool finalDelayCompleted = false;
    static bool positionMotorToFinal = false;
    
    //! ************************************************************************
    //! STEP 1: RETRACT POSITION CLAMP (ONE TIME)
    //! ************************************************************************
    if (!positionClampRetracted) {
        retractPositionClampForPushWood();
        positionClampRetracted = true;
    }
    
    //! ************************************************************************
    //! STEP 2: MOVE POSITION MOTOR TO HOME (ONE TIME)
    //! ************************************************************************
    if (positionClampRetracted && !positionMotorToHome) {
        movePositionMotorToHomeForPushWood();
        positionMotorToHome = true;
    }
    
    //! ************************************************************************
    //! STEP 3: WAIT FOR HOME, THEN SWAP TO SECURE CONTROL (ONE TIME)
    //! ************************************************************************
    if (positionMotorToHome && !clampsSwappedToSecure) {
        if (positionMotor.distanceToGo() == 0) {
            swapToSecureControlForPushWood();
            clampsSwappedToSecure = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 4: WAIT 300MS (ONE TIME)
    //! ************************************************************************
    if (clampsSwappedToSecure && !swapDelayCompleted) {
        waitForPushWoodSwapDelay();
        swapDelayCompleted = true;
    }
    
    //! ************************************************************************
    //! STEP 5: MOVE POSITION MOTOR TO ADVANCE POSITION (ONE TIME)
    //! ************************************************************************
    if (swapDelayCompleted && !positionMotorAdvanced) {
        advancePositionMotorForPushWood();
        positionMotorAdvanced = true;
    }
    
    //! ************************************************************************
    //! STEP 6: WAIT FOR ADVANCE, THEN SWAP TO POSITION CONTROL (ONE TIME)
    //! ************************************************************************
    if (positionMotorAdvanced && !clampsSwappedToPosition) {
        if (positionMotor.distanceToGo() == 0) {
            swapToPositionControlForPushWood();
            clampsSwappedToPosition = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 7: WAIT 50MS (ONE TIME)
    //! ************************************************************************
    if (clampsSwappedToPosition && !finalDelayCompleted) {
        waitForPushWoodFinalDelay();
        finalDelayCompleted = true;
    }
    
    //! ************************************************************************
    //! STEP 8: MOVE TO FINAL TRAVEL POSITION (ONE TIME)
    //! ************************************************************************
    if (finalDelayCompleted && !positionMotorToFinal) {
        movePositionMotorToFinalForPushWood();
        positionMotorToFinal = true;
    }
    
    //! ************************************************************************
    //! STEP 8.5: RUN MOTOR TO ENSURE IT MOVES
    //! ************************************************************************
    positionMotor.run();
    
    //! ************************************************************************
    //! STEP 9: WAIT FOR COMPLETION AND TRANSITION TO IDLE
    //! ************************************************************************
    if (positionMotorToFinal) {
        if (positionMotor.distanceToGo() == 0) {
            Serial.println("PUSHWOOD: Position motor at final position - transitioning to IDLE");
            currentState = IDLE;
            
            // Reset state variables for next cycle
            positionClampRetracted = false;
            positionMotorToHome = false;
            clampsSwappedToSecure = false;
            swapDelayCompleted = false;
            positionMotorAdvanced = false;
            clampsSwappedToPosition = false;
            finalDelayCompleted = false;
            positionMotorToFinal = false;
        }
    }
} 