#include "StateMachine/StateMachine.h"
#include <FastAccelStepper.h>
#include <Bounce2.h>

//* ************************************************************************
//* ************************ NOWOOD STATE *********************************
//* ************************************************************************
//! NOWOOD state implementation - Processing when no wood is detected
//! 
//! Step-by-step sequence:
//! 1. Retract the secure wood clamp
//! 2. Move the position motor to -1
//! 3. Return the cut motor to position 0 (this should occur simultaneously with everything else)
//! 4. Retract the position clamp and extend the position clamp
//! 5. Move the position motor to POSITION_TRAVEL_DISTANCE
//! 6. Return to idle state

// External variable declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;

// State variables
static bool nowoodStateEntered = false;
static bool secureClampRetracted = false;
static bool positionMotorToMinusOne = false;
static bool cutMotorReturnStarted = false;
static bool clampOperationsComplete = false;
static bool finalPositionMove = false;

void executeNOWOOD() {
    //! ************************************************************************
    //! STEP 1: RETRACT THE SECURE WOOD CLAMP
    //! ************************************************************************
    if (!nowoodStateEntered) {
        Serial.println("=== NOWOOD STATE ENTERED ===");
        nowoodStateEntered = true;
        
        // Set safety flag to require switch cycling before next cycle
        extern bool startSwitchSafeAfterNoWood;
        startSwitchSafeAfterNoWood = false;
        Serial.println("Safety flag set - Start switch must be cycled OFF->ON before next cycle");
        
        // Reset all state variables
        secureClampRetracted = false;
        positionMotorToMinusOne = false;
        cutMotorReturnStarted = false;
        clampOperationsComplete = false;
        finalPositionMove = false;
        
        Serial.println("!1. Retracting the secure wood clamp");
        retractClamp(WOOD_SECURE_CLAMP_TYPE);
        secureClampRetracted = true;
    }
    
    //! ************************************************************************
    //! STEP 2: MOVE THE POSITION MOTOR TO -1
    //! ************************************************************************
    if (secureClampRetracted && !positionMotorToMinusOne) {
        Serial.println("!2. Moving position motor to -1");
        if (positionMotor) {
            long targetSteps = (long)(-1.0f * STEPS_PER_INCH_POSITION);
            moveMotorTo(POSITION_MOTOR, targetSteps, POSITION_MOTOR_NORMAL_SPEED);
        }
        positionMotorToMinusOne = true;
    }
    
    //! ************************************************************************
    //! STEP 3: RETURN THE CUT MOTOR TO POSITION 0 (SIMULTANEOUS)
    //! ************************************************************************
    if (positionMotorToMinusOne && !cutMotorReturnStarted) {
        Serial.println("!3. Returning cut motor to position 0");
        if (cutMotor) {
            moveMotorTo(CUT_MOTOR, 0, CUT_MOTOR_RETURN_SPEED);
            cutMotorReturnStarted = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 4: RETRACT POSITION CLAMP AND EXTEND POSITION CLAMP
    //! ************************************************************************
    if (positionMotorToMinusOne && !clampOperationsComplete) {
        if (positionMotor && !positionMotor->isRunning()) {
            Serial.println("!4. Retracting position clamp and extending position clamp");
            retractClamp(POSITION_CLAMP_TYPE);
            extendClamp(POSITION_CLAMP_TYPE);
            clampOperationsComplete = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 5: MOVE THE POSITION MOTOR TO POSITION_TRAVEL_DISTANCE
    //! ************************************************************************
    if (clampOperationsComplete && !finalPositionMove) {
        Serial.println("!5. Moving position motor to POSITION_TRAVEL_DISTANCE");
        if (positionMotor) {
            moveMotorTo(POSITION_MOTOR, POSITION_MOTOR_TRAVEL_POSITION, POSITION_MOTOR_NORMAL_SPEED);
            finalPositionMove = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 6: RETURN TO IDLE STATE
    //! ************************************************************************
    if (finalPositionMove) {
        if (positionMotor && !positionMotor->isRunning()) {
            Serial.println("!6. Position motor movement complete - returning to IDLE state");
            changeState(IDLE);
            
            // Reset state variables for next entry
            nowoodStateEntered = false;
        }
    }
} 