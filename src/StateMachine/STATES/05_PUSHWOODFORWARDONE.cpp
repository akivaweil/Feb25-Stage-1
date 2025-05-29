#include "StateMachine/StateMachine.h"
#include <FastAccelStepper.h>
#include <Bounce2.h>

//* ************************************************************************
//* ************************ PUSHWOODFORWARD STATE ***********************
//* ************************************************************************
//! PUSHWOODFORWARD state implementation - Push wood forward sequence
//! 
//! Step-by-step sequence:
//! 1. Retract the position clamp
//! 2. Move the position motor to 0
//! 3. Extend the position clamp and retract the secure wood clamp
//! 4. Wait 300ms 
//! 5. Move the position motor to POSITION_TRAVEL_DISTANCE - 0.1
//! 6. Retract the position clamp and extend the wood secure clamp
//! 7. Wait 50ms
//! 8. Move the position motor to POSITION_TRAVEL_DISTANCE

// External variable declarations
extern FastAccelStepper *positionMotor;

// State variables
static bool pushwoodStateEntered = false;
static bool positionClampRetracted = false;
static bool positionMotorToZero = false;
static bool clampOperationsComplete = false;
static bool firstWaitComplete = false;
static bool positionMotorToTravelMinus = false;
static bool secondClampOperationsComplete = false;
static bool secondWaitComplete = false;
static unsigned long firstWaitStartTime = 0;
static unsigned long secondWaitStartTime = 0;

void executePUSHWOODFORWARDONE() {
    //! ************************************************************************
    //! STEP 1: RETRACT THE POSITION CLAMP
    //! ************************************************************************
    if (!pushwoodStateEntered) {
        Serial.println("=== PUSHWOODFORWARD STATE ENTERED ===");
        pushwoodStateEntered = true;
        
        // Reset all state variables
        positionClampRetracted = false;
        positionMotorToZero = false;
        clampOperationsComplete = false;
        firstWaitComplete = false;
        positionMotorToTravelMinus = false;
        secondClampOperationsComplete = false;
        secondWaitComplete = false;
        firstWaitStartTime = 0;
        secondWaitStartTime = 0;
        
        Serial.println("!1. Retracting the position clamp");
        retractClamp(POSITION_CLAMP_TYPE);
        positionClampRetracted = true;
    }
    
    //! ************************************************************************
    //! STEP 2: MOVE THE POSITION MOTOR TO 0
    //! ************************************************************************
    if (positionClampRetracted && !positionMotorToZero) {
        Serial.println("!2. Moving position motor to 0");
        if (positionMotor) {
            moveMotorTo(POSITION_MOTOR, 0, POSITION_MOTOR_NORMAL_SPEED);
        }
        positionMotorToZero = true;
    }
    
    //! ************************************************************************
    //! STEP 3: EXTEND POSITION CLAMP AND RETRACT SECURE WOOD CLAMP
    //! ************************************************************************
    if (positionMotorToZero && !clampOperationsComplete) {
        if (positionMotor && !positionMotor->isRunning()) {
            Serial.println("!3. Extending position clamp and retracting secure wood clamp");
            extendClamp(POSITION_CLAMP_TYPE);
            retractClamp(WOOD_SECURE_CLAMP_TYPE);
            clampOperationsComplete = true;
            firstWaitStartTime = millis();
        }
    }
    
    //! ************************************************************************
    //! STEP 4: WAIT 300MS
    //! ************************************************************************
    if (clampOperationsComplete && !firstWaitComplete) {
        if (millis() - firstWaitStartTime >= 300) {
            Serial.println("!4. 300ms wait complete");
            firstWaitComplete = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 5: MOVE POSITION MOTOR TO POSITION_TRAVEL_DISTANCE - 0.1
    //! ************************************************************************
    if (firstWaitComplete && !positionMotorToTravelMinus) {
        float targetPosition = POSITION_TRAVEL_DISTANCE - 0.1f;
        Serial.print("!5. Moving position motor to ");
        Serial.print(targetPosition);
        Serial.println(" inches");
        
        if (positionMotor) {
            long targetSteps = (long)(targetPosition * STEPS_PER_INCH_POSITION);
            moveMotorTo(POSITION_MOTOR, targetSteps, POSITION_MOTOR_NORMAL_SPEED);
        }
        positionMotorToTravelMinus = true;
    }
    
    //! ************************************************************************
    //! STEP 6: RETRACT POSITION CLAMP AND EXTEND WOOD SECURE CLAMP
    //! ************************************************************************
    if (positionMotorToTravelMinus && !secondClampOperationsComplete) {
        if (positionMotor && !positionMotor->isRunning()) {
            Serial.println("!6. Retracting position clamp and extending wood secure clamp");
            retractClamp(POSITION_CLAMP_TYPE);
            extendClamp(WOOD_SECURE_CLAMP_TYPE);
            secondClampOperationsComplete = true;
            secondWaitStartTime = millis();
        }
    }
    
    //! ************************************************************************
    //! STEP 7: WAIT 50MS
    //! ************************************************************************
    if (secondClampOperationsComplete && !secondWaitComplete) {
        if (millis() - secondWaitStartTime >= 50) {
            Serial.println("!7. 50ms wait complete");
            secondWaitComplete = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 8: MOVE POSITION MOTOR TO POSITION_TRAVEL_DISTANCE
    //! ************************************************************************
    if (secondWaitComplete) {
        Serial.println("!8. Moving position motor to POSITION_TRAVEL_DISTANCE");
        if (positionMotor) {
            moveMotorTo(POSITION_MOTOR, POSITION_MOTOR_TRAVEL_POSITION, POSITION_MOTOR_NORMAL_SPEED);
        }
        
        // Reset state variables for next entry
        pushwoodStateEntered = false;
        
        // Note: This state doesn't automatically transition - it completes the sequence
        Serial.println("PUSHWOODFORWARD sequence complete");
    }
} 