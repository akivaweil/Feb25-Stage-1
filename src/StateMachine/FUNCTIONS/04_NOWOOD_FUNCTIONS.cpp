#include "StateMachine/StateMachine.h"
#include "Config/Config.h"
#include <AccelStepper.h>
#include <Bounce2.h>

// External variable declarations
extern AccelStepper cutMotor;
extern AccelStepper positionMotor;
extern SystemState currentState;

//* ************************************************************************
//* ************************ NOWOOD FUNCTIONS ****************************
//* ************************************************************************
//! NOWOOD-specific functions for handling when no wood is detected
//! These functions handle the sequence when wood is not caught by the catcher

//* ************************************************************************
//* ************************ CLAMP OPERATIONS FOR NOWOOD *****************
//* ************************************************************************

void retractSecureClampForNowood() {
    digitalWrite(WOOD_SECURE_CLAMP, HIGH);
    Serial.println("NOWOOD: Secure wood clamp retracted");
}

void resetClampPositionsForNowood() {
    // Retract position clamp
    digitalWrite(POSITION_CLAMP, HIGH);
    Serial.println("NOWOOD: Position clamp retracted");
    
    // Extend position clamp (reset to operational position)
    digitalWrite(POSITION_CLAMP, LOW);
    Serial.println("NOWOOD: Position clamp extended - reset to operational position");
}

//* ************************************************************************
//* ************************ MOTOR OPERATIONS FOR NOWOOD *****************
//* ************************************************************************

void movePositionMotorToNegOneForNowood() {
    // Move to -1 position (negative 1 step)
    positionMotor.setSpeed(POSITION_MOTOR_NORMAL_SPEED);
    positionMotor.moveTo(-1);
    Serial.println("NOWOOD: Position motor moving to -1 position");
}

void returnCutMotorToHomeForNowood() {
    cutMotor.setSpeed(CUT_MOTOR_RETURN_SPEED);
    cutMotor.moveTo(0);
    Serial.println("NOWOOD: Cut motor returning to home position");
}

void advancePositionMotorToTravelForNowood() {
    positionMotor.setSpeed(POSITION_MOTOR_NORMAL_SPEED);
    positionMotor.moveTo(POSITION_MOTOR_TRAVEL_POSITION);
    Serial.println("NOWOOD: Position motor moving to travel position");
}

//* ************************************************************************
//* ************************ STATE TRANSITION FOR NOWOOD *****************
//* ************************************************************************

void transitionFromNowoodToIdle() {
    Serial.println("NOWOOD -> IDLE: Returning to idle state - ready for next cycle");
    currentState = IDLE;
}

//* ************************************************************************
//* ************************ MAIN NOWOOD EXECUTION FUNCTION **************
//* ************************************************************************

void executeNowoodSequence() {
    static bool secureClampRetracted = false;
    static bool positionMotorToNegOne = false;
    static bool cutMotorReturnStarted = false;
    static bool clampsReset = false;
    static bool positionMotorToTravel = false;
    
    //! ************************************************************************
    //! STEP 1: RETRACT SECURE CLAMP (ONE TIME)
    //! ************************************************************************
    if (!secureClampRetracted) {
        retractSecureClampForNowood();
        secureClampRetracted = true;
    }
    
    //! ************************************************************************
    //! STEP 2: MOVE POSITION MOTOR TO -1 (ONE TIME)
    //! ************************************************************************
    if (!positionMotorToNegOne) {
        movePositionMotorToNegOneForNowood();
        positionMotorToNegOne = true;
    }
    
    //! ************************************************************************
    //! STEP 3: START CUT MOTOR RETURN (ONE TIME)
    //! ************************************************************************
    if (!cutMotorReturnStarted) {
        returnCutMotorToHomeForNowood();
        cutMotorReturnStarted = true;
    }
    
    //! ************************************************************************
    //! STEP 4: RESET CLAMPS WHEN POSITION MOTOR AT -1
    //! ************************************************************************
    if (positionMotorToNegOne && !clampsReset) {
        if (positionMotor.distanceToGo() == 0) {
            resetClampPositionsForNowood();
            clampsReset = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 5: MOVE POSITION MOTOR TO TRAVEL POSITION (ONE TIME)
    //! ************************************************************************
    if (clampsReset && !positionMotorToTravel) {
        advancePositionMotorToTravelForNowood();
        positionMotorToTravel = true;
    }
    
    //! ************************************************************************
    //! STEP 6: RUN MOTORS TO ENSURE THEY MOVE
    //! ************************************************************************
    cutMotor.run();
    positionMotor.run();
    
    //! ************************************************************************
    //! STEP 7: CHECK FOR COMPLETION AND TRANSITION TO IDLE
    //! ************************************************************************
    if (positionMotorToTravel && cutMotorReturnStarted) {
        bool cutMotorDone = (cutMotor.distanceToGo() == 0);
        bool positionMotorDone = (positionMotor.distanceToGo() == 0);
        
        if (cutMotorDone && positionMotorDone) {
            Serial.println("NOWOOD: Both motors complete - transitioning to IDLE");
            currentState = IDLE;
            
            // Reset state variables for next cycle
            positionMotorToNegOne = false;
            clampsReset = false;
            cutMotorReturnStarted = false;
            positionMotorToTravel = false;
        }
    }
} 