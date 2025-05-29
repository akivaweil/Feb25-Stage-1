#include "../../../include/StateMachine/StateMachine.h"
#include "Config/Config.h"
#include <FastAccelStepper.h>
#include <Bounce2.h>

// External variable declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;
extern StateType currentState;

//* ************************************************************************
//* ************************ NOWOOD FUNCTIONS ***************************
//* ************************************************************************
//! NOWOOD-specific functions for handling when no wood is detected
//! These functions reset the machine state and prepare for the next cycle

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

void movePositionMotorToNegativeOneForNowood() {
    if (!positionMotor) {
        Serial.println("ERROR: Position motor not initialized");
        return;
    }
    
    // Move to -1 position (negative 1 step)
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_NORMAL_SPEED);
    positionMotor->moveTo(-1);
    Serial.println("NOWOOD: Position motor moving to -1 position");
}

void startCutMotorReturnForNowood() {
    if (!cutMotor) {
        Serial.println("ERROR: Cut motor not initialized");
        return;
    }
    
    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_RETURN_SPEED);
    cutMotor->moveTo(0);
    Serial.println("NOWOOD: Cut motor returning to home position");
}

void movePositionMotorToTravelForNowood() {
    if (!positionMotor) {
        Serial.println("ERROR: Position motor not initialized");
        return;
    }
    
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_NORMAL_SPEED);
    positionMotor->moveTo(POSITION_MOTOR_TRAVEL_POSITION);
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
        movePositionMotorToNegativeOneForNowood();
        positionMotorToNegOne = true;
    }
    
    //! ************************************************************************
    //! STEP 3: START CUT MOTOR RETURN (ONE TIME)
    //! ************************************************************************
    if (!cutMotorReturnStarted) {
        startCutMotorReturnForNowood();
        cutMotorReturnStarted = true;
    }
    
    //! ************************************************************************
    //! STEP 4: WAIT FOR POSITION MOTOR TO REACH -1, THEN RESET CLAMPS
    //! ************************************************************************
    if (positionMotorToNegOne && !clampsReset) {
        if (positionMotor && !positionMotor->isRunning()) {
            resetClampPositionsForNowood();
            clampsReset = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 5: MOVE POSITION MOTOR TO TRAVEL POSITION (ONE TIME)
    //! ************************************************************************
    if (clampsReset && !positionMotorToTravel) {
        movePositionMotorToTravelForNowood();
        positionMotorToTravel = true;
    }
    
    //! ************************************************************************
    //! STEP 6: WAIT FOR ALL MOVEMENTS TO COMPLETE, THEN TRANSITION TO IDLE
    //! ************************************************************************
    if (positionMotorToTravel && cutMotorReturnStarted) {
        bool cutMotorDone = (cutMotor && !cutMotor->isRunning());
        bool positionMotorDone = (positionMotor && !positionMotor->isRunning());
        
        if (cutMotorDone && positionMotorDone) {
            transitionFromNowoodToIdle();
            
            // Reset state variables for next cycle
            secureClampRetracted = false;
            positionMotorToNegOne = false;
            cutMotorReturnStarted = false;
            clampsReset = false;
            positionMotorToTravel = false;
        }
    }
} 