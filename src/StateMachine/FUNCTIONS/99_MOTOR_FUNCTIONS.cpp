#include "StateMachine/StateMachine.h"

//* ************************************************************************
//* ************************ MOTOR FUNCTIONS ***************************
//* ************************************************************************
//! Motor control functions for steppers only
//! ONLY ABSOLUTELY NECESSARY FUNCTIONS - All redundant wrappers removed
//! Use moveMotorTo() directly with appropriate parameters

// External variable declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;

//* ************************************************************************
//* ************************ CORE MOTOR FUNCTIONS ************************
//* ************************************************************************
//! These are the only motor movement functions you should use

void moveMotorTo(MotorType motor, float position, float speed) {
    switch(motor) {
        case CUT_MOTOR:
            if (cutMotor) {
                cutMotor->setSpeedInHz(speed);
                cutMotor->moveTo(position);
                Serial.print("Cut motor moving to position: ");
                Serial.print(position);
                Serial.print(" at speed: ");
                Serial.println(speed);
            }
            break;
        case POSITION_MOTOR:
            if (positionMotor) {
                positionMotor->setSpeedInHz(speed);
                positionMotor->moveTo(position);
                Serial.print("Position motor moving to position: ");
                Serial.print(position);
                Serial.print(" at speed: ");
                Serial.println(speed);
            }
            break;
        default:
            Serial.println("ERROR: Unknown motor type for moveMotorTo operation");
            break;
    }
}

void stopCutMotor() {
    if (cutMotor) {
        cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
        Serial.println("Cut motor stopped");
    }
}

void stopPositionMotor() {
    if (positionMotor) {
        positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());
        Serial.println("Position motor stopped");
    }
}

//* ************************************************************************
//* ************************ SPECIALIZED MOTOR FUNCTIONS *****************
//* ************************************************************************
//! Functions with unique logic that cannot be replaced by moveMotorTo()

void movePositionMotorToTravelWithEarlyActivation() {
    if (positionMotor) {
        positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_NORMAL_SPEED);
        positionMotor->moveTo(POSITION_MOTOR_TRAVEL_POSITION);
        Serial.println("Position motor moving to travel position");
        while(positionMotor->isRunning()){
            // Wait for movement completion - no early activation during position moves
        }
    }
}

void movePositionMotorToInitialAfterHoming() {
    if (positionMotor) {
        positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_NORMAL_SPEED);
        positionMotor->moveTo(0);
        Serial.println("Position motor moving to initial position after homing");
        while(positionMotor->isRunning()){
            // Wait for movement completion - no early activation during position moves
        }
    }
} 