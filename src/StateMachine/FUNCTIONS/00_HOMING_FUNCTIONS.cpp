#include "StateMachine/StateMachine.h"
#include <FastAccelStepper.h>
#include <Bounce2.h>

//* ************************************************************************
//* ************************ HOMING FUNCTIONS *****************************
//* ************************************************************************
//! Homing functions for cut motor and position motor
//! Cut motor: Home in negative direction, stop on switch, set position to 0
//! Position motor: Home in positive direction, stop on switch, set position to (2 + POSITION_TRAVEL_DISTANCE), then move to POSITION_TRAVEL_DISTANCE

// External variable declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;
extern Bounce cutHomingSwitch;
extern Bounce positionHomingSwitch;
extern bool isHomed;

// Homing state variables
bool cutMotorHomed = false;
bool positionMotorHomed = false;
bool positionMotorMovedToTravel = false;

//* ************************************************************************
//* ************************ CUT MOTOR HOMING ****************************
//* ************************************************************************
void homeCutMotor() {
    if (!cutMotor) {
        Serial.println("ERROR: Cut motor not initialized");
        return;
    }
    
    Serial.println("Starting cut motor homing sequence");
    
    // Configure motor for homing
    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_HOMING_SPEED);
    cutMotor->setAcceleration((uint32_t)CUT_MOTOR_NORMAL_ACCELERATION);
    
    // Move in negative direction (CUT_HOMING_DIRECTION = -1)
    if (CUT_HOMING_DIRECTION < 0) {
        cutMotor->moveTo(-50000); // Move far in negative direction
    } else {
        cutMotor->moveTo(50000);  // Move far in positive direction
    }
    
    Serial.println("Cut motor moving to home position...");
}

bool isCutMotorHoming() {
    if (!cutMotor) return false;
    
    // Update homing switch
    cutHomingSwitch.update();
    
    // Check if switch is triggered
    if (cutHomingSwitch.read() == HIGH) {
        // Switch triggered - stop motor immediately and set position to 0
        cutMotor->forceStopAndNewPosition(0);
        cutMotorHomed = true;
        Serial.println("Cut motor homed successfully - position set to 0");
        return false; // Homing complete
    }
    
    // Check if motor is still running
    if (!cutMotor->isRunning()) {
        Serial.println("ERROR: Cut motor stopped without hitting home switch");
        return false; // Homing failed
    }
    
    return true; // Still homing
}

//* ************************************************************************
//* ************************ POSITION MOTOR HOMING ***********************
//* ************************************************************************
void homePositionMotor() {
    if (!positionMotor) {
        Serial.println("ERROR: Position motor not initialized");
        return;
    }
    
    Serial.println("Starting position motor homing sequence");
    
    // Configure motor for homing
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_HOMING_SPEED);
    positionMotor->setAcceleration((uint32_t)POSITION_MOTOR_NORMAL_ACCELERATION);
    
    // Move in positive direction (POSITION_HOMING_DIRECTION = 1)
    if (POSITION_HOMING_DIRECTION > 0) {
        positionMotor->moveTo(50000); // Move far in positive direction
    } else {
        positionMotor->moveTo(-50000); // Move far in negative direction
    }
    
    Serial.println("Position motor moving to home position...");
}

bool isPositionMotorHoming() {
    if (!positionMotor) return false;
    
    // Update homing switch
    positionHomingSwitch.update();
    
    // Check if switch is triggered
    if (positionHomingSwitch.read() == HIGH) {
        // Switch triggered - stop motor immediately and set position to (2 + POSITION_TRAVEL_DISTANCE)
        long homePosition = (long)((2.0 + POSITION_TRAVEL_DISTANCE) * POSITION_MOTOR_STEPS_PER_INCH);
        positionMotor->forceStopAndNewPosition(homePosition);
        positionMotorHomed = true;
        
        Serial.print("Position motor homed successfully - position set to ");
        Serial.print(2.0 + POSITION_TRAVEL_DISTANCE);
        Serial.println(" inches");
        
        return false; // Homing complete
    }
    
    // Check if motor is still running
    if (!positionMotor->isRunning()) {
        Serial.println("ERROR: Position motor stopped without hitting home switch");
        return false; // Homing failed
    }
    
    return true; // Still homing
}

//* ************************************************************************
//* ************************ POSITION MOTOR TRAVEL MOVE ******************
//* ************************************************************************
void movePositionMotorToTravelAfterHoming() {
    if (!positionMotor || !positionMotorHomed) {
        Serial.println("ERROR: Cannot move to travel position - position motor not homed");
        return;
    }
    
    Serial.println("Moving position motor to travel position after homing");
    
    // Configure motor for normal operation
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_NORMAL_SPEED);
    positionMotor->setAcceleration((uint32_t)POSITION_MOTOR_NORMAL_ACCELERATION);
    
    // Move to POSITION_TRAVEL_DISTANCE position
    long travelPosition = (long)(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
    positionMotor->moveTo(travelPosition);
    
    Serial.print("Position motor moving to ");
    Serial.print(POSITION_TRAVEL_DISTANCE);
    Serial.println(" inches travel position");
}

bool isPositionMotorMovingToTravel() {
    if (!positionMotor) return false;
    
    // Check if motor is still running
    if (!positionMotor->isRunning()) {
        positionMotorMovedToTravel = true;
        Serial.println("Position motor reached travel position");
        return false; // Move complete
    }
    
    return true; // Still moving
}

//* ************************************************************************
//* ************************ HOMING SEQUENCE CONTROL *********************
//* ************************************************************************
void startHomingSequence() {
    Serial.println("Starting complete homing sequence");
    
    // Reset homing flags
    cutMotorHomed = false;
    positionMotorHomed = false;
    positionMotorMovedToTravel = false;
    isHomed = false;
    
    // Start cut motor homing
    homeCutMotor();
}

bool isHomingSequenceComplete() {
    // Check if both motors are homed and position motor has moved to travel position
    return cutMotorHomed && positionMotorHomed && positionMotorMovedToTravel;
}

void updateHomingSequence() {
    // Handle cut motor homing
    if (!cutMotorHomed && cutMotor && cutMotor->isRunning()) {
        if (!isCutMotorHoming()) {
            // Cut motor homing finished (success or failure)
            if (cutMotorHomed) {
                // Start position motor homing if cut motor homing was successful
                homePositionMotor();
            }
        }
    }
    
    // Handle position motor homing
    if (cutMotorHomed && !positionMotorHomed && positionMotor && positionMotor->isRunning()) {
        if (!isPositionMotorHoming()) {
            // Position motor homing finished (success or failure)
            if (positionMotorHomed) {
                // Start move to travel position
                movePositionMotorToTravelAfterHoming();
            }
        }
    }
    
    // Handle position motor move to travel position
    if (cutMotorHomed && positionMotorHomed && !positionMotorMovedToTravel && positionMotor && positionMotor->isRunning()) {
        isPositionMotorMovingToTravel();
    }
    
    // Update global homing flag
    if (isHomingSequenceComplete() && !isHomed) {
        isHomed = true;
        Serial.println("Complete homing sequence finished successfully");
    }
}

//* ************************************************************************
//* ************************ HOMING STATUS FUNCTIONS *********************
//* ************************************************************************
bool isCutMotorHomingComplete() {
    return cutMotorHomed;
}

bool isPositionMotorHomingComplete() {
    return positionMotorHomed;
}

bool isPositionMotorAtTravelPosition() {
    return positionMotorMovedToTravel;
}

void resetHomingFlags() {
    cutMotorHomed = false;
    positionMotorHomed = false;
    positionMotorMovedToTravel = false;
    isHomed = false;
    Serial.println("Homing flags reset");
} 