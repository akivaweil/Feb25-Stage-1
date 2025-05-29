#include "StateMachine/StateMachine.h"
#include "StateMachine/MotorFunctions.h"
#include "StateMachine/SensorFunctions.h"
#include <FastAccelStepper.h>

//* ************************************************************************
//* ************************ YESWOOD FUNCTIONS ***************************
//* ************************************************************************
//! YESWOOD-specific functions for wood positioning and secure clamping
//! Functions dedicated to the YESWOOD state operations

// External variable declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;

bool isCutMotorAtHome() {
    if (!cutMotor) {
        Serial.println("WARNING: Cut motor not initialized");
        return false;
    }
    
    return (cutMotor->getCurrentPosition() == 0);
}

bool isPositionMotorAtPosition(float targetPositionInches) {
    if (!positionMotor) {
        Serial.println("WARNING: Position motor not initialized");
        return false;
    }
    
    long targetSteps = (long)(targetPositionInches * POSITION_MOTOR_STEPS_PER_INCH);
    long currentPosition = positionMotor->getCurrentPosition();
    
    // Allow for small tolerance (within 5 steps)
    return (abs(currentPosition - targetSteps) <= 5);
}

bool isPositionMotorAtHome() {
    if (!positionMotor) {
        Serial.println("WARNING: Position motor not initialized");
        return false;
    }
    
    return (positionMotor->getCurrentPosition() == 0);
}

bool isPositionMotorAtTravel() {
    if (!positionMotor) {
        Serial.println("WARNING: Position motor not initialized");
        return false;
    }
    
    long travelSteps = (long)(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
    long currentPosition = positionMotor->getCurrentPosition();
    
    // Allow for small tolerance (within 5 steps)
    return (abs(currentPosition - travelSteps) <= 5);
}

void performWoodSecuring() {
    Serial.println("Performing wood securing sequence");
    
    // Secure the wood piece with clamps
    extendClamp(WOOD_SECURE_CLAMP_TYPE);
    
    // Wait for clamp to engage
    delay(CYLINDER_RETRACT_TIME);
    
    // Retract the wood secure clamp to allow movement
    retractClamp(WOOD_SECURE_CLAMP_TYPE);
    
    Serial.println("Wood securing complete");
}

void performWoodReleasing() {
    Serial.println("Performing wood releasing sequence");
    
    // Retract wood secure clamp
    retractClamp(WOOD_SECURE_CLAMP_TYPE);
    delay(CYLINDER_RETRACT_TIME);
    
    Serial.println("Wood releasing complete");
}

bool verifyWoodPosition() {
    // Check wood sensor to verify wood is in correct position
    if (readWoodSensor()) {
        Serial.println("Wood position verified - wood detected in correct position");
        return true;
    } else {
        Serial.println("WARNING: Wood position verification failed - no wood detected");
        return false;
    }
}

bool verifyClampsRetracted() {
    // This function would verify that clamps are in retracted position
    // For now, we'll assume the function works based on timing
    Serial.println("Verifying clamps are retracted");
    return true;
}

bool verifyClampsExtended() {
    // This function would verify that clamps are in extended position
    // For now, we'll assume the function works based on timing
    Serial.println("Verifying clamps are extended");
    return true;
}

void logYeswoodStatus() {
    Serial.println("=== YESWOOD STATE STATUS ===");
    
    if (cutMotor) {
        Serial.print("Cut motor position: ");
        Serial.println(cutMotor->getCurrentPosition());
        Serial.print("Cut motor at home: ");
        Serial.println(isCutMotorAtHome() ? "YES" : "NO");
    }
    
    if (positionMotor) {
        Serial.print("Position motor position: ");
        Serial.print(positionMotor->getCurrentPosition());
        Serial.print(" steps (");
        Serial.print((float)positionMotor->getCurrentPosition() / POSITION_MOTOR_STEPS_PER_INCH);
        Serial.println(" inches)");
        Serial.print("Position motor at home: ");
        Serial.println(isPositionMotorAtHome() ? "YES" : "NO");
        Serial.print("Position motor at travel: ");
        Serial.println(isPositionMotorAtTravel() ? "YES" : "NO");
    }
    
    Serial.println("=============================");
} 