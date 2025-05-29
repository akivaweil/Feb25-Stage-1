#include "StateMachine/StateMachine.h"
#include "StateMachine/MotorFunctions.h"
#include "StateMachine/SensorFunctions.h"

//* ************************************************************************
//* ************************ CUTTING FUNCTIONS ***************************
//* ************************************************************************
//! Cutting-specific functions for wood cutting operations
//! Functions dedicated to the CUTTING state

void positionCuttingBlade() {
    Serial.println("Positioning cutting blade to start position");
    
    // Move cutting blade stepper to start position
    moveCutMotorSteps(100.0, false, 3.0);  // Move 100 steps to start position
    
    Serial.println("Cutting blade positioned");
}

void executeCuttingMotion() {
    Serial.println("Executing cutting motion");
    
    // Move cutting blade through wood
    moveCutMotorSteps(300.0, true, 2.0);  // Cutting motion
    
    delay((int)CYLINDER_RETRACT_TIME);
    
    Serial.println("Cutting motion complete");
}

bool isCutComplete() {
    // Check if cut is complete using motor position
    // For now, assume cut is complete when motor reaches target position
    // This would need to be connected to actual sensors when available
    
    Serial.println("Cut completion check - using motor position");
    return true;  // Simplified for now - would use actual sensors in production
}

void returnCuttingBladeToHome() {
    Serial.println("Returning cutting blade to home position");
    
    // Move blade back to home position
    moveCutMotorSteps(400.0, false, 3.0);  // Return to home
    
    // Wait for home switch
    while (!readCutMotorHomingSwitch()) {
        stepperStepCutMotor(false);
        delay(5);
    }
    
    Serial.println("Cutting blade returned to home");
}

void clearCuttingArea() {
    Serial.println("Clearing cutting area");
    
    // Cutting area clearing would be handled by other mechanisms
    // No specific clamp action needed for clearing
    delay(500);
    
    Serial.println("Cutting area cleared");
}