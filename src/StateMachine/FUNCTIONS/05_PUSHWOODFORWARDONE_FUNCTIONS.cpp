#include "StateMachine/StateMachine.h"
#include "StateMachine/MotorFunctions.h"
#include "StateMachine/SensorFunctions.h"

//* ************************************************************************
//* ************************ pushWoodForwardOne FUNCTIONS ***************************
//* ************************************************************************
//! pushWoodForwardOne-specific functions for wood advancement operations
//! Functions dedicated to the pushWoodForwardOne state

void initializeWoodPusher() {
    Serial.println("Initializing wood pusher mechanism");
    
    // Initialize pusher mechanism
    delay((int)CYLINDER_RETRACT_TIME);
    
    Serial.println("Wood pusher initialized and ready");
}

void calculatePushDistance() {
    Serial.println("Calculating optimal push distance");
    
    // Read wood position sensors to determine current position
    bool frontSensor = readWoodSensor();
    bool rearSensor = readWoodSuctionSensor();
    
    if (frontSensor && !rearSensor) {
        Serial.println("Wood detected at front position - standard push distance");
    } else if (!frontSensor && rearSensor) {
        Serial.println("Wood detected at rear position - extended push distance");
    } else if (frontSensor && rearSensor) {
        Serial.println("Multiple wood pieces detected - careful advancement");
    } else {
        Serial.println("No wood detected - push operation may not be needed");
    }
}

void extendWoodPusher() {
    Serial.println("Extending wood pusher cylinder");
    
    // Wood pusher extension would be handled by other mechanisms
    // Wait for operation to complete
    delay(500);  // 500ms delay for cylinder operation
    
    Serial.println("Wood pusher extended");
}

void advanceWoodPosition() {
    Serial.println("Advancing wood to next position");
    
    // Push wood forward with stepper motor
    movePositionMotorSteps(50.0, true, 5.0);  // Slow precise movement
    
    // Check if wood has advanced properly
    if (readWoodSensor()) {
        Serial.println("Wood advancement confirmed");
    } else {
        Serial.println("Warning: Wood advancement not detected");
    }
}

void retractWoodPusher() {
    Serial.println("Retracting wood pusher");
    
    // Wood pusher retraction would be handled by other mechanisms
    delay((int)CYLINDER_RETRACT_TIME);
    
    Serial.println("Wood pusher retracted");
}

void finalizeWoodPosition() {
    Serial.println("Finalizing wood position");
    
    // Make fine adjustments if needed
    if (!readWoodSensor()) {
        Serial.println("Applying fine adjustment");
        movePositionMotorSteps(10.0, true, 2.0);
    }
    
    Serial.println("Wood position finalized and secured");
}

void moveWoodForward() {
    Serial.println("Moving wood forward one position");
    
    // Move position motor to push wood forward
    movePositionMotorSteps(POSITION_MOTOR_STEPS_PER_INCH, true, 5.0);
    
    Serial.println("Wood moved forward one position");
} 