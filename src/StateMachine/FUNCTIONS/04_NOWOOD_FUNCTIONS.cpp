#include "StateMachine/StateMachine.h"
#include "StateMachine/MotorFunctions.h"
#include "StateMachine/SensorFunctions.h"

//* ************************************************************************
//* ************************ NOWOOD FUNCTIONS ***************************
//* ************************************************************************
//! NOWOOD-specific functions for no-wood conditions and wood detection
//! Functions dedicated to the NOWOOD state

void scanForWood() {
    Serial.println("Scanning for wood presence");
    
    // Check all wood detection sensors
    bool woodDetected = readWoodSensor();
    bool woodSuctioned = readWoodSuctionSensor();
    
    Serial.print("Wood sensor readings - Wood Present:");
    Serial.print(woodDetected ? "ACTIVE" : "INACTIVE");
    Serial.print(" Wood Suctioned:");
    Serial.println(woodSuctioned ? "ACTIVE" : "INACTIVE");
}

void activateWoodFeeder() {
    Serial.println("Activating wood feeding mechanism");
    
    // Activate wood feeder stepper
    movePositionMotorSteps(100.0, true, 10.0);  // Slow feed motion
    
    // Check if wood enters system
    delay(2000);  // Wait for wood to potentially enter
    
    if (readWoodSensor()) {
        Serial.println("Wood detected after feeding activation");
    } else {
        Serial.println("No wood detected after feeding attempt");
    }
}

void retractAllMechanisms() {
    Serial.println("Retracting all mechanisms to safe position");
    
    // Retract all cylinders
    retractAllCylinders();
    
    Serial.println("All mechanisms retracted");
}

bool checkWoodSupply() {
    // Check if there's wood available in the supply
    // This could be a separate sensor or mechanism
    
    Serial.println("Checking wood supply availability");
    
    // For now, assume wood supply is available
    // This would need to be connected to an actual sensor when available
    Serial.println("Wood supply available (no sensor connected)");
    return true;
}

void triggerWoodRequest() {
    Serial.println("Triggering wood request signal");
    
    // Activate wood request indicator/signal
    digitalWrite((int)STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite((int)STATUS_LED_PIN, LOW);
    delay(100);
    digitalWrite((int)STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite((int)STATUS_LED_PIN, LOW);
    
    // Could also activate external signal relay here
    // digitalWrite((int)SPARE_DIGITAL_1, HIGH);
    
    Serial.println("Wood request signal sent");
}

void enterStandbyMode() {
    Serial.println("Entering standby mode - minimal power consumption");
    
    // Retract all cylinders for safety
    retractAllCylinders();
    
    // Turn off non-essential LEDs
    digitalWrite((int)STATUS_LED_PIN, LOW);
    
    // Keep only wood detection sensors active
    Serial.println("Standby mode active - monitoring for wood");
}

void exitStandbyMode() {
    Serial.println("Exiting standby mode - systems ready");
    
    // Re-enable systems
    digitalWrite((int)STATUS_LED_PIN, HIGH);
    
    Serial.println("Systems reactivated from standby");
} 