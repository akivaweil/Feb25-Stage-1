#include "StateMachine/StateMachine.h"
#include <FastAccelStepper.h>
#include <Bounce2.h>

//* ************************************************************************
//* ************************ HOMING FUNCTIONS *****************************
//* ************************************************************************
//! Simplified homing functions - only essential functionality
//! Cut motor: Home to negative direction, stop on switch, set position to 0
//! Position motor: Home to positive direction, stop on switch, set position calculated from travel distance

// External variable declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;
extern Bounce cutHomingSwitch;
extern Bounce positionHomingSwitch;
extern bool isHomed;

//* ************************************************************************
//* ************************ SIMPLE BLOCKING HOMING **********************
//* ************************************************************************
//! Blocking homing - motors home sequentially and system waits

void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout) {
    if (!cutMotor) return;
    
    Serial.println("Homing cut motor...");
    unsigned long startTime = millis();
    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_HOMING_SPEED);
    cutMotor->moveTo(-40000);

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
        yield(); // Prevent watchdog reset
        if (millis() - startTime > timeout) {
            Serial.println("Cut motor homing timeout!");
            cutMotor->stopMove();
            return;
        }
    }
    cutMotor->stopMove();
    cutMotor->setCurrentPosition(0);
    Serial.println("Cut motor homed to position 0");
}

void homePositionMotorBlocking(Bounce& homingSwitch) {
    if (!positionMotor) return;
    
    Serial.println("Homing position motor...");
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_HOMING_SPEED);
    positionMotor->moveTo(10000 * POSITION_MOTOR_STEPS_PER_INCH);

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
        yield(); // Prevent watchdog reset
    }
    positionMotor->stopMove();
    positionMotor->setCurrentPosition(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
    Serial.print("Position motor homed to position ");
    Serial.print(POSITION_TRAVEL_DISTANCE);
    Serial.println(" inches");
    
    // Move to travel position after homing
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_NORMAL_SPEED);
    positionMotor->moveTo(POSITION_MOTOR_TRAVEL_POSITION);
    Serial.println("Moving to travel position...");
    while(positionMotor->isRunning()) {
        yield(); // Prevent watchdog reset
        delay(10); // Small delay to prevent excessive loop iterations
    }
    Serial.println("Position motor at travel position");
}

//* ************************************************************************
//* ************************ COMPLETE HOMING SEQUENCE ********************
//* ************************************************************************
//! Single function to home both motors in sequence

void executeCompleteHomingSequence() {
    Serial.println("=== STARTING COMPLETE HOMING SEQUENCE ===");
    isHomed = false;
    
    // Home cut motor first
    homeCutMotorBlocking(cutHomingSwitch, 30000); // 30 second timeout
    
    // Home position motor second  
    homePositionMotorBlocking(positionHomingSwitch);
    
    isHomed = true;
    Serial.println("=== HOMING SEQUENCE COMPLETE ===");
}

//* ************************************************************************
//* ************************ DIAGNOSTIC FUNCTIONS ************************
//* ************************************************************************
//! Simple diagnostic functions

bool checkAndRecalibrateCutMotorHome(int attempts) {
    if (!cutMotor) return false;

    bool sensorDetectedHome = false;
    for (int i = 0; i < attempts; i++) {
        cutHomingSwitch.update();
        Serial.print("Cut position switch read attempt "); 
        Serial.print(i + 1); 
        Serial.print(": "); 
        Serial.println(cutHomingSwitch.read());
        
        if (cutHomingSwitch.read() == HIGH) {
            sensorDetectedHome = true;
            cutMotor->setCurrentPosition(0);
            Serial.println("Cut motor position recalibrated to 0");
            break;
        }
    }
    return sensorDetectedHome;
} 