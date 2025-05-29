#include "StateMachine/StateMachine.h"
#include "Config/Config.h"
#include <AccelStepper.h>
#include <Bounce2.h>
#include "OTA_Manager.h"

//* ************************************************************************
//* ************************ HOMING FUNCTIONS *****************************
//* ************************************************************************
//! Simplified homing functions - only essential functionality
//! Cut motor: Home to negative direction, stop on switch, set position to 0
//! Position motor: Home to positive direction, stop on switch, set position calculated from travel distance

// External variable declarations
extern AccelStepper cutMotor;
extern AccelStepper positionMotor;
extern Bounce cutHomingSwitch;
extern Bounce positionHomingSwitch;
extern bool isHomed;

//* ************************************************************************
//* ************************ SIMPLE BLOCKING HOMING **********************
//* ************************************************************************
//! Blocking homing - motors home sequentially and system waits

void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout) {
    Serial.println("Homing cut motor...");
    unsigned long startTime = millis();
    cutMotor.setSpeed(CUT_MOTOR_HOMING_SPEED);
    cutMotor.moveTo(CUT_HOMING_DIRECTION * CUT_MOTOR_HOMING_DISTANCE);

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
        cutMotor.run();
        //! Handle OTA updates
        handleOTA();
        yield(); // Prevent watchdog reset
        if (millis() - startTime > timeout) {
            Serial.println("Cut motor homing timeout!");
            cutMotor.stop();
            return;
        }
    }
    cutMotor.stop();
    cutMotor.setCurrentPosition(0);
    Serial.println("Cut motor homed to position 0");
}

void homePositionMotorBlocking(Bounce& homingSwitch, unsigned long timeout) {
    Serial.println("Homing position motor...");
    unsigned long startTime = millis();
    positionMotor.setSpeed(POSITION_MOTOR_HOMING_SPEED);
    positionMotor.moveTo(POSITION_HOMING_DIRECTION * POSITION_MOTOR_HOMING_DISTANCE);

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
        positionMotor.run();
        //! Handle OTA updates
        handleOTA();
        yield(); // Prevent watchdog reset
        if (millis() - startTime > timeout) {
            Serial.println("Position motor homing timeout!");
            positionMotor.stop();
            return;
        }
    }
    positionMotor.stop();
    positionMotor.setCurrentPosition(POSITION_MOTOR_TRAVEL_POSITION + 1.0 * POSITION_MOTOR_STEPS_PER_INCH);
    Serial.print("Position motor homed to position ");
    Serial.print(POSITION_TRAVEL_DISTANCE);
    Serial.println(" inches");
    
    // Move to travel position after homing
    positionMotor.setSpeed(POSITION_MOTOR_NORMAL_SPEED);
    positionMotor.moveTo(POSITION_MOTOR_TRAVEL_POSITION);
    Serial.println("Moving to travel position...");
    while(positionMotor.distanceToGo() != 0) {
        positionMotor.run();
        //! Handle OTA updates
        handleOTA();
        yield(); // Prevent watchdog reset
        delay(5); // Small delay to prevent excessive loop iterations
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
    homeCutMotorBlocking(cutHomingSwitch, CUT_HOME_TIMEOUT);
    
    // Home position motor second  
    homePositionMotorBlocking(positionHomingSwitch, POSITION_HOME_TIMEOUT);
    
    isHomed = true;
    Serial.println("=== HOMING SEQUENCE COMPLETE ===");
}

//* ************************************************************************
//* ************************ DIAGNOSTIC FUNCTIONS ************************
//* ************************************************************************
//! Simple diagnostic functions

bool checkAndRecalibrateCutMotorHome(int attempts) {
    bool sensorDetectedHome = false;
    for (int i = 0; i < attempts; i++) {
        cutHomingSwitch.update();
        Serial.print("Cut position switch read attempt "); 
        Serial.print(i + 1); 
        Serial.print(": "); 
        Serial.println(cutHomingSwitch.read());
        
        if (cutHomingSwitch.read() == HIGH) {
            sensorDetectedHome = true;
            cutMotor.setCurrentPosition(0);
            Serial.println("Cut motor position recalibrated to 0");
            break;
        }
    }
    return sensorDetectedHome;
} 