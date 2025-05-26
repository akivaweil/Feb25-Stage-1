#include "StateMachine/HomingState.h"
#include "StateMachine/StateManager.h"
#include "Functions.h"

//* ************************************************************************
//* ************************** HOMING STATE ********************************
//* ************************************************************************
// Handles the homing sequence for all motors.
// Step 1: Blink blue LED to indicate homing in progress.
// Step 2: Home the cut motor (blocking). If fails, it might retry or transition to ERROR (currently retries).
// Step 3: If cut motor homed, home the position motor (blocking). Disengage position clamp before homing.
// Step 4: If position motor homed, move position motor to its initial position after homing (blocking). Re-engage position clamp.
// Step 5: If all homing and initial positioning are complete, set isHomed flag to true.
// Step 6: Turn off blue LED, turn on green LED.
// Step 7: Ensure servo is at 2 degrees.
// Step 8: Transition to IDLE state.

void HomingState::onEnter(StateManager& stateManager) {
    // Reset homing state variables when entering
    cutMotorHomed = false;
    positionMotorHomed = false;
    positionMotorMoved = false;
    positionHomingPhaseInitiated = false;
    blinkTimer = 0;
}

void HomingState::execute(StateManager& stateManager) {
    // Blink blue LED to indicate homing in progress
    if (millis() - blinkTimer > 500) {
        bool blinkState = stateManager.getBlinkState();
        blinkState = !blinkState;
        stateManager.setBlinkState(blinkState);
        if (blinkState) turnBlueLedOn(); else turnBlueLedOff();
        blinkTimer = millis();
    }

    if (!cutMotorHomed) {
        Serial.println("Starting cut motor homing phase (blocking)...");
        extern const unsigned long CUT_HOME_TIMEOUT; // This is in main.cpp
        homeCutMotorBlocking(*stateManager.getCutHomingSwitch(), CUT_HOME_TIMEOUT);
        if (stateManager.getCutMotor() && stateManager.getCutMotor()->getCurrentPosition() == 0) { // Check if homing was successful
            cutMotorHomed = true;
        } else {
            Serial.println("Cut motor homing failed or timed out. Retrying or error.");
        }
    } else if (!positionMotorHomed) {
        if (!positionHomingPhaseInitiated) {
            Serial.println("Starting position motor homing phase (blocking)..."); 
            retractPositionClamp(); 
            Serial.println("Position clamp disengaged for homing."); 
            positionHomingPhaseInitiated = true;
        }
        homePositionMotorBlocking(*stateManager.getPositionHomingSwitch());
        positionMotorHomed = true; 
        positionHomingPhaseInitiated = false; // Reset for next potential homing cycle
    } else if (!positionMotorMoved) {
        Serial.println("Moving position motor to initial position after homing (blocking)...");
        movePositionMotorToInitialAfterHoming(); 
        extendPositionClamp();
        Serial.println("Position clamp re-engaged.");
        positionMotorMoved = true;
        Serial.println("Position motor moved to initial position (blocking complete).");
    } else {
        Serial.println("Homing sequence complete. System ready."); 
        cutMotorHomed = false; 
        positionMotorHomed = false;
        positionMotorMoved = false;
        
        extern bool isHomed; // This is in main.cpp
        isHomed = true; 

        turnBlueLedOff();
        turnGreenLedOn();

        // Set initial servo position via function call
        handleCatcherServoReturn();
        stateManager.changeState(IDLE);
    }
} 