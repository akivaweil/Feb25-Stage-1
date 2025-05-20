// IMPORTANT NOTE: This file contains helper functions used by 'Stage 1 Feb25.cpp'.
// It relies on 'Stage 1 Feb25.cpp' for pin definitions and global variable declarations (via extern).
#include "Functions.h"

//* ************************************************************************
//* *********************** FUNCTION DEFINITIONS *************************
//* ************************************************************************
// This file contains helper functions for clamp control, LED control,
// inter-stage signaling, and motor control.

// Note: Pin definitions, global servo variables, motor objects, and related constants
// are declared as 'extern' in functions.h and defined in the main .cpp file.

//* ************************************************************************
//* *********************** SIGNALING FUNCTIONS ****************************
//* ************************************************************************
// Contains functions related to signaling other stages or components.

void sendSignalToStage2() {
  // Set the signal pin HIGH to trigger Stage 1 to Stage 2 machine (active HIGH)
  digitalWrite(STAGE2_SIGNAL_OUT_PIN, HIGH);
  signalStage2StartTime = millis();
  signalStage2Active = true;
  Serial.println("Signal sent to Stage 1 to Stage 2 machine");

  servoMotor.write(90);
  servoAt90StartTime = millis();
  servoIsAt90AndTiming = true;
  Serial.println("Servo moved to 90 degrees with signal.");
}

//* ************************************************************************
//* ************************* CLAMP FUNCTIONS ******************************
//* ************************************************************************
// Contains functions for controlling various clamps.
// Clamp Logic: LOW = engaged (extended), HIGH = disengaged (retracted)
// Catcher Clamp Logic: HIGH = engaged (extended), LOW = disengaged (retracted)

void extendPositionClamp() {
  digitalWrite(POSITION_CLAMP, LOW); // Engaged
  Serial.println("Position Clamp Extended (Engaged)");
}

void retractPositionClamp() {
  digitalWrite(POSITION_CLAMP, HIGH); // Disengaged
  Serial.println("Position Clamp Retracted (Disengaged)");
}

void extendWoodSecureClamp() {
  digitalWrite(WOOD_SECURE_CLAMP, LOW); // Engaged
  Serial.println("Wood Secure Clamp Extended (Engaged)");
}

void retractWoodSecureClamp() {
  digitalWrite(WOOD_SECURE_CLAMP, HIGH); // Disengaged
  Serial.println("Wood Secure Clamp Retracted (Disengaged)");
}

void extendCatcherClamp() {
  digitalWrite(CATCHER_CLAMP_PIN, HIGH); // Engaged (Reversed Logic)
  catcherClampEngageTime = millis();
  catcherClampIsEngaged = true;
  Serial.println("Catcher Clamp Extended (Engaged)");
}

void retractCatcherClamp() {
  digitalWrite(CATCHER_CLAMP_PIN, LOW); // Disengaged (Reversed Logic)
  catcherClampIsEngaged = false; // Assuming we want to clear the flag when explicitly retracting
  Serial.println("Catcher Clamp Retracted (Disengaged)");
}

//* ************************************************************************
//* *************************** LED FUNCTIONS ******************************
//* ************************************************************************
// Contains functions for controlling LEDs.

void turnRedLedOn() {
  digitalWrite(RED_LED, HIGH);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  Serial.println("Red LED ON");
}

void turnRedLedOff() {
  digitalWrite(RED_LED, LOW);
  Serial.println("Red LED OFF");
}

void turnYellowLedOn() {
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  Serial.println("Yellow LED ON");
}

void turnYellowLedOff() {
  digitalWrite(YELLOW_LED, LOW);
  Serial.println("Yellow LED OFF");
}

void turnGreenLedOn() {
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  Serial.println("Green LED ON");
}

void turnGreenLedOff() {
  digitalWrite(GREEN_LED, LOW);
  Serial.println("Green LED OFF");
}

void turnBlueLedOn() {
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  Serial.println("Blue LED ON");
}

void turnBlueLedOff() {
  digitalWrite(BLUE_LED, LOW);
  Serial.println("Blue LED OFF");
}

void allLedsOff() {
    turnRedLedOff();
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();
}

//* ************************************************************************
//* *********************** MOTOR CONTROL FUNCTIONS ************************
//* ************************************************************************

void configureCutMotorForCutting() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)CUT_NORMAL_SPEED);
        cutMotor->setAcceleration((uint32_t)CUT_ACCELERATION);
    }
}

void configureCutMotorForReturn() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)CUT_RETURN_SPEED);
        cutMotor->setAcceleration((uint32_t)CUT_ACCELERATION); // Assuming same acceleration for return for now
    }
}

void configurePositionMotorForNormalOperation() {
    if (positionMotor) {
        positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED);
        positionMotor->setAcceleration((uint32_t)POSITION_ACCELERATION);
    }
}

void configurePositionMotorForReturn() {
    if (positionMotor) {
        positionMotor->setSpeedInHz((uint32_t)POSITION_RETURN_SPEED);
        positionMotor->setAcceleration((uint32_t)POSITION_RETURN_ACCELERATION);
    }
}

void moveCutMotorToCut() {
    if (cutMotor) {
        cutMotor->moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
    }
}

void moveCutMotorToHome() {
    if (cutMotor) {
        cutMotor->moveTo(-0.02 * CUT_MOTOR_STEPS_PER_INCH); // Minimal overshoot
    }
}

void movePositionMotorToTravel() {
    if (positionMotor) {
        positionMotor->moveTo(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
    }
}

void movePositionMotorToHome() {
    if (positionMotor) {
        positionMotor->moveTo(0);
    }
}

void movePositionMotorToPosition(float targetPositionInches) {
    if (positionMotor) {
        positionMotor->moveTo(targetPositionInches * POSITION_MOTOR_STEPS_PER_INCH);
    }
}

void stopCutMotor() {
    if (cutMotor) {
        cutMotor->stopMove();
    }
}

void stopPositionMotor() {
    if (positionMotor) {
        positionMotor->stopMove();
    }
}

// Basic blocking homing function for Cut Motor - can be expanded
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout) {
    if (!cutMotor) return;
    unsigned long startTime = millis();
    cutMotor->setSpeedInHz((uint32_t)CUT_HOMING_SPEED);
    cutMotor->moveTo(-40000); // Large move towards switch

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
        if (millis() - startTime > timeout) {
            Serial.println("Cut motor homing timeout!");
            cutMotor->stopMove();
            return;
        }
        // Yield for other tasks, important in blocking loops
        // delay(1); // Or use a more RTOS-friendly yield if available
    }
    cutMotor->stopMove();
    cutMotor->setCurrentPosition(0);
    Serial.println("Cut motor homed.");
}

// Basic blocking homing function for Position Motor - can be expanded
void homePositionMotorBlocking(Bounce& homingSwitch) {
    if (!positionMotor) return;
    positionMotor->setSpeedInHz((uint32_t)POSITION_HOMING_SPEED);
    positionMotor->moveTo(-10000 * POSITION_MOTOR_STEPS_PER_INCH); // Large move towards switch

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
        // No timeout here as per original logic, but could be added
        // delay(1);
    }
    positionMotor->stopMove();
    positionMotor->setCurrentPosition(-1 * POSITION_MOTOR_STEPS_PER_INCH); // Offset
    Serial.println("Position motor homed.");
    configurePositionMotorForNormalOperation(); // Prepare for next move
}

void movePositionMotorToInitialAfterHoming() {
    if (positionMotor) {
        configurePositionMotorForNormalOperation();
        movePositionMotorToTravel(); // Moves to POSITION_TRAVEL_DISTANCE
        // Wait for move completion (blocking for simplicity, can be non-blocking)
        while(positionMotor->isRunning()){
            // delay(1);
        }
    }
} 