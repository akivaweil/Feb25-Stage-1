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

void sendSignalToTA() {
  // Set the signal pin HIGH to trigger Transfer Arm (active HIGH)
  digitalWrite(TA_SIGNAL_OUT_PIN, HIGH);
  signalTAStartTime = millis();
  signalTAActive = true;
  Serial.println("Signal sent to Transfer Arm (TA)");

  servoMotor.write(90);
  servoAt90StartTime = millis();
  servoIsAt90AndTiming = true;
  Serial.println("Servo moved to 90 degrees with TA signal.");
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

// Point 1: LED Blinking Logic
void handleHomingLedBlink() {
    static unsigned long blinkTimer = 0; // Retain static for independent timing
    if (millis() - blinkTimer > 500) {
        blinkState = !blinkState;
        if (blinkState) turnBlueLedOn(); else turnBlueLedOff();
        blinkTimer = millis();
    }
}

void handleErrorLedBlink() {
    if (millis() - lastErrorBlinkTime > 250) { // lastErrorBlinkTime is global
        errorBlinkState = !errorBlinkState; // errorBlinkState is global
        if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        lastErrorBlinkTime = millis();
    }
}

// Note: For SUCTION_ERROR_HOLD, the local static variables were moved to global
// or passed as references if they need to be unique to that state's instance of blinking.
// For this refactor, assuming lastSuctionErrorBlinkTime and suctionErrorBlinkState
// were intended to be specific to the SUCTION_ERROR_HOLD state, they are passed by reference.
void handleSuctionErrorLedBlink(unsigned long& lastBlinkTimeRef, bool& blinkStateRef) {
    if (millis() - lastBlinkTimeRef >= 1500) {
        lastBlinkTimeRef = millis();
        blinkStateRef = !blinkStateRef;
        if(blinkStateRef) turnRedLedOn(); else turnRedLedOff();
    }
    // Ensure other LEDs are off
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();
}

//* ************************************************************************
//* *********************** MOTOR CONTROL FUNCTIONS ************************
//* ************************************************************************

void configureCutMotorForCutting() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_NORMAL_SPEED);
        cutMotor->setAcceleration((uint32_t)CUT_MOTOR_NORMAL_ACCELERATION);
    }
}

void configureCutMotorForReturn() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_RETURN_SPEED);
        cutMotor->setAcceleration((uint32_t)CUT_MOTOR_NORMAL_ACCELERATION); // Assuming same acceleration for return for now
    }
}

void configurePositionMotorForNormalOperation() {
    if (positionMotor) {
        positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_NORMAL_SPEED);
        positionMotor->setAcceleration((uint32_t)POSITION_MOTOR_NORMAL_ACCELERATION);
    }
}

void configurePositionMotorForReturn() {
    if (positionMotor) {
        positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_RETURN_SPEED);
        positionMotor->setAcceleration((uint32_t)POSITION_MOTOR_RETURN_ACCELERATION);
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
    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_HOMING_SPEED);
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
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_HOMING_SPEED);
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
            // delay(1); // Consider if non-blocking is needed for responsiveness
        }
    }
}

// Point 3: Complex conditional logic
// Checks the cut motor homing switch multiple times and recalibrates if detected.
// Returns true if home detected and recalibrated, false otherwise.
bool checkAndRecalibrateCutMotorHome(int attempts) {
    if (!cutMotor) return false; // Should not happen if motor is initialized

    bool sensorDetectedHome = false;
    for (int i = 0; i < attempts; i++) {
        delay(30); // Small delay for sensor to settle or for debounce update
        cutHomingSwitch.update();
        Serial.print("Cut position switch read attempt "); Serial.print(i + 1); Serial.print(": "); Serial.println(cutHomingSwitch.read());
        if (cutHomingSwitch.read() == HIGH) {
            sensorDetectedHome = true;
            cutMotor->setCurrentPosition(0); // Recalibrate to 0 when switch is hit
            Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
            break;
        }
    }
    return sensorDetectedHome;
}

//* ************************************************************************
//* ************************* SWITCH LOGIC FUNCTIONS ***********************
//* ************************************************************************
// Point 2: Switch handling

void handleReloadMode() {
    if (currentState == READY) { // Only applicable in READY state
        bool reloadSwitchOn = reloadSwitch.read() == HIGH;
        if (reloadSwitchOn && !isReloadMode) {
            isReloadMode = true;
            retractPositionClamp();
            retractWoodSecureClamp();
            turnYellowLedOn();
            Serial.println("Entered reload mode");
        } else if (!reloadSwitchOn && isReloadMode) {
            isReloadMode = false;
            extendPositionClamp();
            extendWoodSecureClamp();
            turnYellowLedOff();
            Serial.println("Exited reload mode, ready for operation");
        }
    }
}

void handleErrorAcknowledgement() {
    // This handles the general error acknowledgement via reloadSwitch
    // It was present in the main loop and also within the CUTTING state's homePositionErrorDetected block.
    if (reloadSwitch.rose() && (currentState == ERROR || currentState == CUTTING)) { // Check if in ERROR or if a cutting error is active
        // For CUTTING state, the homePositionErrorDetected flag logic needs to remain there,
        // but the transition to ERROR_RESET can be centralized if errorAcknowledged is set.
        if (currentState == ERROR) {
            currentState = ERROR_RESET;
            errorAcknowledged = true; // Set flag, main loop will see this for ERROR state
            Serial.println("Error acknowledged by reload switch (from ERROR state). Transitioning to ERROR_RESET.");
        }
        // If in CUTTING, setting errorAcknowledged might be used by the CUTTING state to proceed.
        // The original CUTTING state logic directly transitioned. For now, we set the flag.
        // The calling code in CUTTING will need to check this flag if it relies on it.
        // For direct transition from specific cutting error, that logic is better kept in cutting stage.
        // This function primarily handles the generic ERROR state reset.
    }
}

void handleStartSwitchSafety() {
    // Original logic from setup() and main loop for startSwitchSafe
    // Call this once in setup() after startCycleSwitch.update()
    // And continuously in the main loop before checking shouldStartCycle()
    if (!startSwitchSafe && startCycleSwitch.fell()) {
        startSwitchSafe = true;
        Serial.println("Start switch is now safe to use (cycled OFF).");
    }
    // Initial check (typically for setup)
    // This part might be better directly in setup, but included here for completeness if called from there.
    // If called repeatedly from loop, this `else if` might be redundant if startSwitchSafe is managed correctly.
    /* else if (startCycleSwitch.read() == HIGH && !startSwitchSafe) {
        Serial.println("WARNING: Start switch is ON. Turn it OFF before operation.");
    }*/
}

void handleStartSwitchContinuousMode(){
    bool startSwitchOn = startCycleSwitch.read() == HIGH;
    if (startSwitchOn != continuousModeActive && startSwitchSafe) {
        continuousModeActive = startSwitchOn;
        if (continuousModeActive) {
            Serial.println("Continuous operation mode activated");
        } else {
            Serial.println("Continuous operation mode deactivated");
        }
    }
}

//* ************************************************************************
//* ************************* STATE LOGIC HELPERS **************************
//* ************************************************************************
// Point 3: Complex conditional logic

bool shouldStartCycle() {
    // Condition from READY state to start a cycle
    return ((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress))
            && !woodSuctionError && startSwitchSafe);
}

// Point 4: Servo Timing
void handleServoReturn() {
    if (servoIsAt90AndTiming && (millis() - servoAt90StartTime >= SERVO_HOLD_AT_90_DURATION_MS)) {
        servoMotor.write(2);
        servoIsAt90AndTiming = false;
        Serial.println("Servo returned to 2 degrees after hold.");
    }
}

// Function for point 5, if pursued and renamed
void handleTASignalTiming() { 
  if (signalTAActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
    digitalWrite(TA_SIGNAL_OUT_PIN, LOW); // Return to inactive state (LOW)
    signalTAActive = false;
    Serial.println("Signal to Transfer Arm (TA) completed"); 
  }
}

void handleCatcherClampDisengage() { // Point 4
  if (catcherClampIsEngaged && (millis() - catcherClampEngageTime >= CATCHER_CLAMP_ENGAGE_DURATION_MS)) {
    retractCatcherClamp();
    Serial.println("Catcher Clamp disengaged after 1 second.");
  }
} 