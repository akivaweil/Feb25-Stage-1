#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>
#include "Config/Config.h"
#include "Config/Pins_Definitions.h"
#include "StateMachine/StateMachine.h"

//* ************************************************************************
//* *********************** GENERAL HELPER FUNCTIONS **********************
//* ************************************************************************
// Helper functions for clamp control, LED control, inter-stage signaling,
// and motor control used by the main Stage 1 control system.

// External variable declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;
extern Servo catcherServo;
extern bool signalTAActive;
extern unsigned long signalTAStartTime;
extern unsigned long catcherServoActiveStartTime;
extern bool catcherServoIsActiveAndTiming;
extern unsigned long catcherClampEngageTime;
extern bool catcherClampIsEngaged;

// External variable declarations for additional functions
extern Bounce cutHomingSwitch;
extern Bounce positionHomingSwitch;
extern Bounce reloadSwitch;
extern Bounce startCycleSwitch;
extern SystemState currentState;
extern bool isReloadMode;
extern bool errorAcknowledged;
extern bool startSwitchSafe;
extern bool continuousModeActive;
extern bool cuttingCycleInProgress;
extern bool woodSuctionError;
extern bool woodCaughtCheckPending;
extern unsigned long woodCaughtCheckTime;
extern bool wasWoodCaughtError;

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

  catcherServo.write(CATCHER_SERVO_ACTIVE_POSITION);
  catcherServoActiveStartTime = millis();
  catcherServoIsActiveAndTiming = true;
  Serial.print("Catcher servo moved to ");
  Serial.print(CATCHER_SERVO_ACTIVE_POSITION);
  Serial.println(" degrees with TA signal.");
}

//* ************************************************************************
//* ************************* CLAMP CONTROL FUNCTIONS **********************
//* ************************************************************************
// Clamp control functions are now handled by the unified parameter-based system
// in 99_MOTOR_FUNCTIONS.cpp using extendClamp(ClampType) and retractClamp(ClampType)

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

void handleHomingLedBlink() {
    static unsigned long blinkTimer = 0;
    static bool blinkState = false;
    if (millis() - blinkTimer > 500) {
        blinkState = !blinkState;
        if (blinkState) turnBlueLedOn(); else turnBlueLedOff();
        blinkTimer = millis();
    }
}

void handleErrorLedBlink() {
    static unsigned long lastErrorBlinkTime = 0;
    static bool errorBlinkState = false;
    if (millis() - lastErrorBlinkTime > 250) {
        errorBlinkState = !errorBlinkState;
        if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        lastErrorBlinkTime = millis();
    }
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
        cutMotor->setAcceleration((uint32_t)CUT_MOTOR_NORMAL_ACCELERATION);
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
        cutMotor->moveTo(CUT_MOTOR_CUT_POSITION);
        Serial.println("Cut motor moving to cut position");
    }
}

void moveCutMotorToHome() {
    if (cutMotor) {
        cutMotor->moveTo(0);
        Serial.println("Cut motor moving to home position");
    }
}

void movePositionMotorToTravel() {
    if (positionMotor) {
        positionMotor->moveTo(POSITION_MOTOR_TRAVEL_POSITION);
        Serial.println("Position motor moving to travel position");
    }
}

void movePositionMotorToTravelWithEarlyActivation() {
    if (positionMotor) {
        positionMotor->moveTo(POSITION_MOTOR_TRAVEL_POSITION);
        Serial.println("Position motor moving to travel position with early activation monitoring");
        while(positionMotor->isRunning()){
            checkCatcherServoEarlyActivation();
            checkCatcherClampEarlyActivation();
        }
    }
}

void movePositionMotorToHome() {
    if (positionMotor) {
        positionMotor->moveTo(0);
        Serial.println("Position motor moving to home position");
    }
}

void movePositionMotorToPosition(float targetPositionInches) {
    if (positionMotor) {
        long targetSteps = (long)(targetPositionInches * STEPS_PER_INCH_POSITION);
        positionMotor->moveTo(targetSteps);
        Serial.print("Position motor moving to ");
        Serial.print(targetPositionInches);
        Serial.println(" inches");
    }
}

void stopCutMotor() {
    if (cutMotor) {
        cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
        Serial.println("Cut motor stopped");
    }
}

void stopPositionMotor() {
    if (positionMotor) {
        positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());
        Serial.println("Position motor stopped");
    }
}

//* ************************************************************************
//* ********************* MOTOR HOMING FUNCTIONS ***************************
//* ************************************************************************

// Basic blocking homing function for Cut Motor - can be expanded
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout) {
    if (!cutMotor) return;
    unsigned long startTime = millis();
    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_HOMING_SPEED);
    cutMotor->moveTo(-40000);

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
        if (millis() - startTime > timeout) {
            Serial.println("Cut motor homing timeout!");
            cutMotor->stopMove();
            return;
        }
    }
    cutMotor->stopMove();
    cutMotor->setCurrentPosition(0);
    Serial.println("Cut motor homed.");
}

// Basic blocking homing function for Position Motor - can be expanded
void homePositionMotorBlocking(Bounce& homingSwitch) {
    if (!positionMotor) return;
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_HOMING_SPEED);
    positionMotor->moveTo(10000 * POSITION_MOTOR_STEPS_PER_INCH);

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
    }
    positionMotor->stopMove();
    positionMotor->setCurrentPosition(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
    Serial.println("Position motor homed.");
    configurePositionMotorForNormalOperation();
}

void movePositionMotorToInitialAfterHoming() {
    if (positionMotor) {
        configurePositionMotorForNormalOperation();
        movePositionMotorToHome();
        while(positionMotor->isRunning()){
            checkCatcherServoEarlyActivation();
            checkCatcherClampEarlyActivation();
        }
    }
}

// Complex conditional logic
// Checks the cut motor homing switch multiple times and recalibrates if detected.
// Returns true if home detected and recalibrated, false otherwise.
bool checkAndRecalibrateCutMotorHome(int attempts) {
    if (!cutMotor) return false;

    bool sensorDetectedHome = false;
    for (int i = 0; i < attempts; i++) {
        cutHomingSwitch.update();
        Serial.print("Cut position switch read attempt "); Serial.print(i + 1); Serial.print(": "); Serial.println(cutHomingSwitch.read());
        if (cutHomingSwitch.read() == HIGH) {
            sensorDetectedHome = true;
            cutMotor->setCurrentPosition(0);
            Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
            break;
        }
    }
    return sensorDetectedHome;
}

//* ************************************************************************
//* ************************* SWITCH LOGIC FUNCTIONS ***********************
//* ************************************************************************

void handleReloadMode() {
    if (currentState == IDLE) {
        bool reloadSwitchOn = reloadSwitch.read() == HIGH;
        if (reloadSwitchOn && !isReloadMode) {
            isReloadMode = true;
            retractClamp(POSITION_CLAMP_TYPE);
            retractClamp(WOOD_SECURE_CLAMP_TYPE);
            turnYellowLedOn();
            Serial.println("Entered reload mode");
        } else if (!reloadSwitchOn && isReloadMode) {
            isReloadMode = false;
            extendClamp(POSITION_CLAMP_TYPE);
            extendClamp(WOOD_SECURE_CLAMP_TYPE);
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

bool shouldStartCycle() {
    // Condition from READY state to start a cycle
    return ((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress))
            && !woodSuctionError && startSwitchSafe);
}

// Early activation functions for catcher components
void checkCatcherServoEarlyActivation() {
    if (positionMotor) {
        float currentPositionInches = (float)positionMotor->getCurrentPosition() / STEPS_PER_INCH_POSITION;
        float targetPositionInches = POSITION_TRAVEL_DISTANCE;
        float earlyActivationPositionInches = targetPositionInches - CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;
        
        if (currentPositionInches >= earlyActivationPositionInches && !catcherServoIsActiveAndTiming) {
            catcherServo.write(CATCHER_SERVO_ACTIVE_POSITION);
            catcherServoActiveStartTime = millis();
            catcherServoIsActiveAndTiming = true;
            Serial.print("Catcher servo early activation at ");
            Serial.print(currentPositionInches);
            Serial.print(" inches (");
            Serial.print(CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES);
            Serial.println(" inches early)");
        }
    }
}

void checkCatcherClampEarlyActivation() {
    if (positionMotor) {
        float currentPositionInches = (float)positionMotor->getCurrentPosition() / STEPS_PER_INCH_POSITION;
        float targetPositionInches = POSITION_TRAVEL_DISTANCE;
        float earlyActivationPositionInches = targetPositionInches - CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;
        
        if (currentPositionInches >= earlyActivationPositionInches && !catcherClampIsEngaged) {
            extendClamp(CATCHER_CLAMP_TYPE);
            Serial.print("Catcher clamp early activation at ");
            Serial.print(currentPositionInches);
            Serial.print(" inches (");
            Serial.print(CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES);
            Serial.println(" inches early)");
        }
    }
}

// Catcher Servo Timing
void handleCatcherServoReturn() {
    // Move catcher servo to home position
    catcherServo.write(CATCHER_SERVO_HOME_POSITION);
    Serial.print("Catcher servo returned to home position (");
    Serial.print(CATCHER_SERVO_HOME_POSITION);
    Serial.println(" degrees).");
}

void handleTASignalTiming() { 
  if (signalTAActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
    digitalWrite(TA_SIGNAL_OUT_PIN, LOW); // Return to inactive state (LOW)
    signalTAActive = false;
    Serial.println("Signal to Transfer Arm (TA) completed"); 
  }
}

void handleCatcherClampDisengage() {
  if (catcherClampIsEngaged && (millis() - catcherClampEngageTime >= CATCHER_CLAMP_ENGAGE_DURATION_MS)) {
    retractClamp(CATCHER_CLAMP_TYPE);
    Serial.println("Catcher Clamp disengaged after 1 second.");
  }
}

void movePositionMotorToYesWoodHome() {
    if (positionMotor) {
        positionMotor->moveTo(0);
        Serial.println("Position motor moving to YES_WOOD home position (0 inches)");
    }
} 