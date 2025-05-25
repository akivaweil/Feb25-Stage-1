#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <esp_system.h>
#include <ESP32Servo.h>
#include "OTAUpdater/ota_updater.h"
#include "Functions.h"

//* ************************************************************************
//* ************************ AUTOMATED TABLE SAW **************************
//* ************************************************************************
// Main control system for Stage 1 of the automated table saw.
// Handles state machine logic, motor control, sensor monitoring, and safety systems.

// Motor Pin Definitions
const int CUT_MOTOR_PULSE_PIN = 12;
const int CUT_MOTOR_DIR_PIN = 11;
const int POSITION_MOTOR_PULSE_PIN = 17;
const int POSITION_MOTOR_DIR_PIN = 18;

// Servo Pin Definition
const int CATCHER_SERVO_PIN = 14;

// Servo Position Constants
const int CATCHER_SERVO_HOME_POSITION = 24;     // Home position (degrees)
const int CATCHER_SERVO_ACTIVE_POSITION = 90;   // Position when activated (degrees)

// Switch and Sensor Pin Definitions
const int CUT_MOTOR_HOMING_SWITCH = 3;
const int POSITION_MOTOR_HOMING_SWITCH = 16;
const int RELOAD_SWITCH = 6;
const int START_CYCLE_SWITCH = 5;
const int WOOD_SENSOR = 10;
const int WAS_WOOD_SUCTIONED_SENSOR = 39;
const int FIX_POSITION_BUTTON = 41;

// Clamp Pin Definitions
const int POSITION_CLAMP = 36;
const int WOOD_SECURE_CLAMP = 48;
const int CATCHER_CLAMP_PIN = 42;

// Signal pin definition for Transfer Arm
const int TA_SIGNAL_OUT_PIN = 8;

// LED Pin Definitions
const int RED_LED = 47;
const int YELLOW_LED = 21;
const int GREEN_LED = 37;
const int BLUE_LED = 19;

// Catcher Servo timing configuration
const unsigned long CATCHER_SERVO_ACTIVE_HOLD_DURATION_MS = 2000;
unsigned long catcherServoActiveStartTime = 0;
bool catcherServoIsActiveAndTiming = false;

// Catcher Clamp timing variables
const unsigned long CATCHER_CLAMP_ENGAGE_DURATION_MS = 1500; // 1 second
unsigned long catcherClampEngageTime = 0;
bool catcherClampIsEngaged = false;

// Wood Caught Error Check timing
const unsigned long WOOD_CAUGHT_CHECK_DELAY_MS = 1000; // 1 second delay to check if wood was caught

// SystemStates Enum is now in Functions.h
SystemState currentState = STARTUP;
SystemState previousState = ERROR_RESET; // Initialize to a different state to ensure first print

// Motor Configuration
const int CUT_MOTOR_STEPS_PER_INCH = 500;  // 4x increase from 38
const int POSITION_MOTOR_STEPS_PER_INCH = 1000; // Restored to original value
const float CUT_TRAVEL_DISTANCE = 9.0; // inches
const float POSITION_TRAVEL_DISTANCE = 3.4; // inches
const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES = 0.1; // Inches for incremental reverse
const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES = 0.4; // Max inches for incremental reverse before error
const int CUT_HOMING_DIRECTION = -1;
const int POSITION_HOMING_DIRECTION = -1;

// Speed and Acceleration Settings
// These values are critical for the machine's performance and safety.
// Adjust with caution and test thoroughly after any changes.

// ************************************************************************
// * CUT MOTOR SETTINGS                                                   *
// ************************************************************************
// ! Normal Cutting Operation (Cutting State)
const float CUT_MOTOR_NORMAL_SPEED = 700;      // Speed for the cutting pass (steps/sec)
const float CUT_MOTOR_NORMAL_ACCELERATION = 10000; // Acceleration for the cutting pass (steps/sec^2)

// ! Return Stroke (Returning State / End of Cutting State)
const float CUT_MOTOR_RETURN_SPEED = 20000;     // Speed for returning after a cut (steps/sec)
// Note: Return acceleration for cut motor typically uses CUT_MOTOR_NORMAL_ACCELERATION or a dedicated return acceleration if needed.
// For simplicity, we can assume it uses the same as normal acceleration or adjust if specific behavior is required.

// ! Homing Operation (Homing State)
const float CUT_MOTOR_HOMING_SPEED = 1000;      // Speed for homing the cut motor (steps/sec)
// Note: Homing acceleration for cut motor typically uses a gentler profile, often reusing CUT_MOTOR_NORMAL_ACCELERATION or a specific, lower value.

// ************************************************************************
// * POSITION MOTOR SETTINGS                                              *
// ************************************************************************
// ! Normal Positioning Operation (Positioning State / Parts of Cutting State)
const float POSITION_MOTOR_NORMAL_SPEED = 20000;    // Speed for normal positioning moves (steps/sec)
const float POSITION_MOTOR_NORMAL_ACCELERATION = 30000; // Acceleration for normal positioning (steps/sec^2)

// ! Return to Home/Start (Returning State / End of Cutting State / Homing after initial move)
const float POSITION_MOTOR_RETURN_SPEED = 30000;    // Speed for returning to home or start position (steps/sec)
const float POSITION_MOTOR_RETURN_ACCELERATION = 30000; // Acceleration for return moves (steps/sec^2)

// ! Homing Operation (Homing State)
const float POSITION_MOTOR_HOMING_SPEED = 2000;     // Speed for homing the position motor (steps/sec)
// Note: Homing acceleration for position motor, similar to cut motor, often reuses POSITION_MOTOR_NORMAL_ACCELERATION or a specific, lower value.

// Add a timeout constant for cut motor homing
const unsigned long CUT_HOME_TIMEOUT = 5000; // 5000 ms (5 seconds) timeout

// Create motor objects
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *cutMotor = NULL;
FastAccelStepper *positionMotor = NULL;

// Servo object
Servo catcherServo;

// Bounce objects for debouncing switches
Bounce cutHomingSwitch = Bounce();
Bounce positionHomingSwitch = Bounce();
Bounce reloadSwitch = Bounce();
Bounce startCycleSwitch = Bounce();
Bounce fixPositionButton = Bounce();

// System flags
bool isHomed = false;
bool isReloadMode = false;
bool woodPresent = false;
bool woodSuctionError = false;
bool wasWoodCaughtError = false; // New flag for wood caught error
bool errorAcknowledged = false;
bool cuttingCycleInProgress = false;
bool continuousModeActive = false;  // New flag for continuous operation
bool startSwitchSafe = false;       // New flag to track if start switch is safe
bool woodCaughtCheckPending = false; // Flag to indicate when a wood caught check is scheduled
unsigned long woodCaughtCheckTime = 0; // Time when the wood caught check should occur

// Timers for various operations
unsigned long lastBlinkTime = 0;
unsigned long lastErrorBlinkTime = 0;
unsigned long errorStartTime = 0;
unsigned long positionMoveStartTime = 0;

// LED states
bool blinkState = false;
bool errorBlinkState = false;

// Global variables for signal handling
unsigned long signalTAStartTime = 0; // For Transfer Arm signal
bool signalTAActive = false;      // For Transfer Arm signal
const unsigned long TA_SIGNAL_DURATION = 150; // Duration for TA signal

const float CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = 1.0; // New constant

// New flag to track cut motor return during yes-wood mode
bool cutMotorInYesWoodReturn = false;

// Global variable for FIX_POSITION state steps
int fixPositionStep = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Automated Table Saw Control System - Stage 1");
  
  setupOTA();

  //! Configure pin modes
  pinMode(CUT_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(CUT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_DIR_PIN, OUTPUT);
  
  pinMode(CUT_MOTOR_HOMING_SWITCH, INPUT_PULLDOWN);
  pinMode(POSITION_MOTOR_HOMING_SWITCH, INPUT_PULLDOWN);
  pinMode(RELOAD_SWITCH, INPUT_PULLDOWN);
  pinMode(START_CYCLE_SWITCH, INPUT_PULLDOWN);
  pinMode(FIX_POSITION_BUTTON, INPUT_PULLDOWN);
  
  pinMode(WOOD_SENSOR, INPUT_PULLUP);
  pinMode(WAS_WOOD_SUCTIONED_SENSOR, INPUT_PULLUP);
  
  pinMode(POSITION_CLAMP, OUTPUT);
  pinMode(WOOD_SECURE_CLAMP, OUTPUT);
  pinMode(CATCHER_CLAMP_PIN, OUTPUT);
  
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  
  pinMode(TA_SIGNAL_OUT_PIN, OUTPUT);
  digitalWrite(TA_SIGNAL_OUT_PIN, LOW);
  
  //! Initialize clamps and LEDs
  extendPositionClamp();
  extendWoodSecureClamp();
  retractCatcherClamp();
  allLedsOff();
  turnBlueLedOn();
  
  //! Configure switch debouncing
  cutHomingSwitch.attach(CUT_MOTOR_HOMING_SWITCH);
  cutHomingSwitch.interval(3);
  
  positionHomingSwitch.attach(POSITION_MOTOR_HOMING_SWITCH);
  positionHomingSwitch.interval(5);
  
  reloadSwitch.attach(RELOAD_SWITCH);
  reloadSwitch.interval(10);
  
  startCycleSwitch.attach(START_CYCLE_SWITCH);
  startCycleSwitch.interval(20);
  
  fixPositionButton.attach(FIX_POSITION_BUTTON);
  fixPositionButton.interval(20);
  
  //! Initialize motors
  engine.init();

  cutMotor = engine.stepperConnectToPin(CUT_MOTOR_PULSE_PIN);
  if (cutMotor) {
    cutMotor->setDirectionPin(CUT_MOTOR_DIR_PIN);
    configureCutMotorForCutting();
    cutMotor->setCurrentPosition(0);
  } else {
    Serial.println("Failed to init cutMotor");
  }

  positionMotor = engine.stepperConnectToPin(POSITION_MOTOR_PULSE_PIN);
  if (positionMotor) {
    positionMotor->setDirectionPin(POSITION_MOTOR_DIR_PIN);
    configurePositionMotorForNormalOperation();
    positionMotor->setCurrentPosition(0);
  } else {
    Serial.println("Failed to init positionMotor");
  }
  
  //! Initialize servo
  catcherServo.setTimerWidth(14);
  catcherServo.attach(CATCHER_SERVO_PIN);
  
  //! Configure initial state
  currentState = STARTUP;
  
  startCycleSwitch.update();
  if (startCycleSwitch.read() == HIGH) {
    startSwitchSafe = false;
  } else {
    startSwitchSafe = true;
  }
  
  delay(10);
}

void loop() {
  handleOTA(); // Handle OTA requests

  // Update all debounced switches first - moved to top of loop for earlier detection
  cutHomingSwitch.update();
  positionHomingSwitch.update();
  reloadSwitch.update();
  startCycleSwitch.update();
  fixPositionButton.update();
  
  // Check for cut motor hitting home sensor during yes-wood return
  if (cutMotorInYesWoodReturn && cutMotor && cutMotor->isRunning() && cutHomingSwitch.read() == HIGH) {
    Serial.println("Cut motor hit homing sensor during yes-wood return - stopping immediately!");
    cutMotor->forceStopAndNewPosition(0);  // Stop immediately and set position to 0
  }

  // Handle servo return after hold duration at active position AND when WAS_WOOD_SUCTIONED_SENSOR reads HIGH
  if (catcherServoIsActiveAndTiming && (millis() - catcherServoActiveStartTime >= CATCHER_SERVO_ACTIVE_HOLD_DURATION_MS)) {
      if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == HIGH) {
          catcherServoIsActiveAndTiming = false;
          handleCatcherServoReturn(); // Call function to return catcher servo to home position
          Serial.println("Servo timing completed AND WAS_WOOD_SUCTIONED_SENSOR is HIGH, returning catcher servo to home.");
      } else {
          // Only log this message periodically to avoid flooding the serial monitor
          static unsigned long lastLogTime = 0;
          if (millis() - lastLogTime >= 500) { // Log every 500ms
              Serial.println("Waiting for WAS_WOOD_SUCTIONED_SENSOR to read HIGH before returning catcher servo...");
              lastLogTime = millis();
          }
      }
  }

  // Check if wood was caught (if check is pending)
  checkWoodCaught();

  // Handle Catcher Clamp disengagement after 1 second
  if (catcherClampIsEngaged && (millis() - catcherClampEngageTime >= CATCHER_CLAMP_ENGAGE_DURATION_MS)) {
    retractCatcherClamp();
    Serial.println("Catcher Clamp disengaged after 1 second.");
  }

  if (currentState != previousState) {
    Serial.print("Current State: ");
    switch (currentState) {
      case STARTUP: Serial.println("STARTUP"); break;
      case HOMING: Serial.println("HOMING"); break;
      case READY: Serial.println("READY"); break;
      case CUTTING: Serial.println("CUTTING"); break;
      case RETURNING: Serial.println("RETURNING"); break;
      case POSITIONING: Serial.println("POSITIONING"); break;
      case ERROR: Serial.println("ERROR"); break;
      case ERROR_RESET: Serial.println("ERROR_RESET"); break;
      case SUCTION_ERROR_HOLD: Serial.println("SUCTION_ERROR_HOLD"); break;
      case WAS_WOOD_CAUGHT_ERROR: Serial.println("WAS_WOOD_CAUGHT_ERROR"); break; // Added for new state
      case FIX_POSITION: Serial.println("FIX_POSITION"); break;
      default: Serial.println("UNKNOWN"); break;
    }
    previousState = currentState;
  }

  // Read wood sensor (active LOW per explanation)
  woodPresent = (digitalRead(WOOD_SENSOR) == LOW);
  
  // Handle start switch safety check
  if (!startSwitchSafe && startCycleSwitch.fell()) {
    startSwitchSafe = true;
  }
  
  // Handle the reload switch state when in READY state
  if (currentState == READY) {
    // Check current state of reload switch (HIGH = ON with pull-down resistor)
    bool reloadSwitchOn = reloadSwitch.read() == HIGH;
    
    if (reloadSwitchOn && !isReloadMode) {
      // Enter reload mode
      isReloadMode = true;
      retractPositionClamp(); // Disengage position clamp
      retractWoodSecureClamp(); // Disengage wood secure clamp
      turnBlueLedOn();     // Turn on blue LED for reload mode
    } else if (!reloadSwitchOn && isReloadMode) {
      // Exit reload mode
      isReloadMode = false;
      extendPositionClamp();   // Re-engage position clamp
      extendWoodSecureClamp(); // Re-engage wood secure clamp
      turnBlueLedOff();       // Turn off blue LED
    }
  }
  
  // Handle error acknowledgment separately
  if (reloadSwitch.rose() && currentState == ERROR) {
    currentState = ERROR_RESET;
    errorAcknowledged = true;
  }
  
  // Check for continuous mode activation/deactivation - modified to include safety check
  bool startSwitchOn = startCycleSwitch.read() == HIGH;
  if (startSwitchOn != continuousModeActive && startSwitchSafe) {
    continuousModeActive = startSwitchOn;
    if (continuousModeActive) {
    }
  }
  
  // Handle signal timing independently of other operations
  if (signalTAActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
    digitalWrite(TA_SIGNAL_OUT_PIN, LOW); // Return to inactive state (LOW)
    signalTAActive = false;
    Serial.println("Signal to Stage 1 to TA completed");
  }
  
  // Check for fix position button press
  if (fixPositionButton.rose() && currentState == READY) {
    currentState = FIX_POSITION;
    fixPositionStep = 0; // Reset step
    allLedsOff();
    turnBlueLedOn(); // Indicate FIX_POSITION active
    Serial.println("FIX_POSITION button pressed in READY state. Starting cut motor home position verification.");
  }
  
  // State machine
  switch (currentState) {
    //* ************************************************************************
    //* **************************** STARTUP ***********************************
    //* ************************************************************************
    // Handles the initial startup state, transitioning to HOMING.
    // Step 1: Turn on the blue LED to indicate startup/homing.
    // Step 2: Transition to the HOMING state.
    case STARTUP:
      turnBlueLedOn();  // Blue LED on during startup/homing
      currentState = HOMING;
      break;
      
    //* ************************************************************************
    //* ***************************** HOMING ***********************************
    //* ************************************************************************
    // Handles the homing sequence for all motors.
    // Step 1: Blink blue LED to indicate homing in progress.
    // Step 2: Home the cut motor (blocking). If fails, it might retry or transition to ERROR (currently retries).
    // Step 3: If cut motor homed, home the position motor (blocking). Disengage position clamp before homing.
    // Step 4: If position motor homed, move position motor to its initial position after homing (blocking). Re-engage position clamp.
    // Step 5: If all homing and initial positioning are complete, set isHomed flag to true.
    // Step 6: Turn off blue LED, turn on green LED.
    // Step 7: Ensure servo is at 2 degrees.
    // Step 8: Transition to READY state.
    case HOMING:
      { // Scope for static variables from performHomingSequence
        static bool cutMotorHomed = false;
        static bool positionMotorHomed = false;
        static bool positionMotorMoved = false;
        static unsigned long blinkTimer = 0;

        if (millis() - blinkTimer > 500) {
          blinkState = !blinkState; 
          if (blinkState) turnBlueLedOn(); else turnBlueLedOff();
          blinkTimer = millis();
        }

        if (!cutMotorHomed) {
          Serial.println("Starting cut motor homing phase (blocking)...");
          homeCutMotorBlocking(cutHomingSwitch, CUT_HOME_TIMEOUT);
          if (cutMotor && cutMotor->getCurrentPosition() == 0) { // Check if homing was successful
          cutMotorHomed = true;
          } else {
            Serial.println("Cut motor homing failed or timed out. Retrying or error.");
          }
        } else if (!positionMotorHomed) {
          static bool positionHomingPhaseInitiated = false;
          if (!positionHomingPhaseInitiated) {
            Serial.println("Starting position motor homing phase (blocking)..."); 
            retractPositionClamp(); 
            Serial.println("Position clamp disengaged for homing."); 
            positionHomingPhaseInitiated = true;
          }
          homePositionMotorBlocking(positionHomingSwitch);
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
          isHomed = true; 

          turnBlueLedOff();
          turnGreenLedOn();

          // Set initial servo position via function call
          handleCatcherServoReturn();
          currentState = READY;
        }
      } 
      break;
      
    //* ************************************************************************
    //* ****************************** READY ***********************************
    //* ************************************************************************
    // Handles the ready state, awaiting user input or automatic cycle start.
    // If not in reload mode:
    //   Step 1: Turn on green LED to indicate system is ready.
    //   Step 2: Check for start cycle conditions:
    //           - Start switch just flipped ON (rising edge).
    //           - OR Continuous mode active AND not already in a cutting cycle.
    //           - AND Wood suction error is not present.
    //           - AND Start switch is safe to use (wasn't ON at startup or has been cycled).
    //   Step 3: If start conditions met:
    //           - Turn off green LED, turn on yellow LED.
    //           - Set cuttingCycleInProgress flag to true.
    //           - Transition to CUTTING state.
    //           - Configure cut motor for cutting speed.
    //           - Ensure position and wood secure clamps are engaged.
    //           - If no wood is detected, turn on blue LED for no-wood mode indication.
    case READY:
      if (!isReloadMode) {
        turnGreenLedOn();
        
        if (((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress)) 
            && !woodSuctionError) && startSwitchSafe) {
          turnGreenLedOff();
          turnYellowLedOn();
          turnBlueLedOff();
          
          cuttingCycleInProgress = true;
          currentState = CUTTING;
          configureCutMotorForCutting();
          
          extendPositionClamp();
          extendWoodSecureClamp();
          
          if (!woodPresent) {
            turnBlueLedOn();
          }
        }
      }
      break;
      
    //* ************************************************************************
    //* **************************** CUTTING ***********************************
    //* ************************************************************************
    // Handles the wood cutting operation.
    // This state manages a multi-step cutting process.
    // It includes logic for normal cutting, deciding between a Yes-Wood Sequence and a No-Wood Sequence, and error handling.
    // Step 0: Start cut motor movement towards the wood. Reset catcher clamp activation flag.
    // Step 1: Once cut motor reaches 1 inch:
    //          - Check WAS_WOOD_SUCTIONED_SENSOR.
    //          - If LOW (suction error): Stop motors, set LEDs for error, transition to SUCTION_ERROR_HOLD.
    //          - If HIGH (suction OK or no wood): Proceed to Step 2.
    // Step 2a: As cut motor approaches end of travel (offset by CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES):
    //          - Activate catcher clamp if not already activated this cycle.
    // Step 2b: When cut motor completes its full travel:
    //            - Send signal to TA (servo movement and digital signal).
    // Step 2c: Configure cut motor for return speed.
    // Step 2d: Check WOOD_SENSOR:
    //              - If no wood (HIGH): Initiate No-Wood Sequence (moves to Step 6).
    //              - If wood detected (LOW): Initiate Yes-Wood Sequence (simultaneous return, moves to Step 7).
    // Step 3: (Currently bypassed for normal wood path, logic moved or integrated elsewhere). Was intended for initial position motor move completion.
    // Step 4: (Logic moved to Step 7 for wood path). Was intended for position motor reaching home, checking cut motor homing switch, and moving position motor to final position.
    // Step 5: When position motor reaches its final position (POSITION_TRAVEL_DISTANCE) after a Yes-Wood Sequence:
    //          - Engage wood secure clamp.
    //          - Turn off yellow LED.
    //          - Reset cuttingCycleInProgress flag.
    //          - Transition to READY state.
    // Step 6: Handles the No-Wood Sequence. Position motor starts moving to home immediately.
    //          - Sub-step 6.0: Retract wood secure clamp, command position motor to home.
    //          - Sub-step 6.1 (new): After cut motor returns home, engage position clamp. Wait for cylinder.
    //          - Sub-step 6.2 (was 6.1): After position motor reaches home, disengage position clamp. Wait for cylinder.
    //          - Sub-step 6.3 (was 6.2): Move position motor to 2.0 inches.
    //          - Sub-step 6.4 (was 6.3): After position motor reaches 2.0 inches, engage position clamp. Wait for cylinder.
    //          - Sub-step 6.5 (was 6.4): Move position motor to home.
    //          - Sub-step 6.6 (was 6.5): After position motor reaches home, disengage position clamp. Wait for cylinder.
    //          - Sub-step 6.7 (was 6.6): Move position motor to final position (POSITION_TRAVEL_DISTANCE).
    //          - Sub-step 6.8 (was 6.7): After position motor reaches final position, check cut motor homing switch, disengage wood secure clamp, update LEDs, reset flags, transition to READY.
    // Step 7: Handles completion of Yes-Wood Sequence (initiated in Step 2 when wood is detected after cut).
    //          - This step activates after both cut and position motors (which were commanded to return home simultaneously) have stopped.
    //          - Sub-step 7.1: Retract position clamp.
    //          - Sub-step 7.2: Check cut motor homing switch. 
    //            - If detected: Recalibrate cut motor position to 0. Proceed to Sub-step 7.3.
    //            - If NOT detected: Transition to ERROR state (cut motor homing failure). Includes stopping motors, engaging clamps, setting error LEDs.
    //          - Sub-step 7.3: If home was successfully detected in Sub-step 7.2, retract wood secure clamp.
    //          - Sub-step 7.4: If home was successfully detected in Sub-step 7.2, configure position motor for normal operation. Command position motor to final travel position (POSITION_TRAVEL_DISTANCE). Transition to cuttingStep 5 (which waits for this move to complete).
    // If a home position error is detected at any point:
    //   - Blink red/yellow LEDs.
    //   - Stop motors.
    //   - Engage clamps.
    //   - Wait for reload switch to acknowledge, then transition to ERROR_RESET.
    case CUTTING:
      Serial.println("Current State: CUTTING");
      { // Scope for static variables from performCuttingOperation
        static int cuttingStep = 0; // Renamed from cuttingStage
        static unsigned long signalStartTime = 0; // From performCuttingOperation
        static bool signalActive = false;      // From performCuttingOperation
        static bool homePositionErrorDetected = false; // From performCuttingOperation
        static bool catcherClampActivatedThisCycle = false; // <<< NEW
        static float cutMotorIncrementalMoveTotalInches = 0.0; // For incremental homing
        static int cuttingSubStep7 = 0; // <<< NEW SUB-STEP FOR CUTTING STEP 7
        
        // Static variables from performNoWoodOperation, to be used within cuttingStep 6
        static int noWoodStep = 0; // Renamed from noWoodStage
        static unsigned long cylinderActionTime = 0;
        static bool waitingForCylinder = false;


        if (homePositionErrorDetected) {
          Serial.println("Home position error detected during cutting operation."); 
          if (millis() - lastErrorBlinkTime > 100) { 
            errorBlinkState = !errorBlinkState; // Assuming errorBlinkState and lastErrorBlinkTime are global
            if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
            if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
            lastErrorBlinkTime = millis();
          }
          
          if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
          if (positionMotor) positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());
          
          extendPositionClamp();
          extendWoodSecureClamp();
          
          if (reloadSwitch.rose()) { // Assuming reloadSwitch is global
            homePositionErrorDetected = false;
            currentState = ERROR_RESET;
            errorAcknowledged = true; // Assuming errorAcknowledged is global
            Serial.println("Home position error acknowledged by reload switch."); 
          }
        } else { // Moved the main switch block inside this else
            // Handle signal timing independently of motor movements (from performCuttingOperation)
            // Note: This was outside the switch in the original function, might need adjustment if `return` was hit.
            // For now, keeping it similar.
            if (signalActive && millis() - signalStartTime >= 2000) { // Original was 2000ms, matching it.
                signalActive = false;
            }

            switch (cuttingStep) { // Renamed from cuttingStage
            case 0: 
              Serial.println("Cutting Step 0: Starting cut motion."); 
              moveCutMotorToCut();
              catcherClampActivatedThisCycle = false; // Reset for this cut cycle
              cuttingStep = 1; // Renamed from cuttingStage
              break;
              
            case 1: 
              // Modified to use >= as per original file content for cutMotor->getCurrentPosition()
              if (cutMotor && cutMotor->getCurrentPosition() >= (1.0 * CUT_MOTOR_STEPS_PER_INCH)) {
                Serial.println("Cutting Step 1: Cut motor at >= 1 inch, checking wood suction.");
                if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == LOW) { // LOW means NO SUCTION (Error condition)
                  Serial.println("Wood suction error detected! Waiting for cycle switch OFF then ON to reset.");
                  // Stop motors
                  if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
                  if (positionMotor) positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());

                  cuttingCycleInProgress = false;
                  cuttingStep = 0; // Reset cutting step // Renamed from cuttingStage
                  noWoodStep = 0;  // Reset no wood step // Renamed from noWoodStage

                  // Set LEDs for suction error state
                  turnRedLedOn();
                  turnYellowLedOff();
                  turnGreenLedOff();
                  turnBlueLedOff();

                  currentState = SUCTION_ERROR_HOLD; // Transition to new error hold state
                  break; // Break from cuttingStep switch // Renamed from cuttingStage
                } else { // Sensor is LOW, suction is OK
                    cuttingStep = 2; // Renamed from cuttingStage
                    Serial.println("Cutting Step 1: Wood suction OK (or not present). Proceeding to step 2.");
                }
              }
              break;
              
            case 2: 
              // Early Catcher Clamp Activation
              if (!catcherClampActivatedThisCycle && cutMotor &&
                  cutMotor->getCurrentPosition() >= ((CUT_TRAVEL_DISTANCE - CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH)) {
                Serial.println("Cutting Step 2: Activating Catcher Clamp (early).");
                extendCatcherClamp(); // Engage Catcher Clamp (reversed logic: HIGH = engaged)
                catcherClampActivatedThisCycle = true;
              }

              // Check for full cut completion
              if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("Cutting Step 2: Cut fully complete."); 
                sendSignalToTA(); // Signal to Transfer Arm
                configureCutMotorForReturn();

                int sensorValue = digitalRead(WOOD_SENSOR); // Assuming WOOD_SENSOR is global
                bool noWoodDetected = (sensorValue == HIGH);
                
                if (noWoodDetected) {
                  Serial.println("Cutting Step 2: No wood detected after cut. Entering No-Wood Sequence (handled in cuttingStep 6).");
                  // Inlining performNoWoodOperation logic:
                  configureCutMotorForReturn();
                  moveCutMotorToHome();
                  configurePositionMotorForNormalOperation();
                  // End of inlined performNoWoodOperation setup part. Actual sequence is in case 6.
                  cuttingStep = 6; // Renamed from cuttingStage
                } else {
                  Serial.println("Cutting Step 2: Wood detected. Entering Yes-Wood Sequence (simultaneous return, handled in cuttingStep 7)."); 
                  configurePositionMotorForReturn();
                  
                  retractPositionClamp(); 
                  retractWoodSecureClamp(); 
                  Serial.println("Position and Wood Secure clamps disengaged for simultaneous return.");

                  // Set flag to indicate we're in yes-wood return mode - enable homing sensor check
                  cutMotorInYesWoodReturn = true;
                  moveCutMotorToHome();
                  movePositionMotorToYesWoodHome();  // Changed from movePositionMotorToHome()
                  
                  cuttingStep = 7; // Renamed from cuttingStage
                  cuttingSubStep7 = 0; // Reset sub-step for case 7
                }
              }
              break;
              
            case 3: 
              Serial.println("Cutting Step 3: (Should be bypassed for wood path) Initial position move complete.");
              if (positionMotor && !positionMotor->isRunning()) {
                retractPositionClamp();
                retractWoodSecureClamp();
                Serial.println("Position clamp and wood secure clamp disengaged.");

                configurePositionMotorForReturn();
                movePositionMotorToHome();
                Serial.println("Position motor moving to home (0).");
              }
              break;
              
            case 4: { 
              Serial.println("Cutting Step 4: (Logic moved to Step 7 for wood path) Position motor at home (0).");
              if (positionMotor && !positionMotor->isRunning()) {
                retractPositionClamp();
                Serial.println("Position clamp disengaged.");

                if (cutMotor && !cutMotor->isRunning()) {
                  Serial.println("Cut motor also at home. Checking cut motor position switch.");
                  bool sensorDetectedHome = false;
                  for (int i = 0; i < 3; i++) {
                    delay(30);
                    cutHomingSwitch.update(); // Assuming cutHomingSwitch is global
                    Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(cutHomingSwitch.read());
                    if (cutHomingSwitch.read() == HIGH) {
                      sensorDetectedHome = true;
                      if (cutMotor) cutMotor->setCurrentPosition(0); // Recalibrate to 0 when switch is hit
                      Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
                      break;
                    }
                  }
                  if (!sensorDetectedHome) {
                    Serial.println("ERROR: Cut motor position switch did not detect home after return attempt.");
                    if (cutMotorIncrementalMoveTotalInches < CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES) {
                      Serial.print("Attempting incremental move. Total moved: ");
                      Serial.print(cutMotorIncrementalMoveTotalInches);
                      Serial.println(" inches.");
                      cutMotor->move(-CUT_MOTOR_INCREMENTAL_MOVE_INCHES * CUT_MOTOR_STEPS_PER_INCH);
                      cutMotorIncrementalMoveTotalInches += CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
                      // Stay in cuttingStep 4 to re-check sensor after move
                    } else {
                      Serial.println("ERROR: Cut motor position switch did not detect home after MAX incremental moves!");
                      stopCutMotor();
                      stopPositionMotor();
                      extendPositionClamp();
                      turnRedLedOn();
                      turnYellowLedOff();
                      currentState = ERROR;
                      errorStartTime = millis(); 
                      cuttingStep = 0; 
                      cutMotorIncrementalMoveTotalInches = 0.0; // Reset for next attempt
                      Serial.println("Transitioning to ERROR state due to cut motor homing failure after cut.");
                    }
                  } else {
                    Serial.println("Cut motor position switch confirmed home. Moving position motor to final position.");
                    cutMotorIncrementalMoveTotalInches = 0.0; // Reset on success
                    movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
                    cuttingStep = 5; 
                  }
                }
              }
              break;
            }
              
            case 5: 
              if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("Cutting Step 5: Position motor at final position. Cycle complete."); 
                extendWoodSecureClamp(); 
                Serial.println("Wood secure clamp engaged."); 
                turnYellowLedOff();
                cuttingCycleInProgress = false; // Assuming cuttingCycleInProgress is global
                currentState = READY;
                cuttingStep = 0; // Renamed from cuttingStage
                Serial.println("Transitioning to READY state."); 
              }
              break;
              
            case 6: // This case now contains the inlined logic from performNoWoodOperation's sequence
              // The setup for no-wood (motor speeds, initial moveTo(0) for cutMotor) was done in case 2 when noWoodDetected.
              // Cut motor is returning home. Position motor is configured.
              // Position motor starts moving to home immediately in this sequence.

              if (noWoodStep == 0) { // First time entering this specific no-wood logic path
                Serial.println("No-Wood Step 6.0: Initiating position motor to home & retracting wood secure clamp.");
                retractWoodSecureClamp();
                if (positionMotor) {
                    if (positionMotor->getCurrentPosition() != 0 || positionMotor->isRunning()) { // Corrected condition
                        positionMotor->moveTo(0);
                        Serial.println("No-Wood Step 6.0: Position motor commanded to home.");
                    } else {
                        Serial.println("No-Wood Step 6.0: Position motor already at home.");
                    }
                }
                noWoodStep = 1; // Advance to the next part of the no-wood sequence logic
              }

              if (waitingForCylinder && (millis() - cylinderActionTime >= 150)) { // CYLINDER_ACTION_DELAY_MS
                waitingForCylinder = false;
                noWoodStep++; 
              }
              
              if (!waitingForCylinder) {
                switch (noWoodStep) { 
                  case 1: // New Step: Wait for cut motor, then engage position clamp
                    if (cutMotor && !cutMotor->isRunning()) {
                        Serial.println("No-Wood Step 6.1 (new): Cut motor returned home. Engaging position clamp.");
                        extendPositionClamp();
                        cylinderActionTime = millis();
                        waitingForCylinder = true; // Will cause noWoodStep to increment to 2 after delay
                    }
                    // If cut motor is still running, we wait in this step.
                    break;
                    
                  case 2: // Was original noWoodStep 1: wait for position motor, then retract position clamp
                    if (positionMotor && !positionMotor->isRunning()) {
                      Serial.println("No-Wood Step 6.2 (was 6.1): Position motor at home. Disengaging position clamp."); 
                      retractPositionClamp();
                      cylinderActionTime = millis();
                      waitingForCylinder = true; // Increments to 3
                    }
                    break;
                    
                  case 3: // Was original noWoodStep 2: move position motor to 2.0 inches
                    Serial.println("No-Wood Step 6.3 (was 6.2): Moving position motor to 2.0 inches."); 
                    configurePositionMotorForNormalOperation(); // Ensure correct config
                    movePositionMotorToPosition(2.0);
                    noWoodStep = 4; // Directly advance step here as it's a command
                    break;
                    
                  case 4: // Was original noWoodStep 3: wait for position motor at 2.0, engage position clamp
                    if (positionMotor && !positionMotor->isRunning()) {
                      Serial.println("No-Wood Step 6.4 (was 6.3): Position motor at 2.0 inches. Engaging position clamp."); 
                      extendPositionClamp();
                      cylinderActionTime = millis();
                      waitingForCylinder = true; // Increments to 5
                    }
                    break;
                    
                  case 5: // Was original noWoodStep 4: move position motor to home
                    Serial.println("No-Wood Step 6.5 (was 6.4): Moving position motor to home."); 
                    configurePositionMotorForNormalOperation();
                    movePositionMotorToHome();
                    noWoodStep = 6; // Directly advance step
                    break;
                    
                  case 6: // Was original noWoodStep 5: wait for position motor at home, retract position clamp
                    if (positionMotor && !positionMotor->isRunning()) {
                      Serial.println("No-Wood Step 6.6 (was 6.5): Position motor at home. Disengaging position clamp."); 
                      retractPositionClamp();
                      cylinderActionTime = millis();
                      waitingForCylinder = true; // Increments to 7
                    }
                    break;
                    
                  case 7: // Was original noWoodStep 6: move position motor to final position
                    Serial.println("No-Wood Step 6.7 (was 6.6): Moving position motor to final position (POSITION_TRAVEL_DISTANCE)."); 
                    configurePositionMotorForNormalOperation();
                    movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
                    noWoodStep = 8; // Directly advance step
                    break;
                    
                  case 8: // Was original noWoodStep 7: wait for motor, check cut home, cleanup
                    if (positionMotor && !positionMotor->isRunning()) {
                      Serial.println("No-Wood Step 6.8 (was 6.7): Position motor at final position."); 
                      bool sensorDetectedHome = false;
                      Serial.println("No-Wood Step 6.8: Checking cut motor position switch."); 
                      for (int i = 0; i < 3; i++) {
                        delay(30);  
                        cutHomingSwitch.update();
                        // Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(cutHomingSwitch.read()); // Keep for debugging if needed
                        if (cutHomingSwitch.read() == HIGH) {
                          sensorDetectedHome = true;
                          if (cutMotor) cutMotor->setCurrentPosition(0); 
                          Serial.println("Cut motor position switch detected HIGH during no-wood sequence completion."); 
                          break;  
                        }
                      }
                      if (!sensorDetectedHome) {
                        Serial.println("WARNING: Cut motor position switch did NOT detect home after no-wood sequence, but proceeding anyway."); 
                      }

                      retractWoodSecureClamp(); 
                      Serial.println("Wood secure clamp disengaged (final check in no-wood)."); 
                      turnYellowLedOff();
                      turnBlueLedOn(); 

                      noWoodStep = 0; 
                      waitingForCylinder = false;
                      cuttingCycleInProgress = false; 
                      currentState = READY;
                      continuousModeActive = false;
                      
                      // Check if cycle switch is currently ON - if yes, require cycling
                      if (startCycleSwitch.read() == HIGH) {
                        startSwitchSafe = false;
                        Serial.println("Cycle switch is still ON - must be cycled OFF then ON for next cycle.");
                      } else {
                        Serial.println("Cycle switch is OFF - ready for next cycle.");
                      }
                      
                      cuttingStep = 0; 
                      Serial.println("No-wood sequence complete. Transitioning to READY state. Continuous mode OFF."); 
                    }
                    break;
                }
              }
              break;

            case 7: 
              // This step handles the completion of the "Yes-Wood Sequence".
              // Position clamp engages as soon as position motor is home. Cut motor homing is checked after cut motor is home.
              switch (cuttingSubStep7) {
                case 0: // Wait for position motor to home
                  if (positionMotor && !positionMotor->isRunning()) {
                    Serial.println("Cutting Step 7.0: Position motor has returned home. Engaging position clamp.");
                    extendPositionClamp();
                    Serial.println("Position clamp engaged (position motor home).");
                    cuttingSubStep7 = 1;
                  }
                  break;

                case 1: // Wait for cut motor to home, then proceed with original logic
                  if (cutMotor && !cutMotor->isRunning()) {
                    Serial.println("Cutting Step 7.1: Cut motor has returned home.");
                    // Clear the yes-wood return flag since cut motor has stopped (position motor already stopped)
                    cutMotorInYesWoodReturn = false;

                    // Original logic: Check the cut motor homing switch. If not detected, transition to ERROR.
                    bool sensorDetectedHome = false;
                    Serial.println("Checking cut motor position switch after simultaneous return.");
                    for (int i = 0; i < 3; i++) { 
                      delay(30);  
                      cutHomingSwitch.update();
                      Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(cutHomingSwitch.read());
                      if (cutHomingSwitch.read() == HIGH) {
                        sensorDetectedHome = true;
                        if (cutMotor) cutMotor->setCurrentPosition(0); 
                        Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
                        break; 
                      }
                    }

                    if (!sensorDetectedHome) {
                      // Homing failed, transition to ERROR state.
                      Serial.println("ERROR: Cut motor position switch did not detect home after simultaneous return!");
                      stopCutMotor();
                      // positionMotor is already stopped
                      extendWoodSecureClamp(); 
                      turnRedLedOn();
                      turnYellowLedOff(); 
                      currentState = ERROR;
                      errorStartTime = millis(); 
                      cuttingStep = 0; // Reset main cutting step
                      cuttingSubStep7 = 0; // Reset sub-step
                    } else {
                      // Homing successful, proceed with next steps.
                      // Original logic: Retract the wood secure clamp.
                      retractWoodSecureClamp();
                      Serial.println("Wood secure clamp retracted after successful cut motor home detection.");

                      // Original logic: Move position motor to final travel position.
                      Serial.println("Cut motor position switch confirmed home. Proceeding to move position motor to final travel position.");
                      configurePositionMotorForNormalOperation();
                      movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
                      cuttingStep = 5; // Transition to original cuttingStep 5
                      cuttingSubStep7 = 0; // Reset sub-step
                    }
                  }
                  break;
              }
              break;
            } // End of cuttingStep switch // Renamed from cuttingStage
        } // End of else for homePositionErrorDetected
      } // End of scope for performCuttingOperation static variables
      break;
      
    //* ************************************************************************
    //* *************************** RETURNING **********************************
    //* ************************************************************************
    // Handles the motor return sequence after a cut (largely integrated into CUTTING).
    // This state is now mostly a placeholder as the return logic is integrated directly
    // within the CUTTING state's stages (Stage 2 for initiating return, Stage 6 for no-wood return, Stage 7 for wood return completion).
    // Step 1: Transition to READY state.
    // Step 2: Turn off yellow LED.
    // Step 3: Reset cuttingCycleInProgress flag.
    case RETURNING:
      // performReturnOperation(); // Will be replaced by inlined code
      // This function is no longer needed as the return operation is now handled
      // directly in the performCuttingOperation function (now inlined in case CUTTING)
      
      // Transition to READY state
      currentState = READY;
      turnYellowLedOff();
      cuttingCycleInProgress = false; // Assuming cuttingCycleInProgress is global
      break;
      
    //* ************************************************************************
    //* ************************** POSITIONING *********************************
    //* ************************************************************************
    // Handles the motor positioning sequence (largely integrated into CUTTING).
    // This state is now mostly a placeholder as the positioning logic is integrated directly
    // within the CUTTING state's stages (Stage 2, 5, 6, and 7 involve positioning operations).
    // Step 1: Transition to READY state.
    // Step 2: Turn off yellow LED.
    // Step 3: Reset cuttingCycleInProgress flag.
    case POSITIONING:
      // performPositioningOperation(); // Will be replaced by inlined code
      // This function is no longer needed as the positioning operation is now handled
      // directly in the performCuttingOperation function (now inlined in case CUTTING)
      
      // Transition to READY state
      currentState = READY;
      turnYellowLedOff();
      cuttingCycleInProgress = false; // Assuming cuttingCycleInProgress is global
      break;
      
    //* ************************************************************************
    //* ***************************** ERROR ************************************
    //* ************************************************************************
    // Handles system error states.
    // Step 1: Blink red and yellow LEDs to indicate an error.
    // Step 2: Ensure cut and position motors are stopped.
    // Step 3: Wait for the reload switch to be pressed (rising edge) to acknowledge the error.
    // Step 4: Once error is acknowledged, transition to ERROR_RESET state.
    case ERROR:
      // handleErrorState(); // Will be replaced by inlined code
      Serial.println("Entering handleErrorState (inlined)."); 
      // Blink error LEDs
      if (millis() - lastErrorBlinkTime > 250) { // Assuming lastErrorBlinkTime is global
        errorBlinkState = !errorBlinkState; // Assuming errorBlinkState is global
        if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        lastErrorBlinkTime = millis();
      }
      
      // Keep motors stopped
      stopCutMotor();
      stopPositionMotor();
      
      // Wait for reload switch to acknowledge error
      if (errorAcknowledged) { // Assuming errorAcknowledged and reloadSwitch are global
        currentState = ERROR_RESET;
        Serial.println("Error acknowledged in handleErrorState (inlined). Transitioning to ERROR_RESET."); 
      }
      break;
      
    //* ************************************************************************
    //* ************************** ERROR_RESET *********************************
    //* ************************************************************************
    // Handles the reset sequence after an error has been acknowledged.
    // Step 1: Turn off red and yellow error LEDs.
    // Step 2: Reset errorAcknowledged and woodSuctionError flags.
    // Step 3: Transition to STARTUP state to re-initialize the system (which will lead to HOMING).
    case ERROR_RESET:
      // resetFromError(); // Will be replaced by inlined code
      Serial.println("Entering resetFromError (inlined)."); 
      // Turn off error LEDs
      turnRedLedOff();
      turnYellowLedOff();
      
      // Reset flags
      errorAcknowledged = false; // Global
      woodSuctionError = false; // Global
      
      // Return to homing state to re-initialize
      currentState = STARTUP;
      Serial.println("Error reset, restarting system. Transitioning to STARTUP."); 
      break;

    //* ************************************************************************
    //* ********************* SUCTION ERROR HOLD *******************************
    //* ************************************************************************
    // Handles waiting for user to reset a wood suction error via cycle switch.
    // This state is entered from CUTTING (Step 1) if the WAS_WOOD_SUCTIONED_SENSOR indicates an error (LOW).
    // Step 1: Slowly blink the red LED (e.g., every 1.5 seconds).
    // Step 2: Ensure yellow, green, and blue LEDs are off.
    // Step 3: Monitor the start cycle switch.
    // Step 4: If the start cycle switch shows a rising edge (OFF to ON transition):
    //          - Print a message about resetting from suction error.
    //          - Turn off the red LED.
    //          - Set continuousModeActive to false.
    //          - Set startSwitchSafe to false (requires user to cycle switch again for a new start).
    //          - Transition to HOMING state to re-initialize the system.
    case SUCTION_ERROR_HOLD:
      { // Scope for static variables
        static unsigned long lastSuctionErrorBlinkTime = 0;
        static bool suctionErrorBlinkState = false;

        // Blink RED_LED every 2 seconds
        if (millis() - lastSuctionErrorBlinkTime >= 1500) {
          lastSuctionErrorBlinkTime = millis();
          suctionErrorBlinkState = !suctionErrorBlinkState;
          if(suctionErrorBlinkState) turnRedLedOn(); else turnRedLedOff();
        }
        // Ensure other LEDs are off
        turnYellowLedOff();
        turnGreenLedOff();
        turnBlueLedOff();

        if (startCycleSwitch.rose()) { // Check for start switch OFF to ON transition
          Serial.println("Start cycle switch toggled ON. Resetting from suction error. Transitioning to HOMING.");
          turnRedLedOff();   // Turn off error LED explicitly before changing state
          
          continuousModeActive = false; // Ensure continuous mode is off
          startSwitchSafe = false;      // Require user to cycle switch OFF then ON for a new actual start
          
          currentState = HOMING;        // Go to HOMING to re-initialize
        }
      }
      break;

    //* ************************************************************************
    //* ********************* WAS_WOOD_CAUGHT_ERROR ***************************
    //* ************************************************************************
    // Handles the case where the wood was not properly caught by the catcher.
    // Step 1: Blink red LED at a moderate pace (once per second).
    // Step 2: Ensure all other LEDs are off.
    // Step 3: Ensure position motor is stopped (cut motor continues to home).
    // Step 4: Wait for cycle switch to be pressed (rising edge) to acknowledge the error.
    // Step 5: When cycle switch is pressed, transition to HOMING state.
    case WAS_WOOD_CAUGHT_ERROR:
      { // Scope for static variables
        static unsigned long lastWoodCaughtErrorBlinkTime = 0;
        static bool woodCaughtErrorBlinkState = false;

        // Blink RED_LED once per second
        handleWoodCaughtErrorLedBlink(lastWoodCaughtErrorBlinkTime, woodCaughtErrorBlinkState);

        // Keep position motor stopped (cut motor continues to home)
        stopPositionMotor();
        
        // Check for cycle switch rising edge (OFF to ON transition)
        if (startCycleSwitch.rose()) { 
          Serial.println("Start cycle switch toggled ON. Resetting from wood caught error. Transitioning to HOMING.");
          turnRedLedOff();   // Turn off error LED explicitly before changing state
          
          // Clear error flags
          wasWoodCaughtError = false;
          
          // Reset mode flags
          continuousModeActive = false; // Ensure continuous mode is off
          startSwitchSafe = false;      // Require user to cycle switch OFF then ON for a new actual start
          
          currentState = HOMING;        // Go to HOMING to re-initialize
        }
        
        Serial.println("Waiting for cycle switch to acknowledge wood caught error.");
      }
      break;

    //* ************************************************************************
    //* ******************** CUT MOTOR POSITION FIX ****************************
    //* ************************************************************************
    // Handles manual cut motor home position verification and recalibration.
    // This is a maintenance function specifically for situations where the cut motor
    // may have lost its home position reference or the home sensor needs verification.
    // The position motor movements are only to return the system to operational state
    // after the cut motor position has been verified and recalibrated.
    // 
    // TRIGGERED BY: FIX_POSITION_BUTTON press while in READY state
    // PURPOSE: Verify cut motor can properly detect its home position and recalibrate to 0
    // 
    // Step 0: Extend position clamp, start 100ms timer.
    // Step 1: After 100ms, retract clamps, send both motors to home position.
    // Step 2: Verify cut motor home sensor detection (3-try verification) and recalibrate position to 0.
    //         If cut motor home sensor fails verification, transition to ERROR state.
    // Step 3: Move position motor to operational position, engage clamps, return to READY.
    case FIX_POSITION:
      { // Scope for static timer
        static unsigned long step0Timer_fixPosition = 0; 
        static int fixPositionSubStep2 = 0; // <<< NEW SUB-STEP FOR FIX_POSITION STEP 2
        // Serial.print("Current State: FIX_POSITION, Step: "); Serial.println(fixPositionStep);

        switch (fixPositionStep) {
          case 0: // Initial action: extend position clamp and wait 100ms
            Serial.println("FIX_POSITION Step 0: Extending position clamp.");
            extendPositionClamp();
            step0Timer_fixPosition = millis();
            fixPositionStep = 1;
            break;

          case 1: // Wait for 100ms, then start the "yes-wood like" homing sequence
            if (millis() - step0Timer_fixPosition >= 100) {
              Serial.println("FIX_POSITION Step 1: Position clamp extended. Starting cut motor home verification sequence.");
              retractPositionClamp(); 
              retractWoodSecureClamp(); 

              configurePositionMotorForReturn();
              configureCutMotorForReturn();      

              moveCutMotorToHome();
              movePositionMotorToYesWoodHome(); // Moves to 0
              cutMotorInYesWoodReturn = true; // Enable check for cut motor hitting home switch during move
              fixPositionStep = 2;
              fixPositionSubStep2 = 0; // Reset sub-step for case 2
            }
            break;

          case 2: // Wait for motors to reach home (position motor first, then cut)
            switch (fixPositionSubStep2) {
              case 0: // Wait for position motor to home
                if (positionMotor && !positionMotor->isRunning()) {
                  Serial.println("FIX_POSITION Step 2.0: Position motor at home. Engaging position clamp.");
                  extendPositionClamp();
                  fixPositionSubStep2 = 1;
                }
                break;
              
              case 1: // Wait for cut motor to home, then check sensor
                if (cutMotor && !cutMotor->isRunning()) {
                  Serial.println("FIX_POSITION Step 2.1: Cut motor at home. Starting slow recovery to verify home sensor.");
                  cutMotorInYesWoodReturn = false; // Disable special check now that motor has stopped

                  // Start slow recovery movement toward home
                  cutMotor->setSpeedInHz(CUT_MOTOR_HOMING_SPEED); // Use existing homing speed constant
                  cutMotor->setAcceleration(10000); // Moderate acceleration for controlled movement
                  cutMotor->runBackward(); // Move toward home
                  
                  unsigned long recoveryStartTime = millis();
                  bool homeFoundDuringRecovery = false;
                  const unsigned long RECOVERY_TIMEOUT_MS = 5000; // 5 second timeout
                  
                  Serial.print("FIX_POSITION: Slow recovery started at ");
                  Serial.print(CUT_MOTOR_HOMING_SPEED);
                  Serial.println(" steps/sec with 5-second timeout...");
                  
                  // Monitor for home sensor detection during recovery
                  while ((millis() - recoveryStartTime) < RECOVERY_TIMEOUT_MS) {
                    cutHomingSwitch.update();
                    
                    if (cutHomingSwitch.read() == HIGH) {
                      // Home sensor detected during recovery!
                      cutMotor->forceStopAndNewPosition(0);
                      homeFoundDuringRecovery = true;
                      
                      unsigned long recoveryDuration = millis() - recoveryStartTime;
                      Serial.print("SUCCESS: Cut motor home sensor detected during FIX_POSITION recovery after ");
                      Serial.print(recoveryDuration);
                      Serial.println(" ms. Position recalibrated to 0.");
                      break;
                    }
                    
                    delay(10); // Small delay to prevent excessive sensor polling
                  }
                  
                  if (!homeFoundDuringRecovery) {
                    // Recovery timeout - stop motor and transition to error
                    cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
                    Serial.println("FIX_POSITION Step 2.1 ERROR: Recovery timeout after 5 seconds. Cut motor home sensor NOT detected.");
                    stopCutMotor();
                    // positionMotor already stopped
                    allLedsOff();
                    turnRedLedOn();
                    turnYellowLedOn(); 
                    currentState = ERROR;
                    errorStartTime = millis(); 
                    fixPositionStep = 0; // Reset main fix position step
                    fixPositionSubStep2 = 0; // Reset sub-step
                  } else {
                    Serial.println("FIX_POSITION Step 2.1: Cut motor home confirmed via slow recovery. Moving position motor to final.");
                    retractWoodSecureClamp(); // Ensure wood secure clamp is retracted before final positioning
                    configurePositionMotorForNormalOperation();
                    movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
                    fixPositionStep = 3;
                    fixPositionSubStep2 = 0; // Reset sub-step
                  }
                }
                break;
            }
            break;

          case 3: // Wait for position motor to reach POSITION_TRAVEL_DISTANCE
            if (positionMotor && !positionMotor->isRunning()) {
              Serial.println("FIX_POSITION Step 3: Position motor at final travel distance. Engaging wood secure clamp.");
              extendWoodSecureClamp();
              Serial.println("CUT MOTOR POSITION FIX: Verification complete. Cut motor home position confirmed and recalibrated.");
              allLedsOff(); 
              turnGreenLedOn();
              currentState = READY;
              fixPositionStep = 0; // Reset for next time
            }
            break;
        }
      } // End scope for static timer
      break;
  }
  
  // Run the motors (non-blocking)
  // cutMotor.run(); // Not needed for FastAccelStepper
  // positionMotor.run(); // Not needed for FastAccelStepper
}
