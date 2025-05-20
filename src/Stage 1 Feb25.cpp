#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <esp_system.h> // Include for ESP.restart()
#include "OTAUpdater/ota_updater.h" // Include for OTA
// Remove WiFi and OTA libraries

// Motor Pin Definitions
#define CUT_MOTOR_PULSE_PIN 12
#define CUT_MOTOR_DIR_PIN 11
#define POSITION_MOTOR_PULSE_PIN 17
#define POSITION_MOTOR_DIR_PIN 18

// Switch and Sensor Pin Definitions
#define CUT_MOTOR_HOMING_SWITCH 3
#define POSITION_MOTOR_HOMING_SWITCH 16
#define RELOAD_SWITCH 6
#define START_CYCLE_SWITCH 5
#define WOOD_SENSOR 10
#define WAS_WOOD_SUCTIONED_SENSOR 39

// Clamp Pin Definitions
#define POSITION_CLAMP 36
#define WOOD_SECURE_CLAMP 35

// Add signal pin definition
#define STAGE2_SIGNAL_OUT_PIN 8  // Using pin 19 for direct signaling to Stage 1 to Stage 2 machine

// LED Pin Definitionss
#define RED_LED 47  // Error LED
#define YELLOW_LED 21 // Busy/Reload LED
#define GREEN_LED 37   // Ready LED
#define BLUE_LED 19    // Setup/No-Wood LED

// System States
enum SystemState {
  STARTUP,
  HOMING,
  READY,
  CUTTING,
  RETURNING,
  POSITIONING,
  ERROR,
  ERROR_RESET,
  SUCTION_ERROR_HOLD // New state for specific suction error
};

SystemState currentState = STARTUP;
SystemState previousState = ERROR_RESET; // Initialize to a different state to ensure first print

// Motor Configuration
const int CUT_MOTOR_STEPS_PER_INCH = 500;  // 4x increase from 38
const int POSITION_MOTOR_STEPS_PER_INCH = 1000; // Restored to original value
const float CUT_TRAVEL_DISTANCE = 7.35; // inches
const float POSITION_TRAVEL_DISTANCE = 3.45; // inches
const int CUT_HOMING_DIRECTION = -1;
const int POSITION_HOMING_DIRECTION = -1;

// Speed and Acceleration Settings
const float CUT_NORMAL_SPEED = 2000;  // 4x increase from 35
const float CUT_RETURN_SPEED = 20000;  // 4x increase from 750
const float CUT_ACCELERATION = 10000;  // 4x increase from 1000
const float CUT_HOMING_SPEED = 1000;  // 4x increase from 150
const float POSITION_NORMAL_SPEED = 20000; // Restored to original value
const float POSITION_RETURN_SPEED = 20000; // Restored to original value
const float POSITION_ACCELERATION = 30000; // Restored to original value
const float POSITION_HOMING_SPEED = 2000; // Restored to original value
const float POSITION_RETURN_ACCELERATION = 30000; // Restored to original value

// Add a timeout constant for cut motor homing
const unsigned long CUT_HOME_TIMEOUT = 5000; // 5000 ms (5 seconds) timeout

// Create motor objects
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *cutMotor = NULL;
FastAccelStepper *positionMotor = NULL;

// Bounce objects for debouncing switches
Bounce cutHomingSwitch = Bounce();
Bounce positionHomingSwitch = Bounce();
Bounce reloadSwitch = Bounce();
Bounce startCycleSwitch = Bounce();

// System flags
bool isHomed = false;
bool isReloadMode = false;
bool woodPresent = false;
bool woodSuctionError = false;
bool errorAcknowledged = false;
bool cuttingCycleInProgress = false;
bool continuousModeActive = false;  // New flag for continuous operation
bool startSwitchSafe = false;       // New flag to track if start switch is safe

// Timers for various operations
unsigned long lastBlinkTime = 0;
unsigned long lastErrorBlinkTime = 0;
unsigned long errorStartTime = 0;
unsigned long positionMoveStartTime = 0;

// LED states
bool blinkState = false;
bool errorBlinkState = false;

// Global variables for signal handling
unsigned long signalStage2StartTime = 0;
bool signalStage2Active = false;
const unsigned long STAGE2_SIGNAL_DURATION = 150; // 150 milliseconds signal duration

void sendSignalToStage2() {
  // Set the signal pin HIGH to trigger Stage 1 to Stage 2 machine (active HIGH)
  digitalWrite(STAGE2_SIGNAL_OUT_PIN, HIGH);
  signalStage2StartTime = millis();
  signalStage2Active = true;
  Serial.println("Signal sent to Stage 1 to Stage 2 machine");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Automated Table Saw Control System - Stage 1");
  
  setupOTA(); // Setup OTA

  // Configure motor pins
  pinMode(CUT_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(CUT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_DIR_PIN, OUTPUT);
  
  // Configure switch pins - matches the explanation
  pinMode(CUT_MOTOR_HOMING_SWITCH, INPUT_PULLDOWN);       // Homing switch
  pinMode(POSITION_MOTOR_HOMING_SWITCH, INPUT_PULLDOWN);  // Homing switch
  pinMode(RELOAD_SWITCH, INPUT_PULLUP);                   // Manual switch without pull-down
  pinMode(START_CYCLE_SWITCH, INPUT_PULLUP);              // Manual switch without pull-down
  
  // Configure sensor pins - make consistent with explanation
  pinMode(WOOD_SENSOR, INPUT_PULLUP);          // Active LOW (LOW = wood present)
  pinMode(WAS_WOOD_SUCTIONED_SENSOR, INPUT_PULLUP);          // Active LOW (LOW = wood present)
  
  // Configure clamp pins - LOW = engaged (extended), HIGH = disengaged (retracted)
  pinMode(POSITION_CLAMP, OUTPUT);
  pinMode(WOOD_SECURE_CLAMP, OUTPUT);
  digitalWrite(POSITION_CLAMP, LOW);  // Start with position clamp engaged
  digitalWrite(WOOD_SECURE_CLAMP, LOW); // Start with wood secure clamp engaged
  
  // Configure LED pins
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  
  // All LEDs off to start
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  
  // Turn on blue LED during startup
  digitalWrite(BLUE_LED, HIGH);
  
  // Set up debouncing for switches
  cutHomingSwitch.attach(CUT_MOTOR_HOMING_SWITCH);
  cutHomingSwitch.interval(20);  // 20ms debounce time
  
  positionHomingSwitch.attach(POSITION_MOTOR_HOMING_SWITCH);
  positionHomingSwitch.interval(20);
  
  reloadSwitch.attach(RELOAD_SWITCH);
  reloadSwitch.interval(20);
  
  startCycleSwitch.attach(START_CYCLE_SWITCH);
  startCycleSwitch.interval(20);
  
  // Configure motors
  engine.init(); // Initialize FastAccelStepper engine

  cutMotor = engine.stepperConnectToPin(CUT_MOTOR_PULSE_PIN);
  if (cutMotor) {
    cutMotor->setDirectionPin(CUT_MOTOR_DIR_PIN);
    cutMotor->setSpeedInHz((uint32_t)CUT_NORMAL_SPEED);
    cutMotor->setAcceleration((uint32_t)CUT_ACCELERATION);
    cutMotor->setCurrentPosition(0);
  } else {
    Serial.println("Failed to init cutMotor");
    // Potentially handle error, though user instructions limit this
  }

  positionMotor = engine.stepperConnectToPin(POSITION_MOTOR_PULSE_PIN);
  if (positionMotor) {
    positionMotor->setDirectionPin(POSITION_MOTOR_DIR_PIN);
    positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED);
    positionMotor->setAcceleration((uint32_t)POSITION_ACCELERATION);
    positionMotor->setCurrentPosition(0);
  } else {
    Serial.println("Failed to init positionMotor");
    // Potentially handle error
  }
  
  // Initially set motors to use position 0
  // cutMotor.setCurrentPosition(0); // Done above
  // positionMotor.setCurrentPosition(0); // Done above
  
  // Initialize signal pin
  pinMode(STAGE2_SIGNAL_OUT_PIN, OUTPUT);
  digitalWrite(STAGE2_SIGNAL_OUT_PIN, LOW); // Start with signal inactive (LOW)
  
  // Start in STARTUP state
  currentState = STARTUP;
  
  // Check if start switch is already ON at startup
  startCycleSwitch.update();
  if (startCycleSwitch.read() == HIGH) {
    startSwitchSafe = false;
    // Serial.println("WARNING: Start switch is ON during startup. Turn it OFF before operation.");
  } else {
    startSwitchSafe = true;
  }
  
  // Serial.println("System initialized, ready to begin homing sequence");
  delay(10);  // Brief delay before starting homing
}

void loop() {
  handleOTA(); // Handle OTA requests

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
      case SUCTION_ERROR_HOLD: Serial.println("SUCTION_ERROR_HOLD"); break; // Added for new state
      default: Serial.println("UNKNOWN"); break;
    }
    previousState = currentState;
  }

  // Update all debounced switches
  cutHomingSwitch.update();
  positionHomingSwitch.update();
  reloadSwitch.update();
  startCycleSwitch.update();
  
  // Read wood sensor (active LOW per explanation)
  woodPresent = (digitalRead(WOOD_SENSOR) == LOW);
  
  // Handle start switch safety check
  if (!startSwitchSafe && startCycleSwitch.fell()) {
    // Start switch was turned OFF after being ON during startup
    startSwitchSafe = true;
    // Serial.println("Start switch is now safe to use");
  }
  
  // Handle the reload switch state when in READY state
  if (currentState == READY) {
    // Check current state of reload switch (HIGH = ON with pull-down resistor)
    bool reloadSwitchOn = reloadSwitch.read() == HIGH;
    
    if (reloadSwitchOn && !isReloadMode) {
      // Enter reload mode
      isReloadMode = true;
      digitalWrite(POSITION_CLAMP, HIGH); // Disengage position clamp
      digitalWrite(WOOD_SECURE_CLAMP, HIGH); // Disengage wood secure clamp
      digitalWrite(YELLOW_LED, HIGH);     // Turn on yellow LED for reload mode
      // Serial.println("Entered reload mode");
    } else if (!reloadSwitchOn && isReloadMode) {
      // Exit reload mode
      isReloadMode = false;
      digitalWrite(POSITION_CLAMP, LOW);   // Re-engage position clamp
      digitalWrite(WOOD_SECURE_CLAMP, LOW); // Re-engage wood secure clamp
      digitalWrite(YELLOW_LED, LOW);       // Turn off yellow LED
      // Serial.println("Exited reload mode, ready for operation");
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
      // Serial.println("Continuous operation mode activated");
    } else {
      // Serial.println("Continuous operation mode deactivated");
    }
  }
  
  // Handle signal timing independently of other operations
  if (signalStage2Active && millis() - signalStage2StartTime >= STAGE2_SIGNAL_DURATION) {
    digitalWrite(STAGE2_SIGNAL_OUT_PIN, LOW); // Return to inactive state (LOW)
    signalStage2Active = false;
    Serial.println("Signal to Stage 1 to Stage 2 completed");
  }
  
  // State machine
  switch (currentState) {
    //* ************************************************************************
    //* **************************** STARTUP ***********************************
    //* ************************************************************************
    // Handles the initial startup state, transitioning to HOMING.
    case STARTUP:
      // Serial.println("Current State: STARTUP"); // Removed
      digitalWrite(BLUE_LED, HIGH);  // Blue LED on during startup/homing
      currentState = HOMING;
      break;
      
    //* ************************************************************************
    //* ***************************** HOMING ***********************************
    //* ************************************************************************
    // Handles the homing sequence for all motors.
    case HOMING:
      // Serial.println("Current State: HOMING"); // Removed
      { // Scope for static variables from performHomingSequence
        static bool cutMotorHomed = false;
        static bool positionMotorHomed = false;
        static bool positionMotorMoved = false;
        static unsigned long blinkTimer = 0;
        static unsigned long cutMotorHomingStartTime = 0; 
        // static bool cutMotorMovingAway = false; // No longer needed
        static bool cutMotorMovingToHome = false;
        // static bool positionMotorMovingAway = false; // No longer needed for this simplified logic
        static bool positionMotorMovingToHome = false; // Retain for moving towards switch
        static bool positionMotorMovingToInitial = false;

        if (millis() - blinkTimer > 500) {
          blinkState = !blinkState; 
          digitalWrite(BLUE_LED, blinkState);
          blinkTimer = millis();
        }

        if (!cutMotorHomed) {
          if (cutMotorHomingStartTime == 0) { 
              Serial.println("Starting cut motor homing phase...");
              cutMotorHomingStartTime = millis();
              cutMotorMovingToHome = false; // Ensure flag is reset at start of phase
          }

          if (cutHomingSwitch.read() == HIGH) {
            // Switch is active. Stop motor, set position, and consider homed.
            Serial.println("Cut motor switch is active. Setting as homed.");
            if (cutMotor) {
                cutMotor->stopMove();
                cutMotor->setCurrentPosition(0);
            }
            cutMotorHomed = true;
            cutMotorHomingStartTime = 0; // Reset timeout timer
            Serial.println("Cut motor homed successfully.");
            cutMotorMovingToHome = false; // Reset flag
          } else { // Switch is not active, move towards it
            if (!cutMotorMovingToHome) {
              Serial.println("Cut motor switch not active. Moving towards switch."); 
              if (cutMotor) {
                cutMotor->setSpeedInHz((uint32_t)CUT_HOMING_SPEED);
                cutMotor->moveTo(-40000); // Move in negative direction towards switch
              }
              cutMotorMovingToHome = true;
            }
            // Check if we hit the switch while moving towards it
            // This explicit check is removed as the next iteration will read the switch state at the top of this block.
            // if (cutHomingSwitch.read() == HIGH && cutMotorMovingToHome) { ... } 
            // This was effectively redundant with the main check at the start of if(cutHomingSwitch.read() == HIGH)
          }
          
          // Timeout check remains
          if (!cutMotorHomed && millis() - cutMotorHomingStartTime > CUT_HOME_TIMEOUT && cutMotorHomingStartTime != 0) { 
              Serial.println("Cut motor homing timeout!"); 
              if (cutMotor) cutMotor->stopMove();
              cutMotorHomingStartTime = 0; // Stop timeout check by resetting
              // Consider transitioning to ERROR state here if desired, or retrying homing for cut motor
              // For now, it will just proceed to position motor homing if timeout occurs and cutMotorHomed is still false.
              // To force re-homing or error, you'd set currentState = ERROR or reset cutMotorHomed = false and flags.
          }
        } else if (!positionMotorHomed) {
          // Print only once when starting this phase and disengage clamp
          static bool positionHomingPhaseInitiated = false;
          if (!positionHomingPhaseInitiated) {
            Serial.println("Starting position motor homing phase..."); 
            digitalWrite(POSITION_CLAMP, HIGH); // Disengage clamp for homing
            Serial.println("Position clamp disengaged for homing."); 
            positionHomingPhaseInitiated = true;
            positionMotorMovingToHome = false; // Ensure this is reset at the start of the phase
          }

          if (positionHomingSwitch.read() == HIGH) {
            // Switch is active. Stop motor, set position with offset, and consider homed.
            Serial.println("Position motor switch is active. Setting as homed with offset.");
            if (positionMotor) {
                positionMotor->stopMove();
                // Set current position to be slightly off the switch, as per original logic.
                positionMotor->setCurrentPosition(-1 * POSITION_MOTOR_STEPS_PER_INCH); 
            }
            positionMotorHomed = true;
            Serial.println("Position motor homed successfully.");
            if (positionMotor) positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED); // Prepare for next move (to initial position)
            positionMotorMovingToHome = false;    // Reset flag
            positionHomingPhaseInitiated = false; // Reset for next potential homing cycle
          } else { // Switch is not active, move towards it
            if (!positionMotorMovingToHome) { // If not already commanded to move
              Serial.println("Position motor switch not active. Moving towards switch."); 
              if (positionMotor) {
                positionMotor->setSpeedInHz((uint32_t)POSITION_HOMING_SPEED);
                positionMotor->moveTo(-10000 * POSITION_MOTOR_STEPS_PER_INCH); // Large move towards switch
              }
              positionMotorMovingToHome = true;
            }
            // The next loop iteration will check positionHomingSwitch.read() again if it's still moving.
          }
          // Note: No explicit timeout for position motor homing in this simplified version,
          // relying on it hitting the switch. Add if necessary.

        } else if (!positionMotorMoved) {
          if (!positionMotorMovingToInitial) {
            Serial.println("Moving position motor to initial position after homing."); 
            if (positionMotor) {
              positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED);
              positionMotor->moveTo(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
            }
            positionMotorMovingToInitial = true;
          }

          if (positionMotorMovingToInitial && positionMotor && !positionMotor->isRunning()) {
            digitalWrite(POSITION_CLAMP, LOW); 
            Serial.println("Position clamp re-engaged."); 
            positionMotorMoved = true;
            Serial.println("Position motor moved to initial position.");
            positionMotorMovingToInitial = false; // Reset flag
          }
        } else {
          Serial.println("Homing sequence complete. System ready."); 
          cutMotorHomed = false; // Reset for next homing cycle if needed
          positionMotorHomed = false;
          positionMotorMoved = false;
          cutMotorHomingStartTime = 0; 
          isHomed = true; 

          digitalWrite(BLUE_LED, LOW);
          digitalWrite(GREEN_LED, HIGH);

          currentState = READY;
        }
      } 
      break;
      
    //* ************************************************************************
    //* ****************************** READY ***********************************
    //* ************************************************************************
    // Handles the ready state, awaiting user input or automatic cycle start.
    case READY:
      Serial.println("Current State: READY");
      // System is ready for operation
      if (!isReloadMode) {
        // Solid green LED to indicate ready
        digitalWrite(GREEN_LED, HIGH);
        
        // Start a new cycle if:
        // 1. Start switch was just flipped ON (rising edge), OR
        // 2. Continuous mode is active AND we're not already in a cutting cycle
        // AND the start switch is safe to use
        if (((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress)) 
            && !woodSuctionError) && startSwitchSafe) {
          // Serial.println("Starting cutting cycle");
          // Turn off ready LED, turn on busy LED
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(YELLOW_LED, HIGH);
          digitalWrite(BLUE_LED, LOW);  // Ensure blue LED is off at start of cutting cycle
          
          // Set flag to indicate cycle in progress
          cuttingCycleInProgress = true;
          
          // Always enter cutting state, regardless of wood presence
          currentState = CUTTING;
          // Configure cut motor for cutting speed - change back to normal speed
          if (cutMotor) {
            cutMotor->setSpeedInHz((uint32_t)CUT_NORMAL_SPEED);  // Change from 2000 back to CUT_NORMAL_SPEED (100)
            cutMotor->setAcceleration((uint32_t)CUT_ACCELERATION);
          }
          // Ensure clamps are engaged
          digitalWrite(POSITION_CLAMP, LOW);
          digitalWrite(WOOD_SECURE_CLAMP, LOW);
          
          // Store wood presence for later use
          if (!woodPresent) {
            // Serial.println("No wood detected, will enter no-wood mode after cut");
            digitalWrite(BLUE_LED, HIGH); // Blue LED on for no-wood mode
          }
        }
      }
      break;
      
    //* ************************************************************************
    //* **************************** CUTTING ***********************************
    //* ************************************************************************
    // Handles the wood cutting operation.
    case CUTTING:
      Serial.println("Current State: CUTTING");
      { // Scope for static variables from performCuttingOperation
        static int cuttingStage = 0;
        static unsigned long signalStartTime = 0; // From performCuttingOperation
        static bool signalActive = false;      // From performCuttingOperation
        static bool homePositionErrorDetected = false; // From performCuttingOperation
        
        // Static variables from performNoWoodOperation, to be used within cuttingStage 6
        static int noWoodStage = 0;
        static unsigned long cylinderActionTime = 0;
        static bool waitingForCylinder = false;


        if (homePositionErrorDetected) {
          Serial.println("Home position error detected during cutting operation."); 
          if (millis() - lastErrorBlinkTime > 100) { 
            errorBlinkState = !errorBlinkState; // Assuming errorBlinkState and lastErrorBlinkTime are global
            digitalWrite(RED_LED, errorBlinkState);
            digitalWrite(YELLOW_LED, !errorBlinkState); 
            lastErrorBlinkTime = millis();
          }
          
          if (cutMotor) cutMotor->stopMove();
          if (positionMotor) positionMotor->stopMove();
          
          digitalWrite(POSITION_CLAMP, LOW);
          digitalWrite(WOOD_SECURE_CLAMP, LOW);
          
          if (reloadSwitch.rose()) { // Assuming reloadSwitch is global
            homePositionErrorDetected = false;
            currentState = ERROR_RESET;
            errorAcknowledged = true; // Assuming errorAcknowledged is global
            Serial.println("Home position error acknowledged by reload switch."); 
          }
          // return; // This would exit the loop iteration
        } else { // Moved the main switch block inside this else
            // Handle signal timing independently of motor movements (from performCuttingOperation)
            // Note: This was outside the switch in the original function, might need adjustment if `return` was hit.
            // For now, keeping it similar.
            if (signalActive && millis() - signalStartTime >= 2000) { // Original was 2000ms, matching it.
                signalActive = false;
            }

            switch (cuttingStage) {
            case 0: 
              Serial.println("Cutting Stage 0: Starting cut motion."); 
              if (cutMotor) cutMotor->moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
              cuttingStage = 1;
              break;
              
            case 1: 
              // Modified to use >= as per original file content for cutMotor->getCurrentPosition()
              if (cutMotor && cutMotor->getCurrentPosition() >= (1.0 * CUT_MOTOR_STEPS_PER_INCH)) {
                Serial.println("Cutting Stage 1: Cut motor at >= 1 inch, checking wood suction.");
                if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == LOW) { // LOW
                 means NO SUCTION (Error condition)
                  Serial.println("Wood suction error detected! Waiting for cycle switch OFF then ON to reset.");
                  // Stop motors
                  if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
                  if (positionMotor) positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());

                  cuttingCycleInProgress = false;
                  cuttingStage = 0; // Reset cutting stage
                  noWoodStage = 0;  // Reset no wood stage

                  // Set LEDs for suction error state
                  digitalWrite(RED_LED, HIGH);
                  digitalWrite(YELLOW_LED, LOW);
                  digitalWrite(GREEN_LED, LOW);
                  digitalWrite(BLUE_LED, LOW);

                  currentState = SUCTION_ERROR_HOLD; // Transition to new error hold state
                  break; // Break from cuttingStage switch
                } else { // Sensor is LOW, suction is OK
                    cuttingStage = 2;
                    Serial.println("Cutting Stage 1: Wood suction OK (or not present). Proceeding to stage 2.");
                }
              }
              break;
              
            case 2: 
              if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("Cutting Stage 2: Cut complete."); 
                sendSignalToStage2(); // Assuming sendSignalToStage2 is global

                if (cutMotor) cutMotor->setSpeedInHz((uint32_t)CUT_RETURN_SPEED);

                int sensorValue = digitalRead(WOOD_SENSOR); // Assuming WOOD_SENSOR is global
                bool noWoodDetected = (sensorValue == HIGH);
                
                if (noWoodDetected) {
                  Serial.println("Cutting Stage 2: No wood detected after cut. Entering no-wood operation (inlined)."); 
                  // Inlining performNoWoodOperation logic:
                  Serial.println("Entering performNoWoodOperation function (inlined).");
                  if (cutMotor) {
                    cutMotor->setSpeedInHz((uint32_t)CUT_RETURN_SPEED); // Already set above, but being explicit
                    cutMotor->moveTo(0); 
                  }
                  Serial.println("performNoWoodOperation (inlined): Cut motor set to return to home."); 
                  if (positionMotor) {
                    positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED);
                  }
                  Serial.println("performNoWoodOperation (inlined): Configuration complete. Sequence handled in cuttingStage 6.");
                  // End of inlined performNoWoodOperation setup part. Actual sequence is in case 6.
                  cuttingStage = 6; 
                } else {
                  Serial.println("Cutting Stage 2: Wood detected. Starting simultaneous return to home."); 
                  if (positionMotor) {
                    positionMotor->setSpeedInHz((uint32_t)POSITION_RETURN_SPEED);
                    positionMotor->setAcceleration((uint32_t)POSITION_RETURN_ACCELERATION);
                  }
                  
                  digitalWrite(POSITION_CLAMP, HIGH); 
                  digitalWrite(WOOD_SECURE_CLAMP, HIGH); 
                  Serial.println("Position and Wood Secure clamps disengaged for simultaneous return.");

                  if (cutMotor) cutMotor->moveTo(0);
                  if (positionMotor) positionMotor->moveTo(0);
                  
                  cuttingStage = 7; 
                }
              }
              break;
              
            case 3: 
              Serial.println("Cutting Stage 3: (Should be bypassed for wood path) Initial position move complete.");
              if (positionMotor && !positionMotor->isRunning()) {
                digitalWrite(POSITION_CLAMP, HIGH);
                digitalWrite(WOOD_SECURE_CLAMP, HIGH);
                Serial.println("Position clamp and wood secure clamp disengaged.");

                if (positionMotor) {
                  positionMotor->setAcceleration((uint32_t)POSITION_RETURN_ACCELERATION);
                  positionMotor->moveTo(0);
                }
                Serial.println("Position motor moving to home (0).");
              }
              break;
              
            case 4: { 
              Serial.println("Cutting Stage 4: (Logic moved to Stage 7 for wood path) Position motor at home (0).");
              if (positionMotor && !positionMotor->isRunning()) {
                digitalWrite(POSITION_CLAMP, LOW);
                Serial.println("Position clamp engaged.");

                if (cutMotor && !cutMotor->isRunning()) {
                  Serial.println("Cut motor also at home. Checking cut motor position switch.");
                  bool sensorDetectedHome = false;
                  for (int i = 0; i < 3; i++) {
                    delay(30);
                    cutHomingSwitch.update(); // Assuming cutHomingSwitch is global
                    Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(cutHomingSwitch.read());
                    if (cutHomingSwitch.read() == HIGH) {
                      sensorDetectedHome = true;
                      Serial.println("Cut motor position switch detected HIGH.");
                      break;
                    }
                  }
                  if (!sensorDetectedHome) {
                    Serial.println("ERROR: Cut motor position switch did not detect home after return!");
                    if (cutMotor) cutMotor->stopMove();
                    if (positionMotor) positionMotor->stopMove();
                    digitalWrite(POSITION_CLAMP, LOW);
                    digitalWrite(WOOD_SECURE_CLAMP, LOW);
                    digitalWrite(RED_LED, HIGH);
                    digitalWrite(YELLOW_LED, LOW);
                    currentState = ERROR;
                    errorStartTime = millis(); // Assuming errorStartTime is global
                    cuttingStage = 0;
                    Serial.println("Transitioning to ERROR state due to cut motor homing failure after cut.");
                  } else {
                    Serial.println("Cut motor position switch confirmed home. Moving position motor to final position.");
                    if (positionMotor) positionMotor->moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
                    cuttingStage = 5;
                  }
                }
              }
              break;
            }
              
            case 5: 
              if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("Cutting Stage 5: Position motor at final position. Cycle complete."); 
                digitalWrite(WOOD_SECURE_CLAMP, LOW); 
                Serial.println("Wood secure clamp engaged."); 
                digitalWrite(YELLOW_LED, LOW);
                cuttingCycleInProgress = false; // Assuming cuttingCycleInProgress is global
                currentState = READY;
                cuttingStage = 0; 
                Serial.println("Transitioning to READY state."); 
              }
              break;
              
            case 6: // This case now contains the inlined logic from performNoWoodOperation's sequence
              // The setup for no-wood (motor speeds, initial moveTo(0) for cutMotor) was done in case 2 when noWoodDetected.
              // Static variables for this no-wood sequence (noWoodStage, cylinderActionTime, waitingForCylinder)
              // are now declared at the beginning of the 'case CUTTING:' scope.

              if (waitingForCylinder && (millis() - cylinderActionTime >= 150)) {
                waitingForCylinder = false;
                noWoodStage++;
              }
              
              if (!waitingForCylinder) {
                switch (noWoodStage) {
                  case 0: 
                    if (cutMotor && !cutMotor->isRunning()) { // Check if cut motor has returned (started in case 2)
                      Serial.println("No-Wood Stage 0: Disengaging wood secure clamp, starting position motor to home."); 
                      digitalWrite(WOOD_SECURE_CLAMP, HIGH);
                      // Position motor was configured in case 2, now command it if cut motor is home.
                      // This ordering ensures cut motor has returned before position motor operations requiring clamp changes.
                      if (positionMotor) positionMotor->moveTo(0); 
                      // The second if for cutMotor in original performNoWoodOperation's noWoodStage 0:
                      // This part seems to be for after positionMotor->moveTo(0) is called and cutMotor is also home.
                      // Let's ensure cut motor is indeed home and then engage position clamp.
                      Serial.println("No-Wood Stage 0 (cont.): Cut motor returned home. Engaging position clamp."); 
                      digitalWrite(POSITION_CLAMP, LOW);
                      cylinderActionTime = millis();
                      waitingForCylinder = true;
                      // noWoodStage++; // This was missing, assuming it should go to next stage
                    }
                    // If cutMotor is still running, we wait.
                    break;
                    
                  case 1: 
                    if (positionMotor && !positionMotor->isRunning()) {
                      Serial.println("No-Wood Stage 1: Position motor at home. Disengaging position clamp."); 
                      digitalWrite(POSITION_CLAMP, HIGH);
                      cylinderActionTime = millis();
                      waitingForCylinder = true;
                      // noWoodStage++; // This was missing
                    }
                    break;
                    
                  case 2: 
                    Serial.println("No-Wood Stage 2: Moving position motor to 2.0 inches."); 
                    if (positionMotor) positionMotor->moveTo(2.0 * POSITION_MOTOR_STEPS_PER_INCH);
                    noWoodStage = 3; // Directly advance stage here as it's a command
                    break;
                    
                  case 3: 
                    if (positionMotor && !positionMotor->isRunning()) {
                      Serial.println("No-Wood Stage 3: Position motor at 2.0 inches. Engaging position clamp."); 
                      digitalWrite(POSITION_CLAMP, LOW);
                      cylinderActionTime = millis();
                      waitingForCylinder = true;
                      // noWoodStage++; // This was missing
                    }
                    break;
                    
                  case 4: 
                    Serial.println("No-Wood Stage 4: Moving position motor to home."); 
                    if (positionMotor) positionMotor->moveTo(0);
                    noWoodStage = 5; // Directly advance stage
                    break;
                    
                  case 5: 
                    if (positionMotor && !positionMotor->isRunning()) {
                      Serial.println("No-Wood Stage 5: Position motor at home. Disengaging position clamp."); 
                      digitalWrite(POSITION_CLAMP, HIGH);
                      cylinderActionTime = millis();
                      waitingForCylinder = true;
                      // noWoodStage++; // This was missing
                    }
                    break;
                    
                  case 6: 
                    Serial.println("No-Wood Stage 6: Moving position motor to 3.45 inches (final position)."); 
                    if (positionMotor) positionMotor->moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
                    noWoodStage = 7; // Directly advance stage
                    break;
                    
                  case 7: 
                    if (positionMotor && !positionMotor->isRunning()) {
                      Serial.println("No-Wood Stage 7: Position motor at final position."); 
                      bool sensorDetectedHome = false;
                      Serial.println("No-Wood Stage 7: Checking cut motor position switch."); 
                      for (int i = 0; i < 3; i++) {
                        delay(30);  
                        cutHomingSwitch.update();
                        Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(cutHomingSwitch.read()); 
                        if (cutHomingSwitch.read() == HIGH) {
                          sensorDetectedHome = true;
                          Serial.println("Cut motor position switch detected HIGH during no-wood sequence completion."); 
                          break;  
                        }
                      }
                      if (!sensorDetectedHome) {
                        Serial.println("WARNING: Cut motor position switch did NOT detect home after no-wood sequence, but proceeding anyway."); 
                      }

                      digitalWrite(WOOD_SECURE_CLAMP, HIGH); 
                      Serial.println("Wood secure clamp disengaged (final check in no-wood)."); 
                      digitalWrite(YELLOW_LED, LOW);
                      digitalWrite(BLUE_LED, HIGH); 

                      noWoodStage = 0;
                      waitingForCylinder = false;
                      cuttingCycleInProgress = false; // Global
                      currentState = READY;
                      continuousModeActive = false; // Global
                      startSwitchSafe = false;      // Global
                      cuttingStage = 0; // Reset main cutting stage
                      Serial.println("No-wood sequence complete. Transitioning to READY state. Continuous mode OFF. Start switch needs cycling."); 
                    }
                    break;
                }
              }
              break;

            case 7: 
              if (cutMotor && !cutMotor->isRunning() && positionMotor && !positionMotor->isRunning()) {
                Serial.println("Cutting Stage 7: Both cut and position motors at home.");
                digitalWrite(POSITION_CLAMP, LOW); 
                Serial.println("Position clamp engaged.");

                bool sensorDetectedHome = false;
                Serial.println("Checking cut motor position switch after simultaneous return.");
                for (int i = 0; i < 3; i++) {
                  delay(30);  
                  cutHomingSwitch.update();
                  Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(cutHomingSwitch.read());
                  if (cutHomingSwitch.read() == HIGH) {
                    sensorDetectedHome = true;
                    Serial.println("Cut motor position switch detected HIGH.");
                    break;
                  }
                }

                if (!sensorDetectedHome) {
                  Serial.println("ERROR: Cut motor position switch did not detect home after simultaneous return!");
                  if (cutMotor) cutMotor->stopMove();
                  if (positionMotor) positionMotor->stopMove();
                  digitalWrite(POSITION_CLAMP, LOW);
                  digitalWrite(WOOD_SECURE_CLAMP, LOW);
                  digitalWrite(RED_LED, HIGH);
                  digitalWrite(YELLOW_LED, LOW); 
                  currentState = ERROR;
                  errorStartTime = millis(); // Global
                  cuttingStage = 0; 
                  Serial.println("Transitioning to ERROR state due to cut motor homing failure post-simultaneous return.");
                } else {
                  Serial.println("Cut motor position switch confirmed home. Moving position motor to final travel position.");
                  if (positionMotor) {
                    positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED);
                    positionMotor->setAcceleration((uint32_t)POSITION_ACCELERATION);
                    positionMotor->moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
                  }
                  cuttingStage = 5; 
                }
              }
              break;
            } // End of cuttingStage switch
        } // End of else for homePositionErrorDetected
      } // End of scope for performCuttingOperation static variables
      break;
      
    //* ************************************************************************
    //* *************************** RETURNING **********************************
    //* ************************************************************************
    // Handles the motor return sequence after a cut (largely integrated into CUTTING).
    case RETURNING:
      Serial.println("Current State: RETURNING");
      // performReturnOperation(); // Will be replaced by inlined code
      // This function is no longer needed as the return operation is now handled
      // directly in the performCuttingOperation function (now inlined in case CUTTING)
      
      // Transition to READY state
      currentState = READY;
      digitalWrite(YELLOW_LED, LOW);
      cuttingCycleInProgress = false; // Assuming cuttingCycleInProgress is global
      break;
      
    //* ************************************************************************
    //* ************************** POSITIONING *********************************
    //* ************************************************************************
    // Handles the motor positioning sequence (largely integrated into CUTTING).
    case POSITIONING:
      Serial.println("Current State: POSITIONING");
      // performPositioningOperation(); // Will be replaced by inlined code
      // This function is no longer needed as the positioning operation is now handled
      // directly in the performCuttingOperation function (now inlined in case CUTTING)
      
      // Transition to READY state
      currentState = READY;
      digitalWrite(YELLOW_LED, LOW);
      cuttingCycleInProgress = false; // Assuming cuttingCycleInProgress is global
      break;
      
    //* ************************************************************************
    //* ***************************** ERROR ************************************
    //* ************************************************************************
    // Handles system error states.
    case ERROR:
      Serial.println("Current State: ERROR");
      // handleErrorState(); // Will be replaced by inlined code
      Serial.println("Entering handleErrorState (inlined)."); 
      // Blink error LEDs
      if (millis() - lastErrorBlinkTime > 250) { // Assuming lastErrorBlinkTime is global
        errorBlinkState = !errorBlinkState; // Assuming errorBlinkState is global
        digitalWrite(RED_LED, errorBlinkState);
        digitalWrite(YELLOW_LED, !errorBlinkState); 
        lastErrorBlinkTime = millis();
      }
      
      // Keep motors stopped
      if (cutMotor) cutMotor->stopMove();
      if (positionMotor) positionMotor->stopMove();
      
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
    case ERROR_RESET:
      Serial.println("Current State: ERROR_RESET");
      // resetFromError(); // Will be replaced by inlined code
      Serial.println("Entering resetFromError (inlined)."); 
      // Turn off error LEDs
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      
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
    case SUCTION_ERROR_HOLD:
      // Keep RED LED on to indicate error
      digitalWrite(RED_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, LOW);

      if (startCycleSwitch.rose()) { // Check for start switch OFF to ON transition
        Serial.println("Start cycle switch toggled ON. Resetting from suction error. Transitioning to HOMING.");
        digitalWrite(RED_LED, LOW);   // Turn off error LED
        
        continuousModeActive = false; // Ensure continuous mode is off
        startSwitchSafe = false;      // Require user to cycle switch OFF then ON for a new actual start
        
        currentState = HOMING;        // Go to HOMING to re-initialize
      }
      break;
  }
  
  // Run the motors (non-blocking)
  // cutMotor.run(); // Not needed for FastAccelStepper
  // positionMotor.run(); // Not needed for FastAccelStepper
}
