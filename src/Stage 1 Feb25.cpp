#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <esp_system.h> // Include for ESP.restart()
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
#define WAS_WOOD_SUCTIONED_SENSOR 37

// Clamp Pin Definitions
#define POSITION_CLAMP 36
#define WOOD_SECURE_CLAMP 35

// Add signal pin definition
#define STAGE2_SIGNAL_OUT_PIN 19  // Using pin 19 for direct signaling to Stage 1 to Stage 2 machine

// LED Pin Definitionss
#define RED_LED 45  // Error LED
#define YELLOW_LED 48 // Busy/Reload LED
#define GREEN_LED 47   // Ready LED
#define BLUE_LED 21    // Setup/No-Wood LED

// System States
enum SystemState {
  STARTUP,
  HOMING,
  READY,
  CUTTING,
  RETURNING,
  POSITIONING,
  ERROR,
  ERROR_RESET
};

SystemState currentState = STARTUP;

// Motor Configuration
const int CUT_MOTOR_STEPS_PER_INCH = 1000;  // 4x increase from 38
const int POSITION_MOTOR_STEPS_PER_INCH = 1000; // Restored to original value
const float CUT_TRAVEL_DISTANCE = 7.35; // inches
const float POSITION_TRAVEL_DISTANCE = 3.45; // inches
const int CUT_HOMING_DIRECTION = -1;
const int POSITION_HOMING_DIRECTION = -1;

// Speed and Acceleration Settings
const float CUT_NORMAL_SPEED = 150;  // 4x increase from 35
const float CUT_RETURN_SPEED = 3000;  // 4x increase from 750
const float CUT_ACCELERATION = 4000;  // 4x increase from 1000
const float CUT_HOMING_SPEED = 600;  // 4x increase from 150
const float POSITION_NORMAL_SPEED = 1000; // Restored to original value
const float POSITION_RETURN_SPEED = 10000; // Restored to original value
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

// Add these function declarations before the setup() function
void performHomingSequence();
void performNoWoodOperation();
void performCuttingOperation();
void performReturnOperation();
void performPositioningOperation();
void handleErrorState();
void resetFromError();

void setup() {
  Serial.begin(115200);
  Serial.println("Automated Table Saw Control System - Stage 1");
  
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
    case STARTUP:
      // Serial.println("Starting homing sequence...");
      digitalWrite(BLUE_LED, HIGH);  // Blue LED on during startup/homing
      currentState = HOMING;
      break;
      
    case HOMING:
      performHomingSequence();
      break;
      
    case READY:
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
      
    case CUTTING:
      performCuttingOperation();
      break;
      
    case RETURNING:
      performReturnOperation();
      break;
      
    case POSITIONING:
      performPositioningOperation();
      break;
      
    case ERROR:
      handleErrorState();
      break;
      
    case ERROR_RESET:
      resetFromError();
      break;
  }
  
  // Run the motors (non-blocking)
  // cutMotor.run(); // Not needed for FastAccelStepper
  // positionMotor.run(); // Not needed for FastAccelStepper
}

void performHomingSequence() {
  static bool cutMotorHomed = false;
  static bool positionMotorHomed = false;
  static bool positionMotorMoved = false;
  static unsigned long blinkTimer = 0;
  static unsigned long cutMotorHomingStartTime = 0; // Timer for cut motor homing timeout

  Serial.println("Entering performHomingSequence"); // Added

  // Blink blue LED to indicate homing
  if (millis() - blinkTimer > 500) {
    blinkState = !blinkState;
    digitalWrite(BLUE_LED, blinkState);
    blinkTimer = millis();
  }

  // Home cut motor first
  if (!cutMotorHomed) {
    Serial.println("Homing cut motor..."); // Added
    if (cutMotorHomingStartTime == 0) { // Initialize timer only once
        cutMotorHomingStartTime = millis();
    }

    // If the cut motor position switch is already active, move away first
    if (cutHomingSwitch.read() == HIGH) {
      Serial.println("Cut motor position switch is active, moving away."); // Added
      if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)CUT_HOMING_SPEED);
        // Always move in positive direction to move away from switch
        cutMotor->moveTo(40); // Move in positive direction
      }
      if (cutMotor && !cutMotor->isRunning()) {
        Serial.println("Moved away from cut motor switch, now moving towards it."); // Added
        // Now move back to find the switch - always in negative direction
        if (cutMotor) {
          cutMotor->setSpeedInHz((uint32_t)CUT_HOMING_SPEED);
          // Ensure we're moving in the negative direction
          cutMotor->moveTo(-40000); // Move in negative direction
        }
      }
    } else {
      Serial.println("Cut motor position switch is not active, moving towards it."); // Added
      // Move toward home switch - always in negative direction
      if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)CUT_HOMING_SPEED);
        // Ensure we're moving in the negative direction
        cutMotor->moveTo(-40000); // Move in negative direction
      }
    }

    // Check if we hit the switch
    if (cutHomingSwitch.read() == HIGH) {
      if (cutMotor) cutMotor->stopMove();
      if (cutMotor) cutMotor->setCurrentPosition(0);
      cutMotorHomed = true;
      cutMotorHomingStartTime = 0; // Reset timer
      Serial.println("Cut motor homed successfully");
    } else if (millis() - cutMotorHomingStartTime > CUT_HOME_TIMEOUT) { // Check for timeout
        Serial.println("Cut motor homing timeout!"); // Added
        // Handle timeout: go to ERROR state or retry, etc.
        // For now, just print an error and stop.
        if (cutMotor) cutMotor->stopMove();
        // Potentially set an error flag or change state here
        // For simplicity in this example, we'll just prevent further homing actions.
        // You might want to transition to an ERROR state.
        // currentState = ERROR; // Example: Transition to error state
        // errorStartTime = millis();
        return; // Exit homing or handle error
    }
  } else if (!positionMotorHomed) {
    Serial.println("Homing position motor..."); // Added
    // Home position motor
    digitalWrite(POSITION_CLAMP, HIGH); // Disengage position clamp for homing
    Serial.println("Position clamp disengaged for homing."); // Added

    // If the position motor switch is already active, move away first
    if (positionHomingSwitch.read() == HIGH) {
      Serial.println("Position motor switch is active, moving away."); // Added
      if (positionMotor) {
        positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED);
        // Always move in positive direction to move away from switch
        positionMotor->moveTo(100 * POSITION_MOTOR_STEPS_PER_INCH); // Move in positive direction
      }
      if (positionMotor && !positionMotor->isRunning()) {
        Serial.println("Moved away from position motor switch, now moving towards it."); // Added
        // Now move back to find the switch - always in negative direction
        if (positionMotor) {
          positionMotor->setSpeedInHz((uint32_t)POSITION_HOMING_SPEED);
          // Ensure we're moving in the negative direction
          positionMotor->moveTo(-10000 * POSITION_MOTOR_STEPS_PER_INCH); // Move in negative direction
        }
      }
    } else {
      Serial.println("Position motor switch is not active, moving towards it."); // Added
      // Move toward home switch - always in negative direction  
      if (positionMotor) {
        positionMotor->setSpeedInHz((uint32_t)POSITION_HOMING_SPEED);
        // Ensure we're moving in the negative direction
        positionMotor->moveTo(-10000 * POSITION_MOTOR_STEPS_PER_INCH); // Move in negative direction
      }
    }

    // Check if we hit the switch
    if (positionHomingSwitch.read() == HIGH) {
      if (positionMotor) positionMotor->stopMove();
      // Set current position to -1 inch
      if (positionMotor) positionMotor->setCurrentPosition(-1 * POSITION_MOTOR_STEPS_PER_INCH);
      positionMotorHomed = true;
      Serial.println("Position motor homed successfully");

      // Set back to normal speed after homing is complete
      if (positionMotor) positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED);
    }
  } else if (!positionMotorMoved) {
    Serial.println("Moving position motor to initial position..."); // Added
    // Move position motor to full travel distance
    if (positionMotor) {
      positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED);
      positionMotor->moveTo(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
    }

    if (positionMotor && !positionMotor->isRunning()) {
      digitalWrite(POSITION_CLAMP, LOW); // Re-engage position clamp
      Serial.println("Position clamp re-engaged."); // Added
      positionMotorMoved = true;
      Serial.println("Position motor moved to initial position");
    }
  } else {
    // Homing complete
    Serial.println("Homing sequence complete."); // Added
    cutMotorHomed = false;
    positionMotorHomed = false;
    positionMotorMoved = false;
    isHomed = true;
    cutMotorHomingStartTime = 0; // Reset timer just in case

    // Turn off homing LED, turn on ready LED
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);

    currentState = READY;
    Serial.println("Homing complete, system ready");
  }
}

void performCuttingOperation() {
  static int cuttingStage = 0;
  static unsigned long signalStartTime = 0;
  static bool signalActive = false;
  static bool homePositionErrorDetected = false;
  
  // If home position error is detected, just blink LEDs and stay in this state
  // Do not proceed with any other operations
  if (homePositionErrorDetected) {
    Serial.println("Home position error detected during cutting operation."); // Added
    // Rapid blinking of red and yellow LEDs
    if (millis() - lastErrorBlinkTime > 100) { // Fast 100ms blink rate
      errorBlinkState = !errorBlinkState;
      digitalWrite(RED_LED, errorBlinkState);
      digitalWrite(YELLOW_LED, !errorBlinkState); // Alternate LEDs for better visibility
      lastErrorBlinkTime = millis();
    }
    
    // Stop all motors and do not proceed
    if (cutMotor) cutMotor->stopMove();
    if (positionMotor) positionMotor->stopMove();
    
    // Keep clamps engaged for safety
    digitalWrite(POSITION_CLAMP, LOW);
    digitalWrite(WOOD_SECURE_CLAMP, LOW);
    
    // Check if reload switch is pressed to acknowledge error
    if (reloadSwitch.rose()) {
      homePositionErrorDetected = false;
      currentState = ERROR_RESET;
      errorAcknowledged = true;
      Serial.println("Home position error acknowledged by reload switch."); // Added
    }
    
    // Do not return from this function - system is in error state
    return;
  }
  
  // Handle signal timing independently of motor movements
  if (signalActive && millis() - signalStartTime >= 2000) {
    signalActive = false;
  }
  
  switch (cuttingStage) {
    case 0: // Start cut motion without checking wood suction sensor
      Serial.println("Cutting Stage 0: Starting cut motion."); // Added
      if (cutMotor) cutMotor->moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
      cuttingStage = 1;
      break;
      
    case 1: // Check for wood suction when saw is 1 inch into the cut
      if (cutMotor && cutMotor->getCurrentPosition() >= (1.0 * CUT_MOTOR_STEPS_PER_INCH)) {
        Serial.println("Cutting Stage 1: Cut motor at >= 1 inch, checking wood suction."); // Added
        if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == LOW) {
          Serial.println("Wood suction error detected! Restarting system."); // Added
          digitalWrite(BLUE_LED, LOW);
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(YELLOW_LED, LOW);
          
          for (int i = 0; i < 5; i++) {
            digitalWrite(RED_LED, HIGH);
            delay(150);
            digitalWrite(RED_LED, LOW);
            delay(150);
          }
          
          esp_restart();
        }
        cuttingStage = 2;
        Serial.println("Cutting Stage 1: Wood suction OK (or not present). Proceeding to stage 2."); // Added
      }
      break;
      
    case 2: // Wait for cut to complete
      if (cutMotor && !cutMotor->isRunning()) {
        Serial.println("Cutting Stage 2: Cut complete."); // Added
        sendSignalToStage2();

        if (cutMotor) cutMotor->setSpeedInHz((uint32_t)CUT_RETURN_SPEED);
        if (positionMotor) positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED);

        int sensorValue = digitalRead(WOOD_SENSOR);
        bool noWoodDetected = (sensorValue == HIGH);
        
        if (noWoodDetected) {
          Serial.println("Cutting Stage 2: No wood detected after cut. Entering no-wood operation."); // Added
          performNoWoodOperation();
          cuttingStage = 6;
        } else {
          Serial.println("Cutting Stage 2: Wood detected. Proceeding with normal return."); // Added
          if (cutMotor) cutMotor->moveTo(0);
          digitalWrite(POSITION_CLAMP, LOW); // Ensure position clamp is engaged
          Serial.println("Position clamp engaged for initial position move back."); //Added

          if (positionMotor) {
            long currentPos = positionMotor->getCurrentPosition();
            Serial.print("Current position motor position (steps): "); Serial.println(currentPos); // Added
            Serial.print("Moving position motor back by (steps): "); Serial.println(0.1 * POSITION_MOTOR_STEPS_PER_INCH); // Added
            positionMotor->moveTo(currentPos - (long)(0.1 * POSITION_MOTOR_STEPS_PER_INCH));
          }
          cuttingStage = 3;
        }
      }
      break;
      
    case 3: // Wait for initial position move
      if (positionMotor && !positionMotor->isRunning()) {
        Serial.println("Cutting Stage 3: Initial position move complete."); // Added
        digitalWrite(POSITION_CLAMP, HIGH);
        digitalWrite(WOOD_SECURE_CLAMP, HIGH);
        Serial.println("Position clamp and wood secure clamp disengaged."); // Added

        if (positionMotor) {
          positionMotor->setAcceleration((uint32_t)POSITION_RETURN_ACCELERATION);
          positionMotor->moveTo(0);
        }
        Serial.println("Position motor moving to home (0)."); // Added
        cuttingStage = 4;
      }
      break;
      
    case 4: { // Wait for position motor to reach home position
      if (positionMotor && !positionMotor->isRunning()) {
        Serial.println("Cutting Stage 4: Position motor at home (0)."); // Added
        digitalWrite(POSITION_CLAMP, LOW);
        Serial.println("Position clamp engaged."); // Added

        // Only check for cut motor errors, not position motor errors
        if (cutMotor && !cutMotor->isRunning()) {
          Serial.println("Cut motor also at home. Checking cut motor position switch."); // Added
          bool sensorDetectedHome = false;
          for (int i = 0; i < 3; i++) {
            delay(30);  // Increased delay to 30ms between checks
            cutHomingSwitch.update();
            Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(cutHomingSwitch.read()); // Renamed // Added
            if (cutHomingSwitch.read() == HIGH) {
              sensorDetectedHome = true;
              Serial.println("Cut motor position switch detected HIGH."); // Added
              break;  // One HIGH reading is enough
            }
          }
          if (!sensorDetectedHome) {
            Serial.println("ERROR: Cut motor position switch did not detect home after return!"); // Added
            if (cutMotor) cutMotor->stopMove();
            if (positionMotor) positionMotor->stopMove();
            digitalWrite(POSITION_CLAMP, LOW);
            digitalWrite(WOOD_SECURE_CLAMP, LOW);
            digitalWrite(RED_LED, HIGH);
            digitalWrite(YELLOW_LED, LOW);
            currentState = ERROR;
            errorStartTime = millis();
            cuttingStage = 0; // Reset cutting stage for next cycle attempt after error reset
            Serial.println("Transitioning to ERROR state due to cut motor homing failure after cut."); //Added
          } else {
            Serial.println("Cut motor position switch confirmed home. Moving position motor to final position."); // Added
            if (positionMotor) positionMotor->moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
            cuttingStage = 5;
          }
        }
      }
      break;
    }
      
    case 5: // Wait for position motor to reach final position (normal mode)
      if (positionMotor && !positionMotor->isRunning()) {
        Serial.println("Cutting Stage 5: Position motor at final position. Cycle complete."); // Added
        digitalWrite(WOOD_SECURE_CLAMP, LOW);
        Serial.println("Wood secure clamp engaged."); // Added
        digitalWrite(YELLOW_LED, LOW);
        cuttingCycleInProgress = false;
        currentState = READY;
        cuttingStage = 0; // Reset cutting stage for next cycle
        Serial.println("Transitioning to READY state."); //Added
      }
      break;
      
    case 6: // Wait for no-wood mode sequence to complete
      // Serial.println("Cutting Stage 6: Handling no-wood mode sequence."); // Added - This might be too verbose, let's see
      static int noWoodStage = 0;
      static unsigned long cylinderActionTime = 0;
      static bool waitingForCylinder = false;
      
      if (waitingForCylinder && (millis() - cylinderActionTime >= 150)) {
        waitingForCylinder = false;
        noWoodStage++;
      }
      
      if (!waitingForCylinder) {
        switch (noWoodStage) {
          case 0: // Start position motor to home immediately while cut motor is returning
            if (cutMotor && !cutMotor->isRunning()) {
              Serial.println("No-Wood Stage 0: Disengaging wood secure clamp, starting position motor to home."); // Added
              digitalWrite(WOOD_SECURE_CLAMP, HIGH);
              if (positionMotor) positionMotor->moveTo(0);
            }
            
            if (cutMotor && !cutMotor->isRunning()) {
              Serial.println("No-Wood Stage 0: Cut motor returned home. Engaging position clamp."); // Added
              digitalWrite(POSITION_CLAMP, LOW);
              cylinderActionTime = millis();
              waitingForCylinder = true;
            }
            break;
            
          case 1: // Wait for position motor to reach home (if it hasn't already)
            if (positionMotor && !positionMotor->isRunning()) {
              Serial.println("No-Wood Stage 1: Position motor at home. Disengaging position clamp."); // Added
              digitalWrite(POSITION_CLAMP, HIGH);
              cylinderActionTime = millis();
              waitingForCylinder = true;
            }
            break;
            
          case 2: // Move position motor to 2.0 inches
            Serial.println("No-Wood Stage 2: Moving position motor to 2.0 inches."); // Added
            if (positionMotor) positionMotor->moveTo(2.0 * POSITION_MOTOR_STEPS_PER_INCH);
            noWoodStage = 3;
            break;
            
          case 3: // Wait for position motor to reach 2.0 inches
            if (positionMotor && !positionMotor->isRunning()) {
              Serial.println("No-Wood Stage 3: Position motor at 2.0 inches. Engaging position clamp."); // Added
              digitalWrite(POSITION_CLAMP, LOW);
              cylinderActionTime = millis();
              waitingForCylinder = true;
            }
            break;
            
          case 4: // Move position motor to home again
            Serial.println("No-Wood Stage 4: Moving position motor to home."); // Added
            if (positionMotor) positionMotor->moveTo(0);
            noWoodStage = 5;
            break;
            
          case 5: // Wait for position motor to reach home
            if (positionMotor && !positionMotor->isRunning()) {
              Serial.println("No-Wood Stage 5: Position motor at home. Disengaging position clamp."); // Added
              digitalWrite(POSITION_CLAMP, HIGH);
              cylinderActionTime = millis();
              waitingForCylinder = true;
            }
            break;
            
          case 6: // Move position motor to 3.45 inches final time
            Serial.println("No-Wood Stage 6: Moving position motor to 3.45 inches (final position)."); // Added
            if (positionMotor) positionMotor->moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
            noWoodStage = 7;
            break;
            
          case 7: // Wait for final position and complete sequence
            if (positionMotor && !positionMotor->isRunning()) {
              Serial.println("No-Wood Stage 7: Position motor at final position."); // Added
              // Only check cut motor position, don't enter error state for position motor
              bool sensorDetectedHome = false;
              Serial.println("No-Wood Stage 7: Checking cut motor position switch."); // Added
              for (int i = 0; i < 3; i++) {
                delay(30);  // Increased delay to 30ms between checks
                cutHomingSwitch.update();
                Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(cutHomingSwitch.read()); // Renamed // Added
                if (cutHomingSwitch.read() == HIGH) {
                  sensorDetectedHome = true;
                  Serial.println("Cut motor position switch detected HIGH during no-wood sequence completion."); // Added
                  break;  // One HIGH reading is enough
                }
              }
              if (!sensorDetectedHome) {
                Serial.println("WARNING: Cut motor position switch did NOT detect home after no-wood sequence, but proceeding anyway."); // Added
              }

              // Proceed regardless of cut motor position - no error state
              digitalWrite(WOOD_SECURE_CLAMP, HIGH); // Ensure wood secure clamp is disengaged
              Serial.println("Wood secure clamp disengaged (final check in no-wood)."); // Added
              digitalWrite(YELLOW_LED, LOW);
              digitalWrite(BLUE_LED, HIGH); // Blue LED for no-wood ready

              noWoodStage = 0;
              waitingForCylinder = false;
              cuttingCycleInProgress = false;
              currentState = READY;
              continuousModeActive = false; // Ensure continuous mode is off after a no-wood cycle
              startSwitchSafe = false;      // Require start switch to be cycled
              cuttingStage = 0; // Reset main cutting stage
              Serial.println("No-wood sequence complete. Transitioning to READY state. Continuous mode OFF. Start switch needs cycling."); //Added
            }
            break;
        }
      }
      break;
  }
}

void performReturnOperation() {
  // This function is no longer needed as the return operation is now handled
  // directly in the performCuttingOperation function
  
  // Transition to READY state
  currentState = READY;
  digitalWrite(YELLOW_LED, LOW);
  cuttingCycleInProgress = false;
}

void performPositioningOperation() {
  // This function is no longer needed as the positioning operation is now handled
  // directly in the performCuttingOperation function
  
  // Transition to READY state
  currentState = READY;
  digitalWrite(YELLOW_LED, LOW);
  cuttingCycleInProgress = false;
}

void performNoWoodOperation() {
  Serial.println("Entering performNoWoodOperation function."); // Added
  // Special sequence for no-wood mode

  // Configure motors for return
  if (cutMotor) {
    cutMotor->setSpeedInHz((uint32_t)CUT_RETURN_SPEED);
    cutMotor->moveTo(0); // Return cut motor to home
  }
  Serial.println("performNoWoodOperation: Cut motor set to return to home."); // Added

  // Set up the position motor for the sequence
  if (positionMotor) {
    positionMotor->setSpeedInHz((uint32_t)POSITION_NORMAL_SPEED);
  }

  // The sequence will be handled in the cuttingStage 6 case in performCuttingOperation
  Serial.println("performNoWoodOperation: Configuration complete. Sequence handled in cuttingStage 6."); // Added
  // (Do not transition to READY immediately; wait in stage 6 until sequence completes)
}

void handleErrorState() {
  Serial.println("Entering handleErrorState."); // Added
  // Blink error LEDs
  if (millis() - lastErrorBlinkTime > 250) {
    errorBlinkState = !errorBlinkState;
    digitalWrite(RED_LED, errorBlinkState);
    digitalWrite(YELLOW_LED, !errorBlinkState); // Alternate with red
    lastErrorBlinkTime = millis();
  }
  
  // Keep motors stopped
  if (cutMotor) cutMotor->stopMove();
  if (positionMotor) positionMotor->stopMove();
  
  // Wait for reload switch to acknowledge error
  if (errorAcknowledged) {
    currentState = ERROR_RESET;
    Serial.println("Error acknowledged in handleErrorState. Transitioning to ERROR_RESET."); // Added
  }
}

void resetFromError() {
  Serial.println("Entering resetFromError."); // Added
  // Turn off error LEDs
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  
  // Reset flags
  errorAcknowledged = false;
  woodSuctionError = false;
  
  // Return to homing state to re-initialize
  currentState = STARTUP;
  Serial.println("Error reset, restarting system. Transitioning to STARTUP."); //Uncommented and modified
}
