#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>
#include <esp_system.h> // Include for ESP.restart()
// Remove WiFi and OTA libraries

// Motor Pin Definitions
#define CUT_MOTOR_PULSE_PIN 48
#define CUT_MOTOR_DIR_PIN 47
#define POSITION_MOTOR_PULSE_PIN 21
#define POSITION_MOTOR_DIR_PIN 20

// Switch and Sensor Pin Definitions
#define CUT_MOTOR_POSITION_SWITCH 10
#define POSITION_MOTOR_POSITION_SWITCH 9
#define RELOAD_SWITCH 14
#define START_CYCLE_SWITCH 13
#define WOOD_SENSOR 11
#define WAS_WOOD_SUCTIONED_SENSOR 8

// Clamp Pin Definitions
#define POSITION_CLAMP 18
#define WOOD_SECURE_CLAMP 17

// Add signal pin definition
#define STAGE2_SIGNAL_OUT_PIN 19  // Using pin 19 for direct signaling to Stage 1 to Stage 2 machine

// LED Pin Definitionss
#define RED_LED 7  // Error LED
#define YELLOW_LED 6 // Busy/Reload LED
#define GREEN_LED 15   // Ready LED
#define BLUE_LED 16    // Setup/No-Wood LED

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
const int CUT_MOTOR_STEPS_PER_INCH = 38;  // Halved from 76 due to 80-tooth pulley
const int POSITION_MOTOR_STEPS_PER_INCH = 1000;
const float CUT_TRAVEL_DISTANCE = 7; // inches
const float POSITION_TRAVEL_DISTANCE = 3.45; // inches
const int CUT_HOMING_DIRECTION = -1;
const int POSITION_HOMING_DIRECTION = -1;

// Speed and Acceleration Settings
const float CUT_NORMAL_SPEED = 45;  // Halved from 90
const float CUT_RETURN_SPEED = 750;  // Halved from 1500
const float CUT_ACCELERATION = 1000;  // Halved from 2000
const float CUT_HOMING_SPEED = 150;  // Halved from 300
const float POSITION_NORMAL_SPEED = 30000;
const float POSITION_RETURN_SPEED = 30000;
const float POSITION_ACCELERATION = 30000;
const float POSITION_HOMING_SPEED = 2000; // Slower speed for homing operations
const float POSITION_RETURN_ACCELERATION = 30000; // You can adjust this value as needed

// Add a timeout constant for cut motor homing
const unsigned long CUT_HOME_TIMEOUT = 5000; // 5000 ms (5 seconds) timeout

// Create motor objects
AccelStepper cutMotor(AccelStepper::DRIVER, CUT_MOTOR_PULSE_PIN, CUT_MOTOR_DIR_PIN);
AccelStepper positionMotor(AccelStepper::DRIVER, POSITION_MOTOR_PULSE_PIN, POSITION_MOTOR_DIR_PIN);

// Bounce objects for debouncing switches
Bounce cutPositionSwitch = Bounce();
Bounce positionPositionSwitch = Bounce();
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
  pinMode(CUT_MOTOR_POSITION_SWITCH, INPUT_PULLUP);       // Homing switch
  pinMode(POSITION_MOTOR_POSITION_SWITCH, INPUT_PULLUP);  // Homing switch
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
  cutPositionSwitch.attach(CUT_MOTOR_POSITION_SWITCH);
  cutPositionSwitch.interval(20);  // 20ms debounce time
  
  positionPositionSwitch.attach(POSITION_MOTOR_POSITION_SWITCH);
  positionPositionSwitch.interval(20);
  
  reloadSwitch.attach(RELOAD_SWITCH);
  reloadSwitch.interval(20);
  
  startCycleSwitch.attach(START_CYCLE_SWITCH);
  startCycleSwitch.interval(20);
  
  // Configure motors
  cutMotor.setMaxSpeed(CUT_NORMAL_SPEED);
  cutMotor.setAcceleration(CUT_ACCELERATION);
  cutMotor.setMinPulseWidth(10);  // Set minimum pulse width to 3 microseconds
  
  positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_ACCELERATION);
  positionMotor.setMinPulseWidth(10);  // Set minimum pulse width to 3 microseconds
  
  // Initially set motors to use position 0
  cutMotor.setCurrentPosition(0);
  positionMotor.setCurrentPosition(0);
  
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
  cutPositionSwitch.update();
  positionPositionSwitch.update();
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
          cutMotor.setMaxSpeed(CUT_NORMAL_SPEED);  // Change from 2000 back to CUT_NORMAL_SPEED (100)
          cutMotor.setAcceleration(CUT_ACCELERATION);
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
  cutMotor.run();
  positionMotor.run();
}

void performHomingSequence() {
  static bool cutMotorHomed = false;
  static bool positionMotorHomed = false;
  static bool positionMotorMoved = false;
  static unsigned long blinkTimer = 0;
  
  // Blink blue LED to indicate homing
  if (millis() - blinkTimer > 500) {
    blinkState = !blinkState;
    digitalWrite(BLUE_LED, blinkState);
    blinkTimer = millis();
  }
  
  // Home cut motor first
  if (!cutMotorHomed) {
    // If the cut motor position switch is already active, move away first
    if (cutPositionSwitch.read() == HIGH) {
      cutMotor.setMaxSpeed(CUT_HOMING_SPEED);
      cutMotor.moveTo(10); // Move slightly away from switch
      if (cutMotor.distanceToGo() == 0) {
        // Now move back to find the switch
        cutMotor.setSpeed(CUT_HOMING_SPEED * CUT_HOMING_DIRECTION);
        cutMotor.moveTo(-10000); // Move toward switch (will stop when switch activated)
      }
    } else {
      // Move toward home switch
      cutMotor.setSpeed(CUT_HOMING_SPEED * CUT_HOMING_DIRECTION);
      cutMotor.moveTo(-10000); // Large number in homing direction
    }
    
    // Check if we hit the switch
    if (cutPositionSwitch.read() == HIGH) {
      cutMotor.stop();
      cutMotor.setCurrentPosition(0);
      cutMotorHomed = true;
      // Serial.println("Cut motor homed successfully");
    }
  } else if (!positionMotorHomed) {
    // Home position motor
    digitalWrite(POSITION_CLAMP, HIGH); // Disengage position clamp for homing
    
    // If the position motor switch is already active, move away first
    if (positionPositionSwitch.read() == HIGH) {
      positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
      positionMotor.moveTo(100 * POSITION_MOTOR_STEPS_PER_INCH); // Move slightly away
      if (positionMotor.distanceToGo() == 0) {
        // Now move back to find the switch
        positionMotor.setSpeed(POSITION_HOMING_SPEED * POSITION_HOMING_DIRECTION);
        positionMotor.moveTo(-10000 * POSITION_MOTOR_STEPS_PER_INCH); // Move toward switch
      }
    } else {
      // Move toward home switch
      positionMotor.setSpeed(POSITION_HOMING_SPEED * POSITION_HOMING_DIRECTION);
      positionMotor.moveTo(-10000 * POSITION_MOTOR_STEPS_PER_INCH);
    }
    
    // Check if we hit the switch
    if (positionPositionSwitch.read() == HIGH) {
      positionMotor.stop();
      // Set current position to -1 inch
      positionMotor.setCurrentPosition(-1 * POSITION_MOTOR_STEPS_PER_INCH);
      positionMotorHomed = true;
      // Serial.println("Position motor homed successfully");
      
      // Set back to normal speed after homing is complete
      positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
    }
  } else if (!positionMotorMoved) {
    // Move position motor to full travel distance
    positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
    positionMotor.moveTo(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
    
    if (positionMotor.distanceToGo() == 0) {
      digitalWrite(POSITION_CLAMP, LOW); // Re-engage position clamp
      positionMotorMoved = true;
      // Serial.println("Position motor moved to initial position");
    }
  } else {
    // Homing complete
    cutMotorHomed = false;
    positionMotorHomed = false;
    positionMotorMoved = false;
    isHomed = true;
    
    // Turn off homing LED, turn on ready LED
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    
    currentState = READY;
    // Serial.println("Homing complete, system ready");
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
    // Rapid blinking of red and yellow LEDs
    if (millis() - lastErrorBlinkTime > 100) { // Fast 100ms blink rate
      errorBlinkState = !errorBlinkState;
      digitalWrite(RED_LED, errorBlinkState);
      digitalWrite(YELLOW_LED, !errorBlinkState); // Alternate LEDs for better visibility
      lastErrorBlinkTime = millis();
    }
    
    // Stop all motors and do not proceed
    cutMotor.stop();
    positionMotor.stop();
    
    // Keep clamps engaged for safety
    digitalWrite(POSITION_CLAMP, LOW);
    digitalWrite(WOOD_SECURE_CLAMP, LOW);
    
    // Check if reload switch is pressed to acknowledge error
    if (reloadSwitch.rose()) {
      homePositionErrorDetected = false;
      currentState = ERROR_RESET;
      errorAcknowledged = true;
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
      cutMotor.moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
      cuttingStage = 1;
      break;
      
    case 1: // Check for wood suction when saw is 1 inch into the cut
      if (cutMotor.currentPosition() >= (1.0 * CUT_MOTOR_STEPS_PER_INCH)) {
        if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == LOW) {
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
      }
      break;
      
    case 2: // Wait for cut to complete
      if (cutMotor.distanceToGo() == 0) {
        sendSignalToStage2();

        cutMotor.setMaxSpeed(CUT_RETURN_SPEED);
        positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);

        int sensorValue = digitalRead(WOOD_SENSOR);
        bool noWoodDetected = (sensorValue == HIGH);
        
        if (noWoodDetected) {
          performNoWoodOperation();
          cuttingStage = 6;
        } else {
          cutMotor.moveTo(0);
          digitalWrite(POSITION_CLAMP, LOW);
          
          long currentPos = positionMotor.currentPosition();
          positionMotor.moveTo(currentPos - (0.1 * POSITION_MOTOR_STEPS_PER_INCH));
          
          cuttingStage = 3;
        }
      }
      break;
      
    case 3: // Wait for initial position move
      if (positionMotor.distanceToGo() == 0) {
        digitalWrite(POSITION_CLAMP, HIGH);
        digitalWrite(WOOD_SECURE_CLAMP, HIGH);
        
        positionMotor.setAcceleration(POSITION_RETURN_ACCELERATION);
        positionMotor.moveTo(0);
        cuttingStage = 4;
      }
      break;
      
    case 4: { // Wait for position motor to reach home position
      if (positionMotor.distanceToGo() == 0) {
        digitalWrite(POSITION_CLAMP, LOW);
        
        // Only check for cut motor errors, not position motor errors
        if (cutMotor.distanceToGo() == 0) {
          bool sensorDetectedHome = false;
          for (int i = 0; i < 3; i++) {
            delay(30);  // Increased delay to 30ms between checks
            cutPositionSwitch.update();
            if (cutPositionSwitch.read() == HIGH) {
              sensorDetectedHome = true;
              break;  // One HIGH reading is enough
            }
          }
          if (!sensorDetectedHome) {
            cutMotor.stop();
            positionMotor.stop();
            digitalWrite(POSITION_CLAMP, LOW);
            digitalWrite(WOOD_SECURE_CLAMP, LOW);
            digitalWrite(RED_LED, HIGH);
            digitalWrite(YELLOW_LED, LOW);
            currentState = ERROR;
            errorStartTime = millis();
            cuttingStage = 0;
          } else {
            positionMotor.moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
            cuttingStage = 5;
          }
        }
      }
      break;
    }
      
    case 5: // Wait for position motor to reach final position (normal mode)
      if (positionMotor.distanceToGo() == 0) {
        digitalWrite(WOOD_SECURE_CLAMP, LOW);
        digitalWrite(YELLOW_LED, LOW);
        cuttingCycleInProgress = false;
        currentState = READY;
        cuttingStage = 0;
      }
      break;
      
    case 6: // Wait for no-wood mode sequence to complete
      static int noWoodStage = 0;
      static unsigned long cylinderActionTime = 0;
      static bool waitingForCylinder = false;
      static bool positionMotorStarted = false;
      
      if (waitingForCylinder && (millis() - cylinderActionTime >= 150)) {
        waitingForCylinder = false;
        noWoodStage++;
      }
      
      if (!waitingForCylinder) {
        switch (noWoodStage) {
          case 0: // Start position motor to home immediately while cut motor is returning
            if (!positionMotorStarted) {
              digitalWrite(WOOD_SECURE_CLAMP, HIGH);
              positionMotor.moveTo(0);
              positionMotorStarted = true;
            }
            
            if (cutMotor.distanceToGo() == 0) {
              digitalWrite(POSITION_CLAMP, LOW);
              cylinderActionTime = millis();
              waitingForCylinder = true;
              positionMotorStarted = false;
            }
            break;
            
          case 1: // Wait for position motor to reach home (if it hasn't already)
            if (positionMotor.distanceToGo() == 0) {
              digitalWrite(POSITION_CLAMP, HIGH);
              cylinderActionTime = millis();
              waitingForCylinder = true;
            }
            break;
            
          case 2: // Move position motor to 2.0 inches
            positionMotor.moveTo(2.0 * POSITION_MOTOR_STEPS_PER_INCH);
            noWoodStage = 3;
            break;
            
          case 3: // Wait for position motor to reach 2.0 inches
            if (positionMotor.distanceToGo() == 0) {
              digitalWrite(POSITION_CLAMP, LOW);
              cylinderActionTime = millis();
              waitingForCylinder = true;
            }
            break;
            
          case 4: // Move position motor to home again
            positionMotor.moveTo(0);
            noWoodStage = 5;
            break;
            
          case 5: // Wait for position motor to reach home
            if (positionMotor.distanceToGo() == 0) {
              digitalWrite(POSITION_CLAMP, HIGH);
              cylinderActionTime = millis();
              waitingForCylinder = true;
            }
            break;
            
          case 6: // Move position motor to 3.45 inches final time
            positionMotor.moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
            noWoodStage = 7;
            break;
            
          case 7: // Wait for final position and complete sequence
            if (positionMotor.distanceToGo() == 0) {
              // Only check cut motor position, don't enter error state for position motor
              bool sensorDetectedHome = false;
              for (int i = 0; i < 3; i++) {
                delay(30);  // Increased delay to 30ms between checks
                cutPositionSwitch.update();
                if (cutPositionSwitch.read() == HIGH) {
                  sensorDetectedHome = true;
                  break;  // One HIGH reading is enough
                }
              }
              
              // Proceed regardless of cut motor position - no error state
              digitalWrite(WOOD_SECURE_CLAMP, HIGH);
              digitalWrite(YELLOW_LED, LOW);
              digitalWrite(BLUE_LED, HIGH);
              
              noWoodStage = 0;
              waitingForCylinder = false;
              cuttingCycleInProgress = false;
              currentState = READY;
              continuousModeActive = false;
              startSwitchSafe = false;
              cuttingStage = 0;
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
  // Special sequence for no-wood mode
  static int noWoodStage = 0;
  static bool positionMotorStarted = false; // Reset this flag
  
  // Configure motors for return
  cutMotor.setMaxSpeed(CUT_RETURN_SPEED);
  cutMotor.moveTo(0); // Return cut motor to home
  
  // Set up the position motor for the sequence
  positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
  
  // The sequence will be handled in the cuttingStage 6 case in performCuttingOperation
  // Just initialize the first step here
  noWoodStage = 0;
  positionMotorStarted = false; // Reset the flag
  
  // (Do not transition to READY immediately; wait in stage 6 until sequence completes)
}

void handleErrorState() {
  // Blink error LEDs
  if (millis() - lastErrorBlinkTime > 250) {
    errorBlinkState = !errorBlinkState;
    digitalWrite(RED_LED, errorBlinkState);
    digitalWrite(YELLOW_LED, !errorBlinkState); // Alternate with red
    lastErrorBlinkTime = millis();
  }
  
  // Keep motors stopped
  cutMotor.stop();
  positionMotor.stop();
  
  // Wait for reload switch to acknowledge error
  if (errorAcknowledged) {
    currentState = ERROR_RESET;
  }
}

void resetFromError() {
  // Turn off error LEDs
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  
  // Reset flags
  errorAcknowledged = false;
  woodSuctionError = false;
  
  // Return to homing state to re-initialize
  currentState = STARTUP;
  // Serial.println("Error reset, restarting system");
}
