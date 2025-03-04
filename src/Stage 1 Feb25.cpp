#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>

// Motor Pin Definitions
#define CUT_MOTOR_PULSE_PIN 22
#define CUT_MOTOR_DIR_PIN 23
#define POSITION_MOTOR_PULSE_PIN 32
#define POSITION_MOTOR_DIR_PIN 33

// Switch and Sensor Pin Definitions
#define CUT_MOTOR_POSITION_SWITCH 25
#define POSITION_MOTOR_POSITION_SWITCH 27
#define RELOAD_SWITCH 14
#define START_CYCLE_SWITCH 18
#define WOOD_SENSOR 34
#define WAS_WOOD_SUCTIONED_SENSOR 5

// Clamp Pin Definitions
#define POSITION_CLAMP 13
#define WOOD_SECURE_CLAMP 15

// LED Pin Definitions
#define RED_LED 26   // Error LED
#define YELLOW_LED 21 // Busy/Reload LED
#define GREEN_LED 4   // Ready LED
#define BLUE_LED 2    // Setup/No-Wood LED

// Signal Output
#define SIGNAL_TO_STAGE_2_PIN 19

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
const int CUT_MOTOR_STEPS_PER_INCH = 76;
const int POSITION_MOTOR_STEPS_PER_INCH = 1000;
const float CUT_TRAVEL_DISTANCE = 9.0; // inches
const float POSITION_TRAVEL_DISTANCE = 3.45; // inches
const int CUT_HOMING_DIRECTION = -1;
const int POSITION_HOMING_DIRECTION = -1;

// Speed and Acceleration Settings
const float CUT_NORMAL_SPEED = 500;
const float CUT_RETURN_SPEED = 1000;
const float CUT_ACCELERATION = 2000;
const float CUT_HOMING_SPEED = 300;
const float POSITION_NORMAL_SPEED = 50000;
const float POSITION_RETURN_SPEED = 50000;
const float POSITION_ACCELERATION = 40000;
const float POSITION_HOMING_SPEED = 2000; // Slower speed for homing operations

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
bool signalSent = false;
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
  pinMode(RELOAD_SWITCH, INPUT);                          // Manual switch with pull-down
  pinMode(START_CYCLE_SWITCH, INPUT);                     // Manual switch with pull-down
  
  // Configure sensor pins - make consistent with explanation
  pinMode(WOOD_SENSOR, INPUT_PULLUP);          // Active LOW (LOW = wood present)
  pinMode(WAS_WOOD_SUCTIONED_SENSOR, INPUT_PULLUP); // Active LOW (LOW = wood detected)
  
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
  
  // Configure signal output pin
  pinMode(SIGNAL_TO_STAGE_2_PIN, OUTPUT);
  digitalWrite(SIGNAL_TO_STAGE_2_PIN, HIGH); // Active LOW signal
  
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
  
  positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_ACCELERATION);
  
  // Initially set motors to use position 0
  cutMotor.setCurrentPosition(0);
  positionMotor.setCurrentPosition(0);
  
  // Start in STARTUP state
  currentState = STARTUP;
  
  // Check if start switch is already ON at startup
  startCycleSwitch.update();
  if (startCycleSwitch.read() == HIGH) {
    startSwitchSafe = false;
    Serial.println("WARNING: Start switch is ON during startup. Turn it OFF before operation.");
  } else {
    startSwitchSafe = true;
  }
  
  Serial.println("System initialized, ready to begin homing sequence");
  delay(1000);  // Brief delay before starting homing
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
    Serial.println("Start switch is now safe to use");
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
      Serial.println("Entered reload mode");
    } else if (!reloadSwitchOn && isReloadMode) {
      // Exit reload mode
      isReloadMode = false;
      digitalWrite(POSITION_CLAMP, LOW);   // Re-engage position clamp
      digitalWrite(WOOD_SECURE_CLAMP, LOW); // Re-engage wood secure clamp
      digitalWrite(YELLOW_LED, LOW);       // Turn off yellow LED
      Serial.println("Exited reload mode, ready for operation");
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
      Serial.println("Continuous operation mode activated");
    } else {
      Serial.println("Continuous operation mode deactivated");
    }
  }
  
  // State machine
  switch (currentState) {
    case STARTUP:
      Serial.println("Starting homing sequence...");
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
          Serial.println("Starting cutting cycle");
          // Turn off ready LED, turn on busy LED
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(YELLOW_LED, HIGH);
          
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
            Serial.println("No wood detected, will enter no-wood mode after cut");
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
      Serial.println("Cut motor homed successfully");
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
      Serial.println("Position motor homed successfully");
      
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
      Serial.println("Position motor moved to initial position");
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
    Serial.println("Homing complete, system ready");
  }
}

void performCuttingOperation() {
  static int cuttingStage = 0;
  static unsigned long signalStartTime = 0;
  static unsigned long lastDebugTime = 0;
  static bool signalActive = false;
  
  // Debug output
  if (millis() - lastDebugTime > 1000) {
    Serial.print("Cutting stage: ");
    Serial.print(cuttingStage);
    Serial.print(" Position: ");
    Serial.print(cutMotor.currentPosition());
    Serial.print(" Target: ");
    Serial.println(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
    lastDebugTime = millis();
  }
  
  // Handle signal timing independently of motor movements
  if (signalActive && millis() - signalStartTime >= 2000) {
    digitalWrite(SIGNAL_TO_STAGE_2_PIN, HIGH);
    signalActive = false;
    Serial.println("Signal to Stage 2 completed");
  }
  
  switch (cuttingStage) {
    case 0: // Start cut motion
      Serial.println("Starting cut motion to position: " + 
                    String(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH));
      // Move cut motor forward to perform cut
      cutMotor.moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
      cuttingStage = 1;
      break;
      
    case 1: // Wait for cut to complete
      if (cutMotor.distanceToGo() == 0) {
        // Check if wood was suctioned (active LOW per explanation)
        if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == LOW) {
          Serial.println("ERROR: Wood detected in suction sensor after cut, performing hardware reset");
          ESP.restart(); // Hardware reset of ESP32 as per instructions
        }
        
        // Signal Stage 2 controller - start signal but don't wait
        digitalWrite(SIGNAL_TO_STAGE_2_PIN, LOW);
        signalStartTime = millis();
        signalActive = true;
        signalSent = true;
        
        // Configure motors for return speeds
        cutMotor.setMaxSpeed(CUT_RETURN_SPEED);
        positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
        
        // Check if we're in no-wood mode
        if (!woodPresent) {
          // Enter no-wood specific sequence
          performNoWoodOperation();
          cuttingStage = 0; // Reset for next cycle
        } else {
          // Start cut motor return
          cutMotor.moveTo(0);
          
          // Begin position motor return while keeping position clamp engaged
          digitalWrite(POSITION_CLAMP, LOW); // Ensure position clamp is engaged during initial movement
          
          // Move 0.1 inches back from current position (which should be at POSITION_TRAVEL_DISTANCE)
          long currentPos = positionMotor.currentPosition();
          positionMotor.moveTo(currentPos - (0.1 * POSITION_MOTOR_STEPS_PER_INCH));
          
          cuttingStage = 2;
        }
      }
      break;
      
    case 2: // Wait for initial position move
      if (positionMotor.distanceToGo() == 0) {
        // After 0.1 inch, disengage position clamp and wood secure clamp
        digitalWrite(POSITION_CLAMP, HIGH);  // Retract position clamp after 0.1 inch movement
        digitalWrite(WOOD_SECURE_CLAMP, HIGH); // Disengage wood secure clamp
        // Continue position motor movement to home
        positionMotor.moveTo(0);
        cuttingStage = 3;
      }
      break;
      
    case 3: // Wait for position motor to return home
      if (positionMotor.distanceToGo() == 0) {
        // Position motor is now home
        digitalWrite(POSITION_CLAMP, LOW); // Re-engage position clamp when at home position
        Serial.println("Position motor at home, re-engaging position clamp");
        cuttingStage = 4;
      }
      break;
      
    case 4: // Wait for cut motor to return home
      if (cutMotor.distanceToGo() == 0) {
        // Move position motor to 3.45 inches
        positionMotor.moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
        cuttingStage = 5;
      }
      break;
      
    case 5: // Wait for position motor to reach final position
      if (positionMotor.distanceToGo() == 0) {
        // Extend secure wood clamp
        digitalWrite(WOOD_SECURE_CLAMP, LOW);
        
        // Turn off busy LED
        digitalWrite(YELLOW_LED, LOW);
        
        // Reset the cutting cycle in progress flag
        cuttingCycleInProgress = false;
        
        currentState = READY;
        cuttingStage = 0; // Reset for next cycle
        
        // If in continuous mode, the next cycle will start automatically
        // when we return to the READY state and check continuousModeActive
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
  digitalWrite(POSITION_CLAMP, HIGH); // Retract position clamp
  
  // Configure motors for return
  cutMotor.setMaxSpeed(CUT_RETURN_SPEED);
  cutMotor.moveTo(0); // Return cut motor to home
  
  positionMotor.setMaxSpeed(POSITION_RETURN_SPEED);
  // Move position motor to 3.45 inches
  positionMotor.moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
  
  // Transition to READY state once motors complete their movements
  currentState = READY;
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
  Serial.println("Error reset, restarting system");
}
