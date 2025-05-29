#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <esp_system.h>
#include <ESP32Servo.h>
#include "Config/Config.h"
#include "Config/Pins_Definitions.h"
#include "StateMachine/StateMachine.h"
#include "OTA_Manager.h"

//* ************************************************************************
//* ************************ AUTOMATED TABLE SAW **************************
//* ************************************************************************
// Main control system for Stage 1 of the automated table saw.
// Handles state machine logic, motor control, sensor monitoring, and safety systems.

// Pin definitions and configuration constants are now in Config/ header files

// Timing variables (constants moved to Config/Config.h)
unsigned long catcherServoActiveStartTime = 0;
bool catcherServoIsActiveAndTiming = false;

unsigned long catcherClampEngageTime = 0;
bool catcherClampIsEngaged = false;

// SystemStates Enum is now in StateMachine/StateMachine.h
extern SystemState currentState;
extern SystemState previousState;

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
Bounce woodSensor = Bounce();
Bounce wasWoodSuctionedSensor = Bounce();

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
bool startSwitchSafeAfterNoWood = true;  // Flag to track safety after NOWOOD state
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

// New flag to track cut motor return during YES_WOOD mode
bool cutMotorInYesWoodReturn = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Automated Table Saw Control System - Stage 1");
  Serial.println("DIAGNOSTIC VERSION - Adding OTA/WiFi");

  //! Initialize OTA functionality
  Serial.println("Initializing OTA...");
  initOTA();
  Serial.println("OTA initialization complete");
  
  //! Configure basic pin modes
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
  
  // Initialize LEDs directly
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);  // Turn on blue LED
  
  Serial.println("Pin configs complete, initializing motors...");
  
  //! Initialize motors
  engine.init();
  Serial.println("Engine initialized");

  cutMotor = engine.stepperConnectToPin(CUT_MOTOR_PULSE_PIN);
  if (cutMotor) {
    cutMotor->setDirectionPin(CUT_MOTOR_DIR_PIN);
    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_NORMAL_SPEED);
    cutMotor->setAcceleration((uint32_t)CUT_MOTOR_NORMAL_ACCELERATION);
    cutMotor->setCurrentPosition(0);
    Serial.println("Cut motor initialized successfully");
  } else {
    Serial.println("Failed to init cutMotor");
  }

  positionMotor = engine.stepperConnectToPin(POSITION_MOTOR_PULSE_PIN);
  if (positionMotor) {
    positionMotor->setDirectionPin(POSITION_MOTOR_DIR_PIN);
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_NORMAL_SPEED);
    positionMotor->setAcceleration((uint32_t)POSITION_MOTOR_NORMAL_ACCELERATION);
    positionMotor->setCurrentPosition(0);
    Serial.println("Position motor initialized successfully");
  } else {
    Serial.println("Failed to init positionMotor");
  }
  
  Serial.println("Motor setup complete - OTA + Motors working");
  
  //! Configure switch debouncing
  Serial.println("Configuring switch debouncing...");
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
  
  woodSensor.attach(WOOD_SENSOR);
  woodSensor.interval(5);
  
  wasWoodSuctionedSensor.attach(WAS_WOOD_SUCTIONED_SENSOR);
  wasWoodSuctionedSensor.interval(5);
  
  //! Initialize servo
  Serial.println("Initializing servo...");
  catcherServo.setTimerWidth(14);
  catcherServo.attach(CATCHER_SERVO_PIN);
  
  Serial.println("Switches and servo configured");
  
  //! Initialize state machine
  Serial.println("Initializing state machine...");
  initializeStateMachine();
  
  //! Configure initial state
  currentState = STARTUP;
  previousState = STARTUP;
  
  // Check initial switch states for safety
  startCycleSwitch.update();
  if (startCycleSwitch.read() == HIGH) {
    startSwitchSafe = false;
    Serial.println("WARNING: Start cycle switch is ON at startup - Turn OFF first to enable safety");
  } else {
    startSwitchSafe = false;  // Force false even if switch is off - requires a cycle OFF->ON
    Serial.println("Start cycle switch is OFF at startup - Switch must be cycled OFF->ON to enable safety");
  }
  
  Serial.println("State machine initialized");
  Serial.println("Setup complete - full system ready");
}

void loop() {
  // Re-enable state machine to test if it causes watchdog reset issue
  Serial.println("Loop starting - testing state machine...");
  
  // Execute the state machine
  updateStateMachine();
  
  //! Handle OTA updates
  handleOTA();
  
  // Prevent watchdog reset
  yield();
  
  // Add delay to slow down debug output
  delay(500);
  
  Serial.println("Loop completed successfully");
} 