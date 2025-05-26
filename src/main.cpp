#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <esp_system.h>
#include <ESP32Servo.h>
#include "Config/pin_definitions.h"
#include "Config/system_config.h"
#include "OTAUpdater/ota_updater.h"
#include "Functions.h"
#include "StateMachine/StateManager.h"
#include "ErrorStates/fix_cut_motor_position.h"
#include "ErrorStates/standard_error.h"
#include "ErrorStates/error_reset.h"
#include "ErrorStates/suction_error_hold.h"
#include "ErrorStates/wood_caught_error.h"

//* ************************************************************************
//* ************************ AUTOMATED TABLE SAW **************************
//* ************************************************************************
// Main control system for Stage 1 of the automated table saw.
// Handles state machine logic, motor control, sensor monitoring, and safety systems.

// Pin definitions and configuration constants are now in Config/ header files

// Timing variables (constants moved to Config/system_config.h)
unsigned long catcherServoActiveStartTime = 0;
bool catcherServoIsActiveAndTiming = false;

unsigned long catcherClampEngageTime = 0;
bool catcherClampIsEngaged = false;

// SystemStates Enum is now in Functions.h
SystemState currentState = STARTUP;
SystemState previousState = ERROR_RESET; // Initialize to a different state to ensure first print

// Motor configuration constants moved to Config/system_config.h

// Speed and acceleration settings moved to Config/system_config.h

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

// New flag to track cut motor return during YES_WOOD mode
bool cutMotorInYesWoodReturn = false;

// Additional variables needed by states - declarations moved to above

// FIX_POSITION state steps now defined in fix_position.cpp

// StateManager instance is created in StateManager.cpp

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

  // Execute the state machine - all the logic below has been moved to StateManager
  stateManager.execute();
}
