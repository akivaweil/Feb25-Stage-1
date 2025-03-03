#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>

/*
 * AUTOMATED TABLE SAW CONTROL SYSTEM
 * 
 * SAFETY NOTICE: PLEASE DO NOT DELETE OR MODIFY ANYTHING HERE
 * This code controls an automated table saw cutting system. Safety is the absolute priority.
 * - Code clarity and reliability take precedence over processing efficiency
 * - All functions are written to be as explicit and straightforward as possible
 * - Hardware emergency stop switch cuts ALL power to the system when activated
 * - Multiple software safety checks are implemented throughout the cycle
 * - All switches and buttons read HIGH when activated
 * - All cylinders require a HIGH output to disengage
 * - Bounce2 library is used for switch debouncing with a 20ms debounce time
 * - All code should be very very easy to understand for a beginner programmer
 * - my switches are configured where one side is connected to 5v and the other side splits into 10k resistor to ground at its signal pin
*- minimum pulse width through the accelstepper library should be 3ms
*- I am using a CL57T Driver for my closed loop motors and the alarm is active low
 */

/* ===============================================================================
 * IMPORTANT NOTE ABOUT SERIAL DEBUGGING:
 * All Serial.print() and Serial.println() statements have been commented out to improve 
 * performance. DO NOT DELETE these statements - they should be kept for debugging purposes.
 * Simply uncomment them when debugging is needed.
 * ===============================================================================
 */

/* ===============================================================================
* UPDATE THE INSTRUCTIONS BELOW AS YOU CHANGE THE CODE. IF THE CODE AND INSTRUCTIONS ARE DIFFERENT THEN PRIORITIZE THE INSTRUCTIONS.
   🔄 CUTTING CYCLE SEQUENCE

   1. 🏁 Initial Position Check:
      - 🔴🔴 Both clamps engaged (engaged = LOW)
      - 🎯 Cut motor at home position (0)
      - 📐 Position motor at home position (0)

   2. 🚀 Cycle Start:
      - 🔴🔴 Ensure both clamps are engaged
      - ⏱️ Wait for clamp operation verification

   3. 🔪 Cut Sequence:
      - 🔴🔴 Ensure both clamps remain engaged
      - 🎯 Move cut motor forward (9 inches, ⚡100, 📈2000)
      - ⏱️ Wait for cut completion

   4. 🏠 Return Sequence:
      - 🔵 Retract position clamp
      - ⏱️ Wait for clamp operation
      - 🎯 Return cut motor (⚡1250, 📈2000)
      - 🎯 Return position motor (⚡12000, 📈10000)

   5. 📏 Position Sequence:
      - 🎯 Move position motor (3.35 inches, ⚡12000, 📈5000)
      - 🔴 Re-engage the position clamp
      - ⏱️ Wait for stabilization
============================================================================== */

/* ===============================================================================
   🛑 SAFETY NOTICE: DO NOT MODIFY WITHOUT APPROVAL

   AUTOMATED TABLE SAW CONTROL SYSTEM
   - Safety is the absolute priority.
   - All cylinders require a HIGH output to disengage (retracted) and LOW to engage (extended).
   - All switches and buttons read HIGH when activated.
   - Bounce2 is used for debouncing (20ms debounce time).
============================================================================== */

/* ===============================================================================
   📝 SYSTEM DOCUMENTATION

   (See header comments above for full cycle sequence details.)

   ⚠️ Safety Features:
   - Hardware emergency stop cuts all power.
   - Multiple software safety checks.
   - Debounced switch readings.
   - Explicit clamp state verification.
============================================================================== */

/* ===============================================================================
   🏠 HOMING SEQUENCE

   1. Report initial switch states.
   2. For the cut motor:
         - Move toward the home switch (⚡250, 📈2000).
         - Stop immediately when the (active HIGH) switch is detected.
         - Set position to zero.
   3. For the position motor:
         - Disengage the position clamp during homing.
         - Move until the (active HIGH) switch is triggered.
         - Set the current position to zero and re-engage the clamp.
   4. Verify that both motors are homed.
============================================================================== */

/* ===============================================================================
   🔧 HARDWARE CONFIGURATION NOTES
   (If there is any conflict between code and instructions, the instructions take priority.)

   MOTOR CONFIGURATIONS:
   1. Cut Motor (Stepper)
      - PUL: 22, DIR: 23
      - Steps per inch: 76 (GT2 belt with 30-tooth pulley)
      - Normal speed: 100 steps/sec
      - Return speed: 1250 steps/sec
      - Acceleration: 2000 steps/sec²
      - Travel distance: 9 inches
      - Homing direction: -1

   2. Position Motor (Stepper)
      - PUL: 32, DIR: 33
      - Steps per inch: 1015 (ball screw)
      - Normal speed: 1000 steps/sec
      - Return speed: 12000 steps/sec (doubled from 6000)
      - Return acceleration: 10000 steps/sec² (doubled from 5000)
      - Final 3.35" movement speed: 12000 steps/sec (doubled from 6000)
      - Acceleration: 5000 steps/sec²
      - Travel distance: 3.35 inches
      - Homing direction: -1  ← (Reversed as requested)

   SWITCH CONFIGURATIONS:
   1. Cut Motor Position Switch
      - Pin: 25
      - Type: Limit switch, **Active: HIGH**
      - Debounce: 20ms

   2. Position Motor Position Switch
      - Pin: 27
      - Type: Limit switch, Active: HIGH
      - Debounce: 20ms

   3. Reload Switch
      - Pin: 14
      - Type: Manual switch, Active: HIGH
      - Debounce: 20ms

   4. Start Cycle Switch
      - Pin: 18
      - Type: Manual switch, Active: HIGH
      - Debounce: 20ms

   CYLINDER (CLAMP) CONFIGURATIONS:
   1. Position Clamp
      - Pin: 13
      - HIGH = Disengaged (retracted), LOW = Engaged (extended)
      - Operation delay: 50ms

   2. Wood Secure Clamp
      - Pin: 15
      - HIGH = Disengaged (retracted), LOW = Engaged (extended)
      - Operation delay: 50ms

   LED INDICATOR CONFIGURATIONS:
   1. Red LED (Error)
      - Pin: 26
      - Indicates error states
      - Active: HIGH

   2. Yellow LED (Busy)
      - Pin: 21
      - Indicates cutting/working states
      - Active: HIGH

   3. Green LED (Ready)
      - Pin: 4
      - Indicates system ready
      - Active: HIGH

   4. Blue LED (Setup)
      - Pin: 2
      - Indicates startup/homing
      - Active: HIGH

   LED State Meanings:
   - Blue: System is in startup or homing
   - Green: System is ready for operation
   - Yellow: System is actively cutting/working
   - Red: System has encountered an error
============================================================================== */

// ---------------------
// Pin Definitions
// ---------------------

// Motors
const int PIN_CUT_MOTOR_PUL      = 22;
const int PIN_CUT_MOTOR_DIR      = 23;
const int PIN_POSITION_MOTOR_PUL = 32;
const int PIN_POSITION_MOTOR_DIR = 33;

// Switches
const int PIN_CUT_MOTOR_POSITION_SWITCH    = 25;
const int PIN_POSITION_MOTOR_POSITION_SWITCH = 27;
const int PIN_RELOAD_SWITCH                = 14;
const int PIN_START_CYCLE_SWITCH           = 18;

// Clamps
const int PIN_POSITION_CLAMP      = 13;
const int PIN_SECURE_WOOD_CLAMP   = 15;

// Clamp States:
// HIGH output = Disengaged (retracted)
// LOW output  = Engaged (extended)
const int CLAMP_ENGAGED    = LOW;
const int CLAMP_DISENGAGED = HIGH;

// New signal pin for Stage 2 signaling
const int SIGNAL_1TO2_PIN = 19;  // 1to2 signal pin (changed from pin 21)

// <<< NEW: Wood Sensor >>>
const int PIN_WOOD_SENSOR = 34;  // Wood sensor input pin
Bounce woodSensor = Bounce();

// Additional pins
// If pins 0 and 2 are not used and do not interfere with boot mode,
// you can comment them out.
// pinMode(BOOT_PIN, INPUT_PULLUP);    // Enable internal pullup on BOOT pin
// pinMode(EN_PIN, OUTPUT);            // Configure Enable pin
// digitalWrite(EN_PIN, HIGH);         // Enable the ESP32

// ---------------------
// LED Indicators
// ---------------------
const int PIN_RED_LED = 26;     // Red LED for error states
const int PIN_YELLOW_LED = 21;  // Yellow LED for busy/working states
const int PIN_GREEN_LED = 4;    // Green LED for ready state
const int PIN_BLUE_LED = 2;     // Blue LED for homing/setup states

// ---------------------
// Homing and Operation Parameters
// ---------------------
const int HOME_SPEED = 2000;               // Base homing speed (steps/sec)
const int HOME_DIRECTION = -1;             // Cut motor homing direction (-1)
// Position motor homing direction is reversed (now -1)
const int POSITION_HOME_DIRECTION = -1;    
const int DEBOUNCE_INTERVAL = 10;          // Debounce time (ms) - changed from 50ms to 10ms

// Updated steps per inch for the cut motor with 30-tooth pulley
// Original value was 127 steps per inch
const int STEPS_PER_INCH = 76;            // Cut motor steps per inch (adjusted for 30-tooth pulley)
const int POSITION_STEPS_PER_INCH = 1000;  // Position motor steps per inch
const float CUT_MOTOR_TRAVEL = 9;          // 9 inches cut travel
const float POSITION_MOTOR_TRAVEL = 3.35;  // 3.35 inches position travel
const unsigned long CLAMP_OPERATION_DELAY = 50;  // 50ms clamp delay

// Motor speeds and accelerations
const float CUT_MOTOR_SPEED = 150;
//SAVE: const float CUT_MOTOR_SPEED = 100;
const float CUT_MOTOR_RETURN_SPEED = 15000;
const float POSITION_MOTOR_SPEED = 50000;
const float POSITION_MOTOR_RETURN_SPEED = 20000;
const float CUT_MOTOR_ACCEL = 3000;
const float POSITION_MOTOR_ACCEL = 15000;

// ---------------------
// Motor Controllers
// ---------------------
AccelStepper cutMotor(AccelStepper::DRIVER, PIN_CUT_MOTOR_PUL, PIN_CUT_MOTOR_DIR);
AccelStepper positionMotor(AccelStepper::DRIVER, PIN_POSITION_MOTOR_PUL, PIN_POSITION_MOTOR_DIR);

// ---------------------
// Switch Debouncers
// ---------------------
Bounce cutSwitchDebouncer = Bounce();
Bounce positionSwitchDebouncer = Bounce();
Bounce reloadSwitch = Bounce();
Bounce startCycleSwitch = Bounce();

// ---------------------
// System State Variables and Enum
// ---------------------
bool cutMotorHomed = false;
bool positionMotorHomed = false;
bool startupComplete = false;
bool initialStateReported = false;
bool inReloadMode = false;  // Track reload mode state
bool noWoodMode = false;  // Track no-wood mode state

enum SystemState {
    STARTUP,
    HOMING,
    READY,
    CUTTING,
    ERROR
};

SystemState currentState = STARTUP;
String lastError = "";

// Timer for clamp sequencing
unsigned long secureClampTimer = 0;
bool secureClampRetractPending = false;
const unsigned long SECURE_CLAMP_DELAY = 200;  // 100ms delay

// ---------------------
// Function Prototypes
// ---------------------
void configureSerial();
void configureSwitches();
void configureMotorsForHoming();
void configureClamps();
void printSetupComplete();
void updateSwitches();
void reportInitialStateAndCheckHoming();
void handleCutMotorHoming();
void handlePositionMotorHoming();
void runHomingSequence();
void performCutCycle();
void handleReloadSwitch();
void updateSystemState();
void handleError(const String& errorMessage);
bool verifyPosition();
void reportSystemStatus();
bool validateConfiguration();
bool moveToPosition(AccelStepper& motor, long position, float speed, float accel);
void updateLEDs();
void sendSignalToStage2();
void reportError(String errorMessage);

void setup() {
    // Configure LED pins
    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_YELLOW_LED, OUTPUT);
    pinMode(PIN_GREEN_LED, OUTPUT);
    pinMode(PIN_BLUE_LED, OUTPUT);
    
    // All LEDs off initially
    digitalWrite(PIN_RED_LED, LOW);
    digitalWrite(PIN_YELLOW_LED, LOW);
    digitalWrite(PIN_GREEN_LED, LOW);
    digitalWrite(PIN_BLUE_LED, LOW);
    
    // Continue with normal setup
    configureSerial();
    
    // Remove unnecessary delay
    // Serial.println("Serial initialized and ready");
    
    configureSwitches();
    configureMotorsForHoming();
    configureClamps();
    
    // Configure the wood sensor on pin 34 (using internal pullup; sensor is active LOW)
    pinMode(PIN_WOOD_SENSOR, INPUT_PULLUP);
    woodSensor.attach(PIN_WOOD_SENSOR, INPUT_PULLUP);
    woodSensor.interval(DEBOUNCE_INTERVAL);
    // Serial.println("Wood sensor configured on pin 34 (active LOW)");
    
    // Configure the 1to2 signal pin
    pinMode(SIGNAL_1TO2_PIN, OUTPUT);
    digitalWrite(SIGNAL_1TO2_PIN, HIGH);  // Inactive by default
    // Serial.println("Signal pin configured");
    
    printSetupComplete();
    if (!validateConfiguration()) {
        while (1) { delay(1000); }  // Halt system if configuration fails
    }

    // Remove unnecessary delay before starting homing sequence
    // Serial.println("Starting homing sequence...");
}

void loop() {
    // Debug LED to show the loop is running - only if not in homing state
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500 && currentState != HOMING) {
        digitalWrite(PIN_BLUE_LED, !digitalRead(PIN_BLUE_LED));
        lastBlink = millis();
        
        // Reduced debug output - only print every 5 seconds
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint > 5000) {
            // Serial.print("Switch states: Cut=");
            // Serial.print(cutSwitchDebouncer.read());
            // Serial.print(" Pos=");
            // Serial.print(positionSwitchDebouncer.read());
            // Serial.print(" Reload=");
            // Serial.print(reloadSwitch.read());
            // Serial.print(" Start=");
            // Serial.println(startCycleSwitch.read());
            lastDebugPrint = millis();
        }
    }
    
    updateSwitches();
    
    // Report initial state and check for homing (once only)
    if (!initialStateReported) {
        reportInitialStateAndCheckHoming();
        initialStateReported = true;
    }
    
    // Run the homing sequence until complete
    if (!startupComplete) {
        runHomingSequence();
        // Don't call updateLEDs() here, as runHomingSequence() handles LEDs directly
    } else {
        // System is ready, handle normal operation
        if (currentState == READY) {
            // Always check the reload switch state
            handleReloadSwitch();
            
            // Directly check if the start cycle switch is HIGH
            if (startCycleSwitch.read() == HIGH) {
                // Serial.println("Start cycle switch active, initiating cycle sequence");
                // Continue running cycles as long as the switch remains HIGH
                while (startCycleSwitch.read() == HIGH) {
                    updateSwitches();
                    // Extra check added here so a new cycle doesn't start if the switch is off
                    if (startCycleSwitch.read() != HIGH) {
                        // Serial.println("Run cycle switch turned off. Aborting cycle initiation.");
                        break;
                    }
                    // Serial.println("Starting cut cycle...");
                    performCutCycle();
                    updateSwitches();
                    delay(100);
                    if (reloadSwitch.read() == HIGH)
                        break;
                }
            }
        }
    }
    
    // Update system state and report status periodically
    updateSystemState();
    reportSystemStatus();
    
    // Update LEDs after state changes - only if not in homing
    if (currentState != HOMING) {
        updateLEDs();
    }
}

// ---------------------
// Configuration Functions
// ---------------------
void configureSerial() {
    Serial.begin(115200);
    // Minimal delay for serial initialization - 50ms is typically sufficient
    delay(50);
    // Serial.println("----------------------------------------");
    // Serial.println("Serial communication initialized");
    // Serial.println("AUTOMATED TABLE SAW CONTROL SYSTEM");
    // Serial.println("----------------------------------------");
    
    // Configure LED pins
    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_YELLOW_LED, OUTPUT);
    pinMode(PIN_GREEN_LED, OUTPUT);
    pinMode(PIN_BLUE_LED, OUTPUT);
    
    // All LEDs off initially
    digitalWrite(PIN_RED_LED, LOW);
    digitalWrite(PIN_YELLOW_LED, LOW);
    digitalWrite(PIN_GREEN_LED, LOW);
    digitalWrite(PIN_BLUE_LED, LOW);
    
    // Serial.println("LED indicators configured");
}

void configureSwitches() {
    // Configure switches with explicit pullup resistors
    pinMode(PIN_CUT_MOTOR_POSITION_SWITCH, INPUT_PULLUP);
    cutSwitchDebouncer.attach(PIN_CUT_MOTOR_POSITION_SWITCH, INPUT_PULLUP);
    cutSwitchDebouncer.interval(DEBOUNCE_INTERVAL);
    // Serial.println("Cut motor position switch configured with pullup");
    
    pinMode(PIN_POSITION_MOTOR_POSITION_SWITCH, INPUT_PULLUP);
    positionSwitchDebouncer.attach(PIN_POSITION_MOTOR_POSITION_SWITCH, INPUT_PULLUP);
    positionSwitchDebouncer.interval(DEBOUNCE_INTERVAL);
    // Serial.println("Position motor position switch configured with pullup");
    
    pinMode(PIN_RELOAD_SWITCH, INPUT_PULLUP);
    reloadSwitch.attach(PIN_RELOAD_SWITCH, INPUT_PULLUP);
    reloadSwitch.interval(DEBOUNCE_INTERVAL);
    // Serial.println("Reload switch configured with pullup");
    
    pinMode(PIN_START_CYCLE_SWITCH, INPUT_PULLUP);
    startCycleSwitch.attach(PIN_START_CYCLE_SWITCH, INPUT_PULLUP);
    startCycleSwitch.interval(DEBOUNCE_INTERVAL);
    // Serial.println("Start cycle switch configured with pullup");
    
    // Force an initial update of all switches
    cutSwitchDebouncer.update();
    positionSwitchDebouncer.update();
    reloadSwitch.update();
    startCycleSwitch.update();
    
    // Print initial switch states
    // Serial.print("Initial switch states - Cut: ");
    // Serial.print(cutSwitchDebouncer.read());
    // Serial.print(", Position: ");
    // Serial.print(positionSwitchDebouncer.read());
    // Serial.print(", Reload: ");
    // Serial.print(reloadSwitch.read());
    // Serial.print(", Start: ");
    // Serial.println(startCycleSwitch.read());
}

void configureMotorsForHoming() {
    // Configure cut motor for homing with smoother acceleration curve
    float homingSpeed = HOME_SPEED * 1.5; // Increase speed by 50%
    cutMotor.setMaxSpeed(homingSpeed / 2);
    cutMotor.setAcceleration(CUT_MOTOR_ACCEL / 4); // Even gentler acceleration - reduced from /2 to /4
    cutMotor.moveTo(100000 * HOME_DIRECTION);
    // Serial.println("Cut motor configured for homing with smooth acceleration curve");
    
    // Configure position motor for homing with proper acceleration curve
    positionMotor.setMaxSpeed(homingSpeed / 1.5);
    positionMotor.setAcceleration(POSITION_MOTOR_ACCEL / 2); // Gentler acceleration
    positionMotor.moveTo(100000 * POSITION_HOME_DIRECTION);
    // Serial.println("Position motor configured for homing");
    
    // Serial.println("Motors enabled and ready to move");
}

void configureClamps() {
    pinMode(PIN_POSITION_CLAMP, OUTPUT);
    pinMode(PIN_SECURE_WOOD_CLAMP, OUTPUT);
    
    // Both clamps engaged (LOW) initially
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
    // Serial.println("Clamps configured - Both clamps engaged (LOW) initially");
}

void printSetupComplete() {
    // Serial.println("----------------------------------------");
    // Serial.println("Setup complete - Beginning homing sequence");
    // Serial.println("----------------------------------------");
}

// ---------------------
// Switch and State Update Functions
// ---------------------
void updateSwitches() {
    cutSwitchDebouncer.update();
    positionSwitchDebouncer.update();
    reloadSwitch.update();
    startCycleSwitch.update();
    woodSensor.update();  // Make sure wood sensor is also updated
}

void reportInitialStateAndCheckHoming() {
    // Serial.println("\nInitial State Report:");
    
    // For the cut motor, the switch is active when HIGH (pressed).
    // Serial.print("Cut Motor Position Switch State: ");
    // Serial.println(cutSwitchDebouncer.read() == HIGH ? "ACTIVATED" : "NOT ACTIVATED");
    
    // Serial.print("Position Motor Position Switch State: ");
    // Serial.println(positionSwitchDebouncer.read() == HIGH ? "ACTIVATED" : "NOT ACTIVATED");
    
    // Serial.print("Reload Switch State: ");
    // Serial.println(reloadSwitch.read() == HIGH ? "ACTIVATED" : "NOT ACTIVATED");
    // Serial.println("----------------------------------------");
    
    // Set system state to HOMING if either motor needs to be homed
    if (!cutMotorHomed || !positionMotorHomed) {
        currentState = HOMING;
        // Serial.println("Entering HOMING state");
        
        // Initialize LEDs for homing - turn off all LEDs except green
        digitalWrite(PIN_RED_LED, LOW);
        digitalWrite(PIN_YELLOW_LED, LOW);
        digitalWrite(PIN_BLUE_LED, LOW);
        digitalWrite(PIN_GREEN_LED, HIGH);  // Start with green LED on
        // Serial.println("Green LED initialized for homing sequence");
    }
    
    if (cutSwitchDebouncer.read() == HIGH) {
        cutMotorHomed = true;
        cutMotor.setSpeed(0);
        cutMotor.setCurrentPosition(0);
        // Serial.println("Cut motor already at home position");
    } else {
        cutMotor.setMaxSpeed(HOME_SPEED / 8);
        cutMotor.setSpeed((HOME_SPEED / 8) * HOME_DIRECTION);
    }
    
    if (positionSwitchDebouncer.read() == HIGH) {
        positionMotorHomed = true;
        positionMotor.setSpeed(0);
        positionMotor.setCurrentPosition(0);  // Set to actual physical home
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        // Serial.println("Position motor already at home position");
    } else {
        positionMotor.setMaxSpeed(HOME_SPEED);
        positionMotor.setSpeed(HOME_SPEED * POSITION_HOME_DIRECTION);
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
    }
    
    if (cutMotorHomed && positionMotorHomed) {
        // Serial.println("Both motors homed - System is now READY");
        
        // Wait until the run cycle switch is released to avoid immediate cycle start
        updateSwitches();
        while (startCycleSwitch.read() == HIGH) {
            // Serial.println("Run cycle switch is active at homing completion; waiting for release...");
            updateSwitches();
            delay(50);
        }
        
        startupComplete = true;
        currentState = READY;
    }
}

void handleCutMotorHoming() {
    static int cutHomingStage = 0;  // 0: waiting for switch activation, 1: moving away, 2: done
    if (!cutMotorHomed) {
        // Stage 0: wait for cut motor home switch activation (switch reads HIGH)
        if (cutHomingStage == 0) {
            if (cutSwitchDebouncer.read() == HIGH) {
                // Stop motor and mark current position as home
                cutMotor.stop();
                cutMotor.setSpeed(0);
                // Serial.println("Cut motor home switch activated!");
                cutMotor.setCurrentPosition(0);
                // Remove the offset - don't move away from home switch
                cutMotorHomed = true;
                // Serial.println("Cut motor homed at switch position");
                cutHomingStage = 2; // finished - skip the move-away phase
            } else {
                cutMotor.run();
            }
        }
        // Stage 1: non-blocking move-away phase - now skipped
        else if (cutHomingStage == 1) {
            cutMotor.run();
            // When the target is reached, zero the position and mark homed
            if (cutMotor.distanceToGo() == 0) {
                cutMotor.setCurrentPosition(0);
                cutMotorHomed = true;
                // Serial.println("Cut motor homed and moved to safe position");
                cutHomingStage = 2; // finished
            }
        }
    }
}

void handlePositionMotorHoming() {
    static int posHomingStage = 0;  // 0: waiting for home switch; 1: moving to final position; 2: done
    if (!positionMotorHomed) {
        // Disengage the clamp during homing
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        
        // Stage 0: Wait until the position home switch is activated
        if (posHomingStage == 0) {
            if (positionSwitchDebouncer.read() == HIGH) {
                positionMotor.stop();
                // Serial.println("Position motor home switch activated!");
                // Set the current position to -1 inch (using steps)
                positionMotor.setCurrentPosition(-POSITION_STEPS_PER_INCH);
                // Serial.println("Setting position to -1; commanding move to final target (3.35 inches)");
                // Immediately command move to final position 3.35 inches (absolute)
                positionMotor.setMaxSpeed(POSITION_MOTOR_SPEED);
                positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
                positionMotor.moveTo(POSITION_STEPS_PER_INCH * 3.35);
                posHomingStage = 1;
            } else {
                positionMotor.run();
            }
        }
        // Stage 1: Process the move to final position (3.35 inches)
        else if (posHomingStage == 1) {
            positionMotor.run();
            if (positionMotor.distanceToGo() == 0) {
                digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
                positionMotorHomed = true;
                // Serial.println("Position motor homing complete; final position reached (3.35 inches)");
                posHomingStage = 2;
            }
        }
    }
}

void runHomingSequence() {
    static unsigned long homingStartTime = millis();
    static bool homingTimeoutReported = false;
    
    // Set system state to HOMING
    currentState = HOMING;
    
    // Handle green LED blinking once per second during homing
    static unsigned long lastLedBlinkTime = 0;
    static bool ledState = false;
    
    // Force all other LEDs off and control green LED directly
    digitalWrite(PIN_RED_LED, LOW);
    digitalWrite(PIN_YELLOW_LED, LOW);
    digitalWrite(PIN_BLUE_LED, LOW);
    
    // Blink green LED once per second
    if (millis() - lastLedBlinkTime >= 1000) {  // Blink once per second
        ledState = !ledState;
        digitalWrite(PIN_GREEN_LED, ledState);
        lastLedBlinkTime = millis();
        
        // Debug message to confirm LED state
        // Serial.print("Green LED: ");
        // Serial.println(ledState ? "ON" : "OFF");
    }
    
    // Reduced debug output - only print every 5 seconds
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 5000) {
        // Serial.print("Homing status: ");
        // Serial.print(cutMotorHomed ? "Cut DONE" : "Cut PENDING");
        // Serial.print(", ");
        // Serial.println(positionMotorHomed ? "Position DONE" : "Position PENDING");
        lastDebugTime = millis();
    }
    
    // Add a timeout for homing (2 minutes)
    if (!startupComplete && (millis() - homingStartTime > 120000) && !homingTimeoutReported) {
        // Serial.println("WARNING: Homing sequence timeout. Check switch connections.");
        homingTimeoutReported = true;
        
        // Force completion to allow manual operation
        startupComplete = true;
        currentState = READY;
        // Serial.println("Forcing system to READY state to allow manual operation");
    }
    
    // Only run homing if the reload switch is not active
    if (!startupComplete && reloadSwitch.read() == LOW) {
        if (!cutMotorHomed) {
            handleCutMotorHoming();
        }
        if (!positionMotorHomed) {
            handlePositionMotorHoming();
        }
    } else if (!startupComplete && reloadSwitch.read() == HIGH) {
        // Print this message less frequently
        static unsigned long lastReloadMsg = 0;
        if (millis() - lastReloadMsg > 3000) {
            // Serial.println("Release reload switch to begin homing.");
            lastReloadMsg = millis();
        }
    }
    
    // If both motors are homed, mark startup as complete
    if (cutMotorHomed && positionMotorHomed && !startupComplete) {
        // Serial.println("Both motors homed - System is now READY");
        
        // Wait until the run cycle switch is released to avoid immediate cycle start
        updateSwitches();
        while (startCycleSwitch.read() == HIGH) {
            // Serial.println("Run cycle switch is active at homing completion; waiting for release...");
            updateSwitches();
            delay(50);
        }
        
        startupComplete = true;
        currentState = READY;
    }
    
    // Manual override for testing - if both switches are pressed for 5 seconds
    static unsigned long bothSwitchesActiveTime = 0;
    if (!startupComplete && cutSwitchDebouncer.read() == HIGH && positionSwitchDebouncer.read() == HIGH) {
        if (bothSwitchesActiveTime == 0) {
            bothSwitchesActiveTime = millis();
            // Serial.println("Both switches active - starting override timer");
        } else if (millis() - bothSwitchesActiveTime > 5000) {
            // Serial.println("MANUAL OVERRIDE: Both switches held for 5 seconds");
            cutMotorHomed = true;
            positionMotorHomed = true;
            startupComplete = true;
            currentState = READY;
            
            // Set safe positions
            cutMotor.setCurrentPosition(0);
            positionMotor.setCurrentPosition(POSITION_STEPS_PER_INCH * POSITION_MOTOR_TRAVEL);
            
            // Engage clamps
            digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
            digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
            
            // Serial.println("System forced to READY state by manual override");
        }
    } else {
        bothSwitchesActiveTime = 0; // Reset if both switches aren't active
    }
}

// ---------------------
// Cutting Cycle Functions
// ---------------------
void performCutCycle() {
    // Serial.println("Starting cut cycle sequence");
    
    // Check wood sensor immediately at cycle start
    // Update the wood sensor reading multiple times to ensure accuracy
    for (int i = 0; i < 3; i++) {
        woodSensor.update();
        delay(5); // Small delay between readings
    }
    
    // Check if wood is present
    bool woodPresent = (woodSensor.read() == LOW);
    // Serial.print("Wood sensor reading at cycle start: ");
    // Serial.println(woodPresent ? "LOW (wood present)" : "HIGH (no wood)");
    
    // Update mode flag immediately based on current sensor reading
    noWoodMode = !woodPresent;
    
    // Update LEDs to reflect the current mode
    updateLEDs();
    
    // Configure the cut motor for normal operation
    cutMotor.setMaxSpeed(CUT_MOTOR_SPEED);
    cutMotor.setAcceleration(CUT_MOTOR_ACCEL);
    
    // Ensure both clamps are engaged during cutting
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
    delay(CLAMP_OPERATION_DELAY); // Keep this delay for safety
    
    // Move the cut motor forward to execute the cut
    cutMotor.moveTo(STEPS_PER_INCH * CUT_MOTOR_TRAVEL);
    while (cutMotor.distanceToGo() != 0) {
        cutMotor.run();
    }
    
    // Recheck wood sensor after cut is complete (for consistency with previous behavior)
    // Update the wood sensor reading multiple times to ensure accuracy
    for (int i = 0; i < 3; i++) {
        woodSensor.update();
        delay(5); // Small delay between readings
    }
    
    // Final reading to confirm wood presence
    woodPresent = (woodSensor.read() == LOW);
    // Serial.print("Wood sensor reading after cut: ");
    // Serial.println(woodPresent ? "LOW (wood present)" : "HIGH (no wood)");
    
    // Update mode flag based on current sensor reading
    noWoodMode = !woodPresent;
    
    // Update LEDs to reflect the current mode
    updateLEDs();
    
    if (woodPresent) {
        // Serial.println("Wood detected; proceeding with normal cycle");
        
        // STEP 1: Send signal to Stage 2 (non-blocking)
        sendSignalToStage2();
        unsigned long signalStartTime = millis();
        
        // --- NEW SEQUENCE START ---
        // Instead of immediately retracting the secure wood clamp, allow the position motor to start dragging for 50ms.
        // Command the position motor to return home (to drag the wood).
        positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
        positionMotor.moveTo(0);  // Command position motor to return home
        
        // Ensure the position clamp remains engaged initially for proper wood drag.
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        
        // Set a 50ms delay target for when the secure wood clamp should retract.
        unsigned long secureClampRetractionDelay = 100; // in milliseconds
        unsigned long clampRetractionTime = millis() + secureClampRetractionDelay;
        bool secureClampRetracted = false;
        
        // Capture the starting position for the return sequence
        long returnStartPos = positionMotor.currentPosition();
        
        // Serial.println("Waiting for position motor to drag wood back 0.1 inches and 50ms delay...");
        // Wait until the position motor has moved 0.1 inch (in steps)
        while ((returnStartPos - positionMotor.currentPosition()) < (0.1 * POSITION_STEPS_PER_INCH)) {
            positionMotor.run();
            // After 50ms has elapsed, retract secure wood clamp (if not already retracted)
            if (!secureClampRetracted && (millis() >= clampRetractionTime)) {
                // Serial.println("50ms elapsed; retracting secure wood clamp.");
                digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
                secureClampRetracted = true;
            }
            if (millis() - signalStartTime >= 1000) {
                sendSignalToStage2();
                signalStartTime = millis();
            }
        }
        // --- NEW SEQUENCE END ---
        
        // STEP 4: After dragging wood back 0.1 inches, retract the position clamp for the remaining move to home.
        // Serial.println("0.1 inch drag complete. Retracting position clamp for remaining home travel.");
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        
        // Now start the cut motor's return.
        cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
        cutMotor.moveTo(0);
        
        // Serial.println("Returning both motors to home");
        bool positionMotorDone = false;
        bool cutMotorDone = false;
        while (!positionMotorDone || !cutMotorDone) {
            if (!positionMotorDone) {
                positionMotor.run();
                if (positionMotor.distanceToGo() == 0) {
                    positionMotorDone = true;
                    // Serial.println("Position motor reached home");
                }
            }
            if (!cutMotorDone) {
                cutMotor.run();
                if (cutMotor.distanceToGo() == 0) {
                    cutMotorDone = true;
                    // Serial.println("Cut motor reached home");
                    
                    // SAFETY CHECK: Immediately verify cut motor is actually at home position
                    updateSwitches(); // Ensure switch readings are current
                    if (cutSwitchDebouncer.read() != HIGH) {
                        // Cut motor position switch is not activated - potential belt slippage
                        // Serial.println("ERROR: Cut motor not at home position! Possible belt slippage detected.");
                        
                        // Stop position motor immediately if it's still running
                        if (!positionMotorDone) {
                            positionMotor.stop();
                            positionMotorDone = true;
                            // Serial.println("Position motor stopped due to cut motor position error");
                        }
                        
                        // Set error state with blinking red and yellow LEDs
                        bool errorAcknowledged = false;
                        bool ledState = false;
                        unsigned long lastBlinkTime = 0;
                        const unsigned long BLINK_INTERVAL = 250; // 250ms for fast blinking
                        
                        while (!errorAcknowledged) {
                            // Blink red and yellow LEDs alternately
                            if (millis() - lastBlinkTime > BLINK_INTERVAL) {
                                ledState = !ledState;
                                digitalWrite(PIN_RED_LED, ledState);
                                digitalWrite(PIN_YELLOW_LED, !ledState);
                                lastBlinkTime = millis();
                            }
                            
                            // Check for reset condition (reload switch activation)
                            updateSwitches();
                            if (reloadSwitch.read() == HIGH) {
                                delay(500); // Debounce
                                updateSwitches();
                                if (reloadSwitch.read() == HIGH) {
                                    errorAcknowledged = true;
                                    // Serial.println("Error acknowledged via reload switch. Resetting system.");
                                }
                            }
                        }
                        
                        // Turn off error LEDs
                        digitalWrite(PIN_RED_LED, LOW);
                        digitalWrite(PIN_YELLOW_LED, LOW);
                        
                        // Return to ready state without moving position motor forward
                        currentState = READY;
                        return;
                    }
                }
            }
            if (millis() - signalStartTime >= 1000) {
                sendSignalToStage2();
                signalStartTime = millis();
            }
        }
        
        // STEP 5: After both motors are home, re-engage the position clamp as part of final positioning.
        // Serial.println("Both motors at home, engaging position clamp");
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        
        // STEP 6: Wait an additional 100ms before starting the final move.
        delay(100);
        
        // SAFETY CHECK: Verify cut motor is at home position before moving position motor forward
        updateSwitches(); // Ensure switch readings are current
        if (cutSwitchDebouncer.read() != HIGH) {
            // Cut motor position switch is not activated - potential belt slippage
            // Serial.println("ERROR: Cut motor not at home position! Possible belt slippage detected.");
            
            // Set error state with blinking red and yellow LEDs
            bool errorAcknowledged = false;
            bool ledState = false;
            unsigned long lastBlinkTime = 0;
            const unsigned long BLINK_INTERVAL = 250; // 250ms for fast blinking
            
            while (!errorAcknowledged) {
                // Blink red and yellow LEDs alternately
                if (millis() - lastBlinkTime > BLINK_INTERVAL) {
                    ledState = !ledState;
                    digitalWrite(PIN_RED_LED, ledState);
                    digitalWrite(PIN_YELLOW_LED, !ledState);
                    lastBlinkTime = millis();
                }
                
                // Check for reset condition (reload switch activation)
                updateSwitches();
                if (reloadSwitch.read() == HIGH) {
                    delay(500); // Debounce
                    updateSwitches();
                    if (reloadSwitch.read() == HIGH) {
                        errorAcknowledged = true;
                        // Serial.println("Error acknowledged via reload switch. Resetting system.");
                    }
                }
            }
            
            // Turn off error LEDs
            digitalWrite(PIN_RED_LED, LOW);
            digitalWrite(PIN_YELLOW_LED, LOW);
            
            // Return to ready state without moving position motor forward
            currentState = READY;
            return;
        }
        
        // STEP 7: Move position motor to final position (3.45 inches target).
        // Serial.println("Moving position motor to 3.45\"");
        positionMotor.setMaxSpeed(POSITION_MOTOR_SPEED);
        positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
        positionMotor.moveTo(POSITION_STEPS_PER_INCH * 3.45);
        while (positionMotor.distanceToGo() != 0) {
            positionMotor.run();
        }
        
        // Update system state
        currentState = READY;
        // Serial.println("Cut cycle complete - Ready for next cycle");
        
        // Quick update of switches
        updateSwitches();
    } else {
        // No-wood mode handling - streamlined
        // Serial.println("No wood detected; activating no-wood mode");
        
        // Configure clamps for no-wood mode
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
        
        // Set up motors for return
        cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
        positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
        
        // Signal Stage 2
        sendSignalToStage2();
        unsigned long signalStartTime = millis();
        
        // Return both motors simultaneously
        cutMotor.moveTo(0);
        positionMotor.moveTo(POSITION_STEPS_PER_INCH * 0.5);
        
        while (cutMotor.distanceToGo() != 0 || positionMotor.distanceToGo() != 0) {
            if (millis() - signalStartTime >= 1000) {
                sendSignalToStage2();
                signalStartTime = millis();
            }
            if (cutMotor.distanceToGo() != 0)
                cutMotor.run();
            if (positionMotor.distanceToGo() != 0)
                positionMotor.run();
        }
        
        // Serial.println("Motors returned to home positions");
        
        // Add the requested movements for no wood mode
        // 1. Retract the position clamp
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        delay(CLAMP_OPERATION_DELAY);
        
        // 2. Move the position motor to 3.45 inches
        positionMotor.setMaxSpeed(POSITION_MOTOR_SPEED);
        positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
        positionMotor.moveTo(POSITION_STEPS_PER_INCH * 3.45);
        while (positionMotor.distanceToGo() != 0) {
            positionMotor.run();
        }
        
        // 3. Extend the position clamp
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        delay(CLAMP_OPERATION_DELAY);
        
        // 4. Move the position motor back to home position
        positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
        positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
        positionMotor.moveTo(0);
        while (positionMotor.distanceToGo() != 0) {
            positionMotor.run();
        }
        
        // 5. Retract the position clamp
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        delay(CLAMP_OPERATION_DELAY);
        
        // Serial.println("Waiting for switch input...");
        
        // Wait for user input
        while (true) {
            updateSwitches();
            
            // Check reload switch with improved detection
            if (reloadSwitch.read() == HIGH) {
                // Add a small delay and check again to ensure it's a stable reading
                delay(20);
                updateSwitches();
                if (reloadSwitch.read() == HIGH) {
                    // Serial.println("Reload switch detected in no-wood mode");
                    handleReloadSwitch();
                    break;
                }
            }
            
            if (startCycleSwitch.read() == LOW) {
                while (startCycleSwitch.read() == LOW) {
                    updateSwitches();
                    
                    // Check reload switch with improved detection here too
                    if (reloadSwitch.read() == HIGH) {
                        // Add a small delay and check again to ensure it's a stable reading
                        delay(20);
                        updateSwitches();
                        if (reloadSwitch.read() == HIGH) {
                            // Serial.println("Reload switch detected while waiting for start cycle");
                            handleReloadSwitch();
                            return;
                        }
                    }
                    
                    delay(50);
                }
                break;
            }
            delay(50);
        }
    }
}

void handleReloadSwitch() {
    static bool wasPressed = false;
    
    if (reloadSwitch.read() == HIGH && !wasPressed) {
        wasPressed = true;
        inReloadMode = true;  // Enter reload mode
        noWoodMode = false;   // Clear no-wood mode when entering reload mode
        // Serial.println("Reload switch activated - Retracting clamps");
        
        // Ensure both clamps are disengaged
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
        
        // Add a small delay to ensure the clamps have time to respond
        delay(10);
        
        updateLEDs();  // Update LEDs to show reload mode
    }
    else if (reloadSwitch.read() == LOW && wasPressed) {
        wasPressed = false;
        inReloadMode = false;  // Exit reload mode
        // Serial.println("Reload switch released - Re-engaging clamps");
        
        // Ensure both clamps are engaged
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
        
        delay(CLAMP_OPERATION_DELAY);  // Give clamps time to engage
        updateLEDs();  // Update LEDs to exit reload mode
    }
}

void updateSystemState() {
    switch(currentState) {
        case STARTUP:
            if (initialStateReported) currentState = HOMING;
            break;
        case HOMING:
            if (startupComplete) currentState = READY;
            break;
        case READY:
            if (startCycleSwitch.read() == HIGH) {
                currentState = CUTTING;
                // Serial.println("State changed to CUTTING");
            }
            break;
        case CUTTING:
            // The state will be reset to READY at the end of performCutCycle()
            break;
        case ERROR:
            // Add error handling if needed
            break;
    }
    
    // Update LED states after state changes
    updateLEDs();
}

void handleError(const String& errorMessage) {
    currentState = ERROR;
    lastError = errorMessage;
    // Serial.println("ERROR: " + errorMessage);
    
    // Stop all motors.
    cutMotor.stop();
    positionMotor.stop();
    
    // Put clamps in a safe state (engaged = LOW).
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
    
    // Update LEDs to show error state
    updateLEDs();
}

bool verifyPosition() {
    // For the cut motor, verify that if its position is zero, the switch reads HIGH.
    if (cutMotor.currentPosition() == 0 && cutSwitchDebouncer.read() != HIGH) {
        handleError("Cut motor position mismatch");
        return false;
    }
    // For the position motor, verify that if its position is zero, the switch reads HIGH.
    if (positionMotor.currentPosition() == 0 && positionSwitchDebouncer.read() != HIGH) {
        handleError("Position motor location mismatch");
        return false;
    }
    return true;
}

void reportSystemStatus() {
    static unsigned long lastReport = 0;
    if (millis() - lastReport > 5000) {
        // Serial.println("\nSystem Status:");
        // Serial.println("----------------");
        // Serial.print("State: ");
        switch(currentState) {
            case STARTUP: /* Serial.println("STARTUP"); */ break;
            case HOMING: /* Serial.println("HOMING"); */ break;
            case READY: /* Serial.println("READY"); */ break;
            case CUTTING: /* Serial.println("CUTTING"); */ break;
            case ERROR: /* Serial.println("ERROR"); */ break;
        }
        // Serial.print("Cut Motor: ");
        // Serial.print(cutMotor.currentPosition());
        // Serial.print(" / ");
        // Serial.println(cutMotor.targetPosition());
        // Serial.print("Position Motor: ");
        // Serial.print(positionMotor.currentPosition());
        // Serial.print(" / ");
        // Serial.println(positionMotor.targetPosition());
        // Serial.print("Clamps: Pos=");
        // Serial.print(digitalRead(PIN_POSITION_CLAMP));
        // Serial.print("  Wood=");
        // Serial.println(digitalRead(PIN_SECURE_WOOD_CLAMP));
        // Serial.println("----------------\n");
        
        lastReport = millis();
    }
}

bool validateConfiguration() {
    bool isValid = true;
    
    if (CUT_MOTOR_TRAVEL <= 0 || CUT_MOTOR_TRAVEL > 12) {
        // Serial.println("Invalid cut motor travel distance");
        isValid = false;
    }
    
    if (POSITION_MOTOR_TRAVEL <= 0 || POSITION_MOTOR_TRAVEL > 5) {
        // Serial.println("Invalid position motor travel distance");
        isValid = false;
    }
    
    if (!isValid) {
        handleError("Configuration validation failed");
        while (1) { delay(1000); }
    }
    
    return isValid;
}

bool moveToPosition(AccelStepper& motor, long position, float speed, float accel) {
    motor.setMaxSpeed(speed);
    motor.setAcceleration(accel);
    motor.moveTo(position);
    
    unsigned long timeout = millis() + 10000;  // 10-second timeout.
    
    while(motor.distanceToGo() != 0) {
        motor.run();
        if (millis() > timeout) {
            handleError("Motor movement timeout");
            return false;
        }
    }
    return true;
}

void updateLEDs() {
    // Skip LED updates during homing - we handle this separately in runHomingSequence()
    if (currentState == HOMING) {
        return;
    }
    
    // Turn all LEDs off first
    digitalWrite(PIN_RED_LED, LOW);
    digitalWrite(PIN_YELLOW_LED, LOW);
    digitalWrite(PIN_GREEN_LED, LOW);
    digitalWrite(PIN_BLUE_LED, LOW);
    
    // First check for reload mode as it overrides normal states
    if (inReloadMode) {
        digitalWrite(PIN_YELLOW_LED, HIGH);  // Yellow LED for reload mode
        return;  // Exit early as reload mode overrides other states
    }
    
    // Check for no-wood mode
    if (noWoodMode) {
        digitalWrite(PIN_BLUE_LED, HIGH);  // Blue LED for no-wood mode
    }
    
    // Set appropriate LED based on system state
    switch(currentState) {
        case STARTUP:
            digitalWrite(PIN_BLUE_LED, HIGH);    // Blue for setup
            break;
            
        case READY:
        case CUTTING:  // Include CUTTING state here
            digitalWrite(PIN_GREEN_LED, HIGH);   // Green stays on during both ready and cutting states
            if (currentState == CUTTING) {       // Yellow only during cutting
                digitalWrite(PIN_YELLOW_LED, HIGH);
            }
            break;
            
        case ERROR:
            digitalWrite(PIN_RED_LED, HIGH);     // Red for error
            break;
    }
}

// Modify the sendSignalToStage2 function to use a non-blocking pulse with a much longer duration
void sendSignalToStage2() {
    // Send a non-blocking pulse to Stage 2 via GPIO
    static unsigned long signalStartTime = 0;
    const unsigned long pulseDuration = 2000;  // Increased to 2000ms (2 seconds)
    
    // If the signal pin is HIGH, trigger a pulse by setting it LOW and record the current time.
    if (digitalRead(SIGNAL_1TO2_PIN) == HIGH) {
        digitalWrite(SIGNAL_1TO2_PIN, LOW);
        signalStartTime = millis();
        // Serial.println("SIGNAL: Sending LOW pulse to Stage 1 to Stage 2 controller (2 second pulse)");
    }
    // Once the pulse duration has elapsed, set the signal pin HIGH again.
    else if (millis() - signalStartTime >= pulseDuration) {
        digitalWrite(SIGNAL_1TO2_PIN, HIGH);
        // Serial.println("SIGNAL: Completed pulse to Stage 1 to Stage 2 controller");
    }
}

void reportError(String errorMessage) {
    currentState = ERROR;
    lastError = errorMessage;
    // Serial.println("ERROR: " + errorMessage);
    
    // Stop all motors.
    cutMotor.stop();
    positionMotor.stop();
    
    // Put clamps in a safe state (engaged = LOW).
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
    
    // Update LEDs to show error state
    updateLEDs();
}
