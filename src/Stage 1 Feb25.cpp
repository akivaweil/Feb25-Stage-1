#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>

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

   AUTOMATED TABLE SAW CONTROL SYSTEM - STAGE 2
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
      - Steps per inch: 127 (GT2 belt)
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
// Homing and Operation Parameters
// ---------------------
const int HOME_SPEED = 2000;               // Base homing speed (steps/sec)
const int HOME_DIRECTION = -1;             // Cut motor homing direction (-1)
// Position motor homing direction is reversed (now -1)
const int POSITION_HOME_DIRECTION = -1;    
const int DEBOUNCE_INTERVAL = 20;          // Debounce time (ms)

const int STEPS_PER_INCH = 127;            // Cut motor steps per inch
const int POSITION_STEPS_PER_INCH = 1000;  // Position motor steps per inch
const float CUT_MOTOR_TRAVEL = 9;          // 9 inches cut travel
const float POSITION_MOTOR_TRAVEL = 3.35;  // 3.35 inches position travel
const unsigned long CLAMP_OPERATION_DELAY = 50;  // 50ms clamp delay

// Motor speeds and accelerations
const float CUT_MOTOR_SPEED = 150;
//SAVE: const float CUT_MOTOR_SPEED = 100;
const float CUT_MOTOR_RETURN_SPEED = 10000;
const float POSITION_MOTOR_SPEED = 5000;
const float POSITION_MOTOR_RETURN_SPEED = 20000;
const float CUT_MOTOR_ACCEL = 4000;
const float POSITION_MOTOR_ACCEL = 10000;

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
const unsigned long SECURE_CLAMP_DELAY = 100;  // 100ms delay

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

void setup() {
    // If pins 0 and 2 are not used and do not interfere with boot mode,
    // you can comment them out.
    // pinMode(BOOT_PIN, INPUT_PULLUP);    // Enable internal pullup on BOOT pin
    // pinMode(EN_PIN, OUTPUT);            // Configure Enable pin
    // digitalWrite(EN_PIN, HIGH);         // Enable the ESP32

    configureSerial();
    configureSwitches();
    configureMotorsForHoming();
    configureClamps();
    
    // Configure the wood sensor on pin 34 (using internal pullup; sensor is active LOW)
    pinMode(PIN_WOOD_SENSOR, INPUT_PULLUP);
    woodSensor.attach(PIN_WOOD_SENSOR, INPUT_PULLUP);
    woodSensor.interval(DEBOUNCE_INTERVAL);
    Serial.println("Wood sensor configured on pin 34 (active LOW)");
    
    // Configure the 1to2 signal pin
    pinMode(SIGNAL_1TO2_PIN, OUTPUT);
    digitalWrite(SIGNAL_1TO2_PIN, HIGH);  // Inactive by default
    
    printSetupComplete();
    if (!validateConfiguration()) {
        while (1) { delay(1000); }  // Halt system if configuration fails
    }

    // Add delay before starting homing sequence
    Serial.println("Waiting 500ms before starting homing sequence...");
    delay(500);
}

void loop() {
    updateSwitches();
    
    // Report initial state and check for homing (once only)
    if (!initialStateReported) {
        reportInitialStateAndCheckHoming();
        initialStateReported = true;
    }
    
    // Run the homing sequence until complete
    if (!startupComplete) {
        runHomingSequence();
    }
    
    // Add debug messages for switch states
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {  // Print every second
        Serial.print("Start Cycle Switch: ");
        Serial.print(startCycleSwitch.read());
        Serial.print(" Reload Switch: ");
        Serial.println(reloadSwitch.read());
        lastDebugTime = millis();
    }
    
    if (currentState == READY) {
        // Always check the reload switch state so that we can catch both press and release events.
        handleReloadSwitch();
        
        // Then, if the run cycle switch is active, continuously run cycles.
        if (startCycleSwitch.read() == HIGH) {
            while (true) {
                updateSwitches();
                // Check immediately before starting a new cycle
                if (startCycleSwitch.read() == LOW) {
                    Serial.println("Run cycle switch released, stopping continuous cycles.");
                    break;
                }
                Serial.println("Starting cut cycle...");
                performCutCycle();
                updateSwitches();
                delay(100); // Short pause before checking switch states again
                if (reloadSwitch.read() == HIGH)
                    break;
            }
        }
    }
    
    // Update system state and report status periodically
    updateSystemState();
    reportSystemStatus();
}

// ---------------------
// Configuration Functions
// ---------------------
void configureSerial() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial initialization
    Serial.println("----------------------------------------");
    Serial.println("Serial communication initialized");
    Serial.println("AUTOMATED TABLE SAW CONTROL SYSTEM");
    Serial.println("----------------------------------------");
}

void configureSwitches() {
    cutSwitchDebouncer.attach(PIN_CUT_MOTOR_POSITION_SWITCH, INPUT);
    cutSwitchDebouncer.interval(DEBOUNCE_INTERVAL);
    Serial.println("Cut motor position switch configured");
    
    positionSwitchDebouncer.attach(PIN_POSITION_MOTOR_POSITION_SWITCH, INPUT);
    positionSwitchDebouncer.interval(DEBOUNCE_INTERVAL);
    Serial.println("Position motor position switch configured");
    
    reloadSwitch.attach(PIN_RELOAD_SWITCH, INPUT);
    reloadSwitch.interval(DEBOUNCE_INTERVAL);
    Serial.println("Reload switch configured");
    
    startCycleSwitch.attach(PIN_START_CYCLE_SWITCH, INPUT);
    startCycleSwitch.interval(DEBOUNCE_INTERVAL);
    Serial.println("Start cycle switch configured");
}

void configureMotorsForHoming() {
    // Configure cut motor for homing (much gentler acceleration)
    cutMotor.setMaxSpeed(HOME_SPEED / 8);
    cutMotor.setAcceleration(CUT_MOTOR_ACCEL / 8);
    // Set a target position instead of just speed
    cutMotor.moveTo(-100000);  // Move a long distance in homing direction
    Serial.println("Cut motor configured for homing");
    
    // Configure position motor for homing (with reversed direction)
    positionMotor.setMaxSpeed(HOME_SPEED / 2);
    positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
    positionMotor.moveTo(100000 * POSITION_HOME_DIRECTION);
    Serial.println("Position motor configured for homing");
}

void configureClamps() {
    pinMode(PIN_POSITION_CLAMP, OUTPUT);
    pinMode(PIN_SECURE_WOOD_CLAMP, OUTPUT);
    
    // Both clamps engaged (LOW) initially
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
    Serial.println("Clamps configured - Both clamps engaged (LOW) initially");
}

void printSetupComplete() {
    Serial.println("----------------------------------------");
    Serial.println("Setup complete - Beginning homing sequence");
    Serial.println("----------------------------------------");
}

// ---------------------
// Switch and State Update Functions
// ---------------------
void updateSwitches() {
    cutSwitchDebouncer.update();
    positionSwitchDebouncer.update();
    reloadSwitch.update();
    startCycleSwitch.update();
}

void reportInitialStateAndCheckHoming() {
    Serial.println("\nInitial State Report:");
    
    // For the cut motor, the switch is now active when HIGH.
    Serial.print("Cut Motor Position Switch State: ");
    Serial.println(cutSwitchDebouncer.read() == HIGH ? "ACTIVATED" : "NOT ACTIVATED");
    
    Serial.print("Position Motor Position Switch State: ");
    Serial.println(positionSwitchDebouncer.read() == HIGH ? "ACTIVATED" : "NOT ACTIVATED");
    
    Serial.print("Reload Switch State: ");
    Serial.println(reloadSwitch.read() == HIGH ? "ACTIVATED" : "NOT ACTIVATED");
    Serial.println("----------------------------------------");
    
    if (cutSwitchDebouncer.read() == HIGH) {
        cutMotorHomed = true;
        cutMotor.setSpeed(0);
        cutMotor.setCurrentPosition(0);
        Serial.println("Cut motor already at home position");
    } else {
        cutMotor.setMaxSpeed(HOME_SPEED / 8);
        cutMotor.setSpeed((HOME_SPEED / 8) * HOME_DIRECTION);
    }
    
    if (positionSwitchDebouncer.read() == LOW) {
        positionMotorHomed = true;
        positionMotor.setSpeed(0);
        positionMotor.setCurrentPosition(0);  // Set to actual physical home
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        Serial.println("Position motor already at home position");
    } else {
        positionMotor.setMaxSpeed(HOME_SPEED);
        positionMotor.setSpeed(HOME_SPEED * POSITION_HOME_DIRECTION);
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
    }
    
    if (cutMotorHomed && positionMotorHomed) {
        startupComplete = true;
        currentState = READY;
        Serial.println("Both motors already homed - Ready for operation");
    }
}

void handleCutMotorHoming() {
    if (!cutMotorHomed) {
        if (cutSwitchDebouncer.read() == HIGH) {
            cutMotor.setSpeed(0);
            // Set position 0 to be 0.25 inches away from physical home
            cutMotor.setCurrentPosition(-0.25 * STEPS_PER_INCH);
            
            // Move away from switch by 0.25 inches with very gentle acceleration
            cutMotor.setMaxSpeed(HOME_SPEED / 8);
            cutMotor.setAcceleration(CUT_MOTOR_ACCEL / 32);
            cutMotor.moveTo(0);  // Move to position 0 (which is 0.25" from physical home)
            while (cutMotor.distanceToGo() != 0) {
                cutMotor.run();
            }
            
            cutMotorHomed = true;
            Serial.println("Cut motor homed and moved to safe position (0.25 inches from physical home)");
        } else {
            // Use run() instead of runSpeed() to enable acceleration control
            cutMotor.run();
            static unsigned long lastCutUpdate = 0;
            if (millis() - lastCutUpdate > 1000) {
                Serial.println("Cut motor homing in progress...");
                lastCutUpdate = millis();
            }
        }
    }
}

void handlePositionMotorHoming() {
    if (!positionMotorHomed) {
        // Disengage the clamp during homing for the position motor.
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        if (positionSwitchDebouncer.read() == LOW) {
            positionMotor.setSpeed(0);
            positionMotor.setCurrentPosition(0);  // Set to actual physical home
            
            // Move directly to the 3.35" position
            Serial.println("Moving to 3.35 inch position...");
            positionMotor.setMaxSpeed(POSITION_MOTOR_SPEED);
            positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
            positionMotor.moveTo(POSITION_STEPS_PER_INCH * POSITION_MOTOR_TRAVEL);
            while (positionMotor.distanceToGo() != 0) {
                positionMotor.run();
            }
            
            // Now engage the clamp and complete the homing process
            digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
            positionMotorHomed = true;
            startupComplete = true;
            Serial.println("Position motor homed and positioned at 3.35 inches");
            Serial.println("Homing sequence complete");
        } else {
            static unsigned long lastPositionUpdate = 0;
            if (millis() - lastPositionUpdate > 1000) {
                Serial.println("Position motor homing in progress...");
                lastPositionUpdate = millis();
            }
            positionMotor.run();
        }
    }
}

void runHomingSequence() {
    // Only run homing if the reload switch is not active.
    if (!startupComplete && reloadSwitch.read() == LOW) {
        if (!cutMotorHomed) {
            handleCutMotorHoming();
        } else if (!positionMotorHomed) {
            handlePositionMotorHoming();
        }
    } else if (!startupComplete && !cutMotorHomed && !positionMotorHomed) {
        static unsigned long lastSwitchMessage = 0;
        if (millis() - lastSwitchMessage > 2000) {
            Serial.println("\nCannot start homing:");
            if (reloadSwitch.read() == HIGH)
                Serial.println("- Reload switch is active");
            Serial.println("Please release switches to begin homing");
            lastSwitchMessage = millis();
        }
    }
}

// ---------------------
// Cutting Cycle Functions
// ---------------------
void performCutCycle() {
    Serial.println("Starting cut cycle sequence");
    
    // Configure the cut motor for normal operation.
    cutMotor.setMaxSpeed(CUT_MOTOR_SPEED);
    cutMotor.setAcceleration(CUT_MOTOR_ACCEL);
    
    // Ensure both clamps are engaged during cutting
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
    delay(CLAMP_OPERATION_DELAY);
    
    // Move the cut motor forward to execute the cut.
    cutMotor.setMaxSpeed(CUT_MOTOR_SPEED);
    cutMotor.setAcceleration(CUT_MOTOR_ACCEL);
    cutMotor.moveTo(STEPS_PER_INCH * CUT_MOTOR_TRAVEL);
    while (cutMotor.distanceToGo() != 0) {
        cutMotor.run();
    }
    
    // Allow sensor to settle and flush previous state before checking it
    delay(50);
    for (int i = 0; i < 5; i++) {
        woodSensor.update();
        delay(10);
    }
    bool woodPresent = (woodSensor.read() == LOW);
    Serial.print("Wood sensor reading after cut movement: ");
    Serial.println(woodPresent ? "LOW (wood present)" : "HIGH (no wood)");
    
    if(woodPresent) {
        Serial.println("Wood detected; proceeding with normal cycle with secure wood clamp action.");
        // Retract the secure wood clamp before pullback
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
        Serial.println("Secure wood clamp retracted for pullback.");
        long posBackTarget = positionMotor.currentPosition() - (POSITION_STEPS_PER_INCH * 0.1);
        positionMotor.setMaxSpeed(POSITION_MOTOR_SPEED);
        positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
        positionMotor.moveTo(posBackTarget);
        while (positionMotor.distanceToGo() != 0) {
            positionMotor.run();
        }
        // Extend the secure wood clamp immediately after pullback
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
        Serial.println("Secure wood clamp extended after pullback.");
        
        // NORMAL MODE: Execute the regular return sequence
        cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
        positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED * 2);
        positionMotor.setAcceleration(POSITION_MOTOR_ACCEL * 2);
        
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        secureClampTimer = millis();
        secureClampRetractPending = true;
        delay(CLAMP_OPERATION_DELAY);
        
        Serial.println("Signaling 1to2: Setting LOW (active)");
        digitalWrite(SIGNAL_1TO2_PIN, LOW);
        unsigned long signalStartTime = millis();
        
        if (secureClampRetractPending && (millis() - secureClampTimer >= SECURE_CLAMP_DELAY)) {
            digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
            secureClampRetractPending = false;
        }
        
        cutMotor.moveTo(0);
        positionMotor.moveTo(0);
        while (cutMotor.distanceToGo() != 0 || positionMotor.distanceToGo() != 0) {
            if (millis() - signalStartTime >= 1000) {
                digitalWrite(SIGNAL_1TO2_PIN, HIGH);
            }
            if (cutMotor.distanceToGo() != 0)
                cutMotor.run();
            if (positionMotor.distanceToGo() != 0)
                positionMotor.run();
        }
        digitalWrite(SIGNAL_1TO2_PIN, HIGH);
        
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        delay(CLAMP_OPERATION_DELAY + 300);
    
        // Retract secure wood clamp before moving forward
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
        Serial.println("Secure wood clamp retracted for forward movement");
        
        positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED * 2);
        positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
        positionMotor.moveTo(POSITION_STEPS_PER_INCH * (POSITION_MOTOR_TRAVEL + 0.1));
        while (positionMotor.distanceToGo() != 0) {
            positionMotor.run();
        }
        
        // Re-engage secure wood clamp after forward movement
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
        Serial.println("Secure wood clamp re-engaged after forward movement");
        
        delay(50);
        updateSwitches();
        if (startCycleSwitch.read() == LOW) {
            delay(200);
        }
        
        currentState = READY;
        Serial.println("Cut cycle complete - Ready for next cycle");
        // Flush sensor state to avoid carryover in the next cycle
        for (int i = 0; i < 5; i++) {
            woodSensor.update();
            delay(10);
        }
    } else {
        Serial.println("No wood detected; activating no-wood mode for this cycle.");
        // In no-wood mode, ensure the secure wood clamp remains retracted during pullback.
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);   // Extend the position clamp (engaged)
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);  // Keep secure wood clamp disengaged (retracted)
        long posBackTarget = positionMotor.currentPosition() - (POSITION_STEPS_PER_INCH * 0.1);
        positionMotor.setMaxSpeed(POSITION_MOTOR_SPEED);
        positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
        positionMotor.moveTo(posBackTarget);
        while (positionMotor.distanceToGo() != 0) {
            positionMotor.run();
        }
        
        // NO WOOD MODE: Return position motor to 0.5" from physical home
        cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
        positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
        
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
        delay(CLAMP_OPERATION_DELAY);
        
        Serial.println("Signaling 1to2: Setting LOW (active)");
        digitalWrite(SIGNAL_1TO2_PIN, LOW);
        unsigned long signalStartTime = millis();
        
        // Move position motor to 0.5" from physical home
        cutMotor.moveTo(0);  // Return cut motor to home position
        positionMotor.moveTo(POSITION_STEPS_PER_INCH * 0.5);  // Move position motor to 0.5" from physical home
        while (cutMotor.distanceToGo() != 0 || positionMotor.distanceToGo() != 0) {
            if (millis() - signalStartTime >= 1000) {
                digitalWrite(SIGNAL_1TO2_PIN, HIGH);
            }
            if (cutMotor.distanceToGo() != 0)
                cutMotor.run();
            if (positionMotor.distanceToGo() != 0)
                positionMotor.run();
        }
        digitalWrite(SIGNAL_1TO2_PIN, HIGH);
        
        Serial.println("Cut motor returned to home position");
        Serial.println("Position motor returned to 0.5 inches from physical home");
        Serial.println("Waiting for run cycle switch toggle or reload switch...");
        
        // Wait for either reload switch or start cycle switch
        while (true) {
            updateSwitches();
            
            // Check for reload switch
            if (reloadSwitch.read() == HIGH) {
                Serial.println("Reload switch activated - entering reload mode");
                handleReloadSwitch();
                break;
            }
            
            // Check for start cycle switch toggle sequence
            if (startCycleSwitch.read() == LOW) {
                while (startCycleSwitch.read() == LOW) {
                    updateSwitches();
                    // Also check reload switch while waiting
                    if (reloadSwitch.read() == HIGH) {
                        Serial.println("Reload switch activated - entering reload mode");
                        handleReloadSwitch();
                        return;
                    }
                    delay(50);
                }
                break;  // Start cycle switch has been toggled
            }
            
            delay(50);
        }
        
        // Flush sensor state to avoid carryover in the next cycle
        for (int i = 0; i < 5; i++) {
            woodSensor.update();
            delay(10);
        }
    }
}

void handleReloadSwitch() {
    static bool wasPressed = false;
    
    if (reloadSwitch.read() == HIGH && !wasPressed) {  // Just pressed
        wasPressed = true;
        Serial.println("Reload switch activated - Retracting clamps");
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
    }
    else if (reloadSwitch.read() == LOW && wasPressed) {  // Just released
        wasPressed = false;
        Serial.println("Reload switch released - Re-engaging clamps");
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
        delay(CLAMP_OPERATION_DELAY);  // Give clamps time to engage
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
                Serial.println("State changed to CUTTING");
            }
            break;
        case CUTTING:
            // The state will be reset to READY at the end of performCutCycle()
            break;
        case ERROR:
            // Add error handling if needed
            break;
    }
}

void handleError(const String& errorMessage) {
    currentState = ERROR;
    lastError = errorMessage;
    Serial.println("ERROR: " + errorMessage);
    
    // Stop all motors.
    cutMotor.stop();
    positionMotor.stop();
    
    // Put clamps in a safe state (engaged = LOW).
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
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
    if (millis() - lastReport > 5000) {  // Report every 5 seconds.
        Serial.println("\nSystem Status:");
        Serial.println("----------------");
        Serial.print("State: ");
        switch(currentState) {
            case STARTUP: Serial.println("STARTUP"); break;
            case HOMING: Serial.println("HOMING"); break;
            case READY: Serial.println("READY"); break;
            case CUTTING: Serial.println("CUTTING"); break;
            case ERROR: Serial.println("ERROR"); break;
        }
        Serial.print("Cut Motor: ");
        Serial.print(cutMotor.currentPosition());
        Serial.print(" / ");
        Serial.println(cutMotor.targetPosition());
        Serial.print("Position Motor: ");
        Serial.print(positionMotor.currentPosition());
        Serial.print(" / ");
        Serial.println(positionMotor.targetPosition());
        Serial.print("Clamps: Pos=");
        Serial.print(digitalRead(PIN_POSITION_CLAMP));
        Serial.print("  Wood=");
        Serial.println(digitalRead(PIN_SECURE_WOOD_CLAMP));
        Serial.println("----------------\n");
        
        lastReport = millis();
    }
}

bool validateConfiguration() {
    bool isValid = true;
    
    if (CUT_MOTOR_TRAVEL <= 0 || CUT_MOTOR_TRAVEL > 12) {
        Serial.println("Invalid cut motor travel distance");
        isValid = false;
    }
    
    if (POSITION_MOTOR_TRAVEL <= 0 || POSITION_MOTOR_TRAVEL > 5) {
        Serial.println("Invalid position motor travel distance");
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
