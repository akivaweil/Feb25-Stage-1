#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>
// Add WiFi and OTA libraries
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_now.h>  // Added ESP-NOW header

// WiFi credentials
const char* ssid = "Everwood";
const char* password = "Everwood-Staff";

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
#define WOOD_SENSOR 35
#define WAS_WOOD_SUCTIONED_SENSOR 5

// Clamp Pin Definitions
#define POSITION_CLAMP 13
#define WOOD_SECURE_CLAMP 15

// LED Pin Definitionss
#define RED_LED 26   // Error LED
#define YELLOW_LED 21 // Busy/Reload LED
#define GREEN_LED 4   // Ready LED
#define BLUE_LED 2    // Setup/No-Wood LED

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
const float CUT_TRAVEL_DISTANCE = 7; // inches
const float POSITION_TRAVEL_DISTANCE = 3.45; // inches
const int CUT_HOMING_DIRECTION = -1;
const int POSITION_HOMING_DIRECTION = -1;

// Speed and Acceleration Settings
const float CUT_NORMAL_SPEED = 105;
const float CUT_RETURN_SPEED = 1500;
const float CUT_ACCELERATION = 3200;
const float CUT_HOMING_SPEED = 300;
const float POSITION_NORMAL_SPEED = 50000;
const float POSITION_RETURN_SPEED = 50000;
const float POSITION_ACCELERATION = 50000;
const float POSITION_HOMING_SPEED = 2000; // Slower speed for homing operations
const float POSITION_RETURN_ACCELERATION = 50000; // You can adjust this value as needed

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

// --- New ESP-NOW Global Definitions and Functions Start ---

/*
 * NOTE: Although the peer is labeled as 'stage2PeerAddress' and the function is named
 * 'sendSignalToStage2', this signal is actually intended for the machine that moves the
 * wood squares from Stage 1 to Stage 2 (often referred to as the "Stage 1 to Stage 2" machine).
 * This naming convention is maintained for historical reasons despite the potential confusion.
 */
// Replace with the actual MAC address of Stage 2 ESP32
// Use the TEST_MAC_ADDRESS program to find the correct address
uint8_t stage2PeerAddress[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC}; 

typedef struct struct_message {
  char command[32];
} struct_message;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery success" : "Delivery fail");
}

void initESPNOW() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, stage2PeerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void sendSignalToStage2(const char* command) {
  struct_message msg;
  memset(&msg, 0, sizeof(msg));
  strncpy(msg.command, command, sizeof(msg.command) - 1);
  esp_err_t result = esp_now_send(stage2PeerAddress, (uint8_t *)&msg, sizeof(msg));
  if (result == ESP_OK) {
    Serial.print("ESP-NOW message sent: ");
    Serial.println(command);
  } else {
    Serial.println("Error sending ESP-NOW message");
  }
}
// --- New ESP-NOW Global Definitions and Functions End ---

// Add these function declarations before the setup() function
void performHomingSequence();
void performNoWoodOperation();
void performCuttingOperation();
void performReturnOperation();
void performPositioningOperation();
void handleErrorState();
void resetFromError();

void setupOTA() {
  // Connect to WiFi with static IP
  WiFi.mode(WIFI_STA);
  
  // Set static IP configuration
  IPAddress staticIP(192, 168, 1, 223);  // Your desired static IP
  IPAddress gateway(192, 168, 1, 1);     // Your router's IP (typically)
  IPAddress subnet(255, 255, 255, 0);    // Subnet mask
  IPAddress dns(8, 8, 8, 8);             // DNS (Google's public DNS)
  
  // Configure static IP
  if (!WiFi.config(staticIP, gateway, subnet, dns)) {
    // Failed to configure static IP
    for (int i = 0; i < 5; i++) {
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
      delay(100);
    }
  }
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    // Flash blue LED to indicate WiFi connection attempt
    digitalWrite(BLUE_LED, HIGH);
    delay(500);
    digitalWrite(BLUE_LED, LOW);
    delay(500);
    // Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Set up OTA
  ArduinoOTA.setHostname("ESP32-Stage1");
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
    // Serial.println("Start updating " + type);
    
    // Turn off all motors and disable outputs for safety during update
    cutMotor.stop();
    positionMotor.stop();
    digitalWrite(POSITION_CLAMP, HIGH);  // Disengage clamps for safety
    digitalWrite(WOOD_SECURE_CLAMP, HIGH);
    
    // Turn on just the blue LED steadily to indicate OTA in progress
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
  });
  
  ArduinoOTA.onEnd([]() {
    // Serial.println("\nEnd");
    // Single flash of all LEDs to indicate update complete
    digitalWrite(RED_LED, HIGH);
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BLUE_LED, HIGH);
    delay(500);
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    // Only update LED at 50% milestone to minimize flashing
    int percent = progress / (total / 100);
    static int lastMilestone = 0;
    
    // Only light up at 50% to keep it very minimal
    if (percent >= 50 && lastMilestone < 50) {
      lastMilestone = 50;
      digitalWrite(BLUE_LED, LOW);      // Turn off blue
      digitalWrite(YELLOW_LED, HIGH);   // Turn on just yellow at 50%
    }
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    // Serial.printf("Error[%u]: ", error);
    // if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    // else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    // else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    // else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    // else if (error == OTA_END_ERROR) Serial.println("End Failed");
    
    // Just three slow flashes of red LED to indicate error
    for (int i = 0; i < 3; i++) {
      digitalWrite(RED_LED, HIGH);
      delay(300);
      digitalWrite(RED_LED, LOW);
      delay(300);
    }
  });
  
  ArduinoOTA.begin();
  // Serial.println("OTA Ready");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32-stage1")) {
    // Serial.println("mDNS responder started");
    // Add service to mDNS
    MDNS.addService("arduino", "tcp", 3232);
  }
}

void setup() {
  // Serial.begin(115200);
  // Serial.println("Automated Table Saw Control System - Stage 1");
  
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
  
  positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_ACCELERATION);
  
  // Initially set motors to use position 0
  cutMotor.setCurrentPosition(0);
  positionMotor.setCurrentPosition(0);
  
  // Set up OTA
  setupOTA();
  initESPNOW();
  
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
  // Handle OTA updates
  ArduinoOTA.handle();
  
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
  static unsigned long lastDebugTime = 0;
  static bool signalActive = false;
  static unsigned long errorBlinkStartTime = 0;
  static bool homePositionErrorDetected = false;
  
  // Debug output - add wood sensor state to this debug output
  if (millis() - lastDebugTime > 1000) {
    // Serial.print("Cutting stage: ");
    // Serial.print(cuttingStage);
    // Serial.print(" Position: ");
    // Serial.print(cutMotor.currentPosition());
    // Serial.print(" Target: ");
    // Serial.print(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
    // Serial.print(" Wood present: ");
    // Serial.println(woodPresent ? "YES" : "NO");
    lastDebugTime = millis();
  }
  
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
      // Serial.println("Home position error acknowledged, resetting system");
    }
    
    // Do not return from this function - system is in error state
    return;
  }
  
  // Handle signal timing independently of motor movements
  if (signalActive && millis() - signalStartTime >= 2000) {
    signalActive = false;
    // Serial.println("Signal to Stage 1TO2 completed");
  }
  
  switch (cuttingStage) {
    case 0: // Start cut motion without checking wood suction sensor
      // Serial.println("Starting cut motion to position: " + 
      //               String(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH));
      // Move cut motor forward to perform cut
      cutMotor.moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
      cuttingStage = 1;
      break;
      
    case 1: // Check for wood suction when saw is 1 inch into the cut
      // Check if we've moved 1 inch into the cut
      if (cutMotor.currentPosition() >= (1.0 * CUT_MOTOR_STEPS_PER_INCH)) {
        // Check if wood was suctioned (active LOW per explanation)
        if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == LOW) {
          // Serial.println("ERROR: Wood detected in suction sensor during cut, performing hardware reset");
          // Flash red LED before reset
          digitalWrite(BLUE_LED, LOW);  // Turn off blue LED
          digitalWrite(GREEN_LED, LOW); // Turn off green LED
          digitalWrite(YELLOW_LED, LOW); // Turn off yellow LED
          
          // Flash red LED rapidly 5 times before reset with 50% longer duration
          for (int i = 0; i < 5; i++) {
            digitalWrite(RED_LED, HIGH);
            delay(150);  // Increased from 100ms to 150ms (50% longer)
            digitalWrite(RED_LED, LOW);
            delay(150);  // Increased from 100ms to 150ms (50% longer)
          }
          
          ESP.restart(); // Hardware reset of ESP32 as per instructions
        }
        
        // If we passed the check, move to the next stage
        cuttingStage = 2;
      }
      break;
      
    case 2: // Wait for cut to complete
      if (cutMotor.distanceToGo() == 0) {
        // Send ESP-NOW message to Stage 2 to signal start of cycle (non-blocking)
        sendSignalToStage2("startCycle");

        // Configure motors for return speeds
        cutMotor.setMaxSpeed(CUT_RETURN_SPEED);
        positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);

        // Re-read the wood sensor immediately after cut move finishes
        int sensorValue = digitalRead(WOOD_SENSOR);
        bool noWoodDetected = (sensorValue == HIGH);  // HIGH indicates no wood present
        // Serial.print("Checking wood sensor after cut: ");
        // Serial.println(noWoodDetected ? "No wood detected" : "Wood detected");
        
        if (noWoodDetected) {
          // Serial.println("No wood detected: entering no-wood mode");
          performNoWoodOperation();
          cuttingStage = 6;
        } else {
          // Normal operation path (wood is detected)
          // Start cut motor return
          cutMotor.moveTo(0);
          // Begin position motor return while keeping position clamp engaged
          digitalWrite(POSITION_CLAMP, LOW); // Ensure position clamp is engaged during initial movement
          
          // Move 0.1 inches back from current position (which should be at POSITION_TRAVEL_DISTANCE)
          long currentPos = positionMotor.currentPosition();
          positionMotor.moveTo(currentPos - (0.1 * POSITION_MOTOR_STEPS_PER_INCH));
          
          cuttingStage = 3;
        }
      }
      break;
      
    case 3: // Wait for initial position move
      if (positionMotor.distanceToGo() == 0) {
        // After 0.1 inch, disengage position clamp and wood secure clamp
        digitalWrite(POSITION_CLAMP, HIGH);  // Retract position clamp after 0.1 inch movement
        digitalWrite(WOOD_SECURE_CLAMP, HIGH); // Disengage wood secure clamp
        
        // Set the new acceleration profile for returning home
        positionMotor.setAcceleration(POSITION_RETURN_ACCELERATION);
        
        // Continue position motor movement to home
        positionMotor.moveTo(0);
        cuttingStage = 4;
      }
      break;
      
    case 4: { // Wait for position motor to reach home position
      // Check if position motor has reached home
      if (positionMotor.distanceToGo() == 0) {
        // Position motor is now at home position
        // Engage position clamp as soon as it reaches home
        digitalWrite(POSITION_CLAMP, LOW); // Engage position clamp when at home position
        
        // Now wait for cut motor to return home with timeout check
        static unsigned long stage4StartTime = 0;
        static bool stage4TimerStarted = false;
        
        // Initialize the timer when position motor is at home
        if (!stage4TimerStarted) {
          stage4StartTime = millis();
          stage4TimerStarted = true;
        }
        
        // Check if the timeout has been exceeded
        bool timedOut = (millis() - stage4StartTime > CUT_HOME_TIMEOUT);

        if (timedOut) {
          // Timeout exceeded, check home switch regardless of motor step completion
          cutPositionSwitch.update();
          if (cutPositionSwitch.read() != HIGH) {
            // Error: home switch not activated even after timeout
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
            // Extra explicit sensor check before proceeding:
            // Verify the homing sensor is stably pressed over several readings.
            bool sensorStable = true;
            for (int i = 0; i < 3; i++) {
              delay(10); // brief delay for sensor stabilization
              cutPositionSwitch.update();
              if (cutPositionSwitch.read() != HIGH) {
                sensorStable = false;
                break;
              }
            }
            if (!sensorStable) {
              // Error: sensor did not consistently read as pressed
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
              // Sensor is stably pressed. Proceed.
              stage4TimerStarted = false;
              // Position clamp is already engaged, now move to final position
              positionMotor.moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
              cuttingStage = 5;
            }
          }
        } else if (cutMotor.distanceToGo() == 0) {
          // Extra explicit sensor check before proceeding:
          // Verify the homing sensor is stably pressed over multiple checks.
          bool sensorStable = true;
          for (int i = 0; i < 3; i++) {
            delay(10); // brief delay for sensor stabilization
            cutPositionSwitch.update();
            if (cutPositionSwitch.read() != HIGH) {
              sensorStable = false;
              break;
            }
          }
          if (!sensorStable) {
            // Error: sensor did not consistently read as pressed
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
            // Sensor is stably pressed. Proceed.
            stage4TimerStarted = false;
            // Position clamp is already engaged, now move to final position
            positionMotor.moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
            cuttingStage = 5;
          }
        }
      }
      break;
    }
      
    case 5: // Wait for position motor to reach final position (normal mode)
      if (positionMotor.distanceToGo() == 0) {
        // Extend secure wood clamp
        digitalWrite(WOOD_SECURE_CLAMP, LOW);
        
        // Turn off busy LED
        digitalWrite(YELLOW_LED, LOW);
        
        // Reset the cutting cycle in progress flag
        cuttingCycleInProgress = false;
        
        currentState = READY;
        cuttingStage = 0; // Reset for next cycle
      }
      break;
      
    case 6: // Wait for no-wood mode sequence to complete
      static int noWoodStage = 0;
      static unsigned long cylinderActionTime = 0;
      static bool waitingForCylinder = false;
      static bool positionMotorStarted = false;
      
      // Handle cylinder timing without delays
      if (waitingForCylinder && (millis() - cylinderActionTime >= 150)) {
        waitingForCylinder = false;
        // Proceed to the next stage that was waiting for cylinder action
        noWoodStage++;
      }
      
      if (!waitingForCylinder) {
        switch (noWoodStage) {
          case 0: // Start position motor to home immediately while cut motor is returning
            if (!positionMotorStarted) {
              // Retract wood secure clamp during no-wood sequence
              digitalWrite(WOOD_SECURE_CLAMP, HIGH);
              // Start position motor moving to home immediately
              positionMotor.moveTo(0);
              positionMotorStarted = true;
              // Serial.println("No-wood sequence: Starting position motor to home while cut motor returns");
            }
            
            // Check if cut motor is home
            if (cutMotor.distanceToGo() == 0) {
              // Cut motor is home, now extend position cylinder (engage clamp)
              digitalWrite(POSITION_CLAMP, LOW);
              // Serial.println("No-wood sequence: Cut motor home, extending position cylinder (1st time)");
              cylinderActionTime = millis();
              waitingForCylinder = true;
              positionMotorStarted = false; // Reset for next cycle
              // noWoodStage will be incremented after waiting
            }
            break;
            
          case 1: // Wait for position motor to reach home (if it hasn't already)
            if (positionMotor.distanceToGo() == 0) {
              // Retract position cylinder
              digitalWrite(POSITION_CLAMP, HIGH);
              // Serial.println("No-wood sequence: Position motor at home, retracting position cylinder");
              cylinderActionTime = millis();
              waitingForCylinder = true;
              // noWoodStage will be incremented after waiting
            }
            break;
            
          case 2: // Move position motor to 2.0 inches
            positionMotor.moveTo(2.0 * POSITION_MOTOR_STEPS_PER_INCH);
            // Serial.println("No-wood sequence: Moving position motor to 2.0 inches");
            noWoodStage = 3;
            break;
            
          case 3: // Wait for position motor to reach 2.0 inches
            if (positionMotor.distanceToGo() == 0) {
              // Extend position cylinder again (2nd time)
              digitalWrite(POSITION_CLAMP, LOW);
              // Serial.println("No-wood sequence: Extending position cylinder (2nd time)");
              cylinderActionTime = millis();
              waitingForCylinder = true;
              // noWoodStage will be incremented after waiting
            }
            break;
            
          case 4: // Move position motor to home again
            positionMotor.moveTo(0);
            // Serial.println("No-wood sequence: Moving position motor to home (2nd time)");
            noWoodStage = 5;
            break;
            
          case 5: // Wait for position motor to reach home
            if (positionMotor.distanceToGo() == 0) {
              // Retract position cylinder
              digitalWrite(POSITION_CLAMP, HIGH);
              // Serial.println("No-wood sequence: Retracting position cylinder (2nd time)");
              cylinderActionTime = millis();
              waitingForCylinder = true;
              // noWoodStage will be incremented after waiting
            }
            break;
            
          case 6: // Move position motor to 3.45 inches final time
            positionMotor.moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
            // Serial.println("No-wood sequence: Moving position motor to 3.45 inches (final)");
            noWoodStage = 7;
            break;
            
          case 7: // Wait for final position and complete sequence
            if (positionMotor.distanceToGo() == 0) {
              // Extra sensor check: ensure homing sensor is stably pressed before finalizing the sequence.
              bool sensorStable = true;
              for (int i = 0; i < 3; i++) {
                delay(10); // brief delay for sensor stabilization
                cutPositionSwitch.update();
                if (cutPositionSwitch.read() != HIGH) {
                  sensorStable = false;
                  break;
                }
              }
              if (!sensorStable) {
                // Error: sensor did not consistently read as pressed
                cutMotor.stop();
                positionMotor.stop();
                digitalWrite(POSITION_CLAMP, LOW);
                digitalWrite(WOOD_SECURE_CLAMP, LOW);
                digitalWrite(RED_LED, HIGH);
                digitalWrite(YELLOW_LED, LOW);
                currentState = ERROR;
                errorStartTime = millis();
                noWoodStage = 0;
                break;
              }
              // Sensor is stably pressed. Now, proceed with completion.
              // Keep wood secure clamp retracted at the end of the sequence
              digitalWrite(WOOD_SECURE_CLAMP, HIGH); // Keep clamp retracted (HIGH = retracted)
              digitalWrite(YELLOW_LED, LOW);
              digitalWrite(BLUE_LED, HIGH); // Keep blue LED on to indicate waiting for switch reset
              
              // Reset for next cycle
              noWoodStage = 0;
              waitingForCylinder = false;
              cuttingCycleInProgress = false;
              
              // Instead of going directly to READY state, go to a new state that requires switch reset
              currentState = READY;
              
              // Set a flag to indicate we need the start switch to be reset
              continuousModeActive = false;  // Force continuous mode off
              startSwitchSafe = false;       // Require start switch to be turned off before allowing new cycle
              
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
