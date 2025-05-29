#include "StateMachine/StateMachine.h"
#include "StateMachine/SensorFunctions.h"
#include <FastAccelStepper.h>

//* ************************************************************************
//* ************************ CUTTING STATE ***************************
//* ************************************************************************
//! CUTTING state implementation
//! System performs wood cutting operations with error checking and transitions

// External variables and functions
extern bool woodSuctionError;
extern FastAccelStepper *cutMotor;
void forceTriggerWoodSuctionError();

// State variables
static bool cuttingStateEntered = false;
static bool clampsExtended = false;
static bool cutMotorStarted = false;
static bool woodSuctionChecked = false;
static float cutMotorCheckPosition = 0.3f; // 0.3 inches
static long cutMotorCheckSteps = 0;

void executeCUTTING() {
    //! Step 1: Entry actions - extend both clamps on first entry
    if (!cuttingStateEntered) {
        Serial.println("=== CUTTING STATE ENTERED ===");
        cuttingStateEntered = true;
        clampsExtended = false;
        cutMotorStarted = false;
        woodSuctionChecked = false;
        
        // Calculate the step position for 0.3 inch check
        cutMotorCheckSteps = (long)(cutMotorCheckPosition * CUT_MOTOR_STEPS_PER_INCH);
        
        Serial.println("!1. Extending both clamps");
        extendClamp(POSITION_CLAMP_TYPE);
        extendClamp(WOOD_SECURE_CLAMP_TYPE);
        clampsExtended = true;
        
        Serial.println("Clamps extended, waiting brief moment for engagement...");
    }
    
    //! Step 2: Start cut motor movement after clamps are extended
    if (clampsExtended && !cutMotorStarted) {
        Serial.println("!2. Starting cut motor movement to cut position");
        configureCutMotorForCutting();
        moveCutMotorToCut();
        cutMotorStarted = true;
        Serial.print("Cut motor moving to position: ");
        Serial.print(CUT_TRAVEL_DISTANCE);
        Serial.println(" inches");
    }
    
    //! Step 3: Monitor cut motor movement and check for wood suction at 0.3 inches
    if (cutMotorStarted && cutMotor && cutMotor->isRunning()) {
        long currentPosition = cutMotor->getCurrentPosition();
        
        // Check if motor has reached 0.3 inches and we haven't checked yet
        if (currentPosition >= cutMotorCheckSteps && !woodSuctionChecked) {
            Serial.println("!3. Cut motor reached 0.3 inches - checking wood suction sensor");
            
            // Check the wood suction sensor using proper named function
            bool woodSuctioned = readWoodSuctionSensor();
            
            if (!woodSuctioned) { // Sensor reads low (not suctioned)
                Serial.println("ERROR: Wood suction sensor reads LOW - triggering wood suction error!");
                forceTriggerWoodSuctionError();
                
                // Stop the cut motor immediately
                stopCutMotor();
                
                // Transition to ERROR state
                changeState(ERROR);
                
                // Reset state variables for next time
                cuttingStateEntered = false;
                return;
            } else {
                Serial.println("Wood suction sensor OK - continuing cut");
            }
            
            woodSuctionChecked = true;
        }
    }
    
    //! Step 4: Check for cut completion and decide next state based on wood sensor
    if (cutMotorStarted && cutMotor && !cutMotor->isRunning()) {
        Serial.println("!4. Cut motor movement complete - checking wood sensor for next state");
        
        // Read the wood sensor using proper named function
        bool woodDetected = readWoodSensor();
        
        if (!woodDetected) { // Sensor reads low = wood present
            Serial.println("Wood sensor reads LOW - wood detected, transitioning to YESWOOD");
            changeState(YESWOOD);
        } else { // Sensor reads high = no wood
            Serial.println("Wood sensor reads HIGH - no wood detected, transitioning to NOWOOD");
            changeState(NOWOOD);
        }
        
        // Reset state variables for next cutting cycle
        cuttingStateEntered = false;
        Serial.println("=== CUTTING STATE COMPLETE ===");
    }
} 