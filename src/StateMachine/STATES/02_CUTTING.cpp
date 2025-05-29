#include "StateMachine/StateMachine.h"
#include <FastAccelStepper.h>
#include <Bounce2.h>

//* ************************************************************************
//* ************************ CUTTING STATE ********************************
//* ************************************************************************
//! CUTTING state implementation - Execute cutting sequence with wood detection
//! 
//! Step-by-step sequence:
//! 1. Extend both clamps 
//! 2. Move the cut motor to CUT_TRAVEL_DISTANCE. When 0.3 inches in, check if waswoodsuctioned sensor reads low - if so, stop and enter waswoodsuctioned error state
//! 3. When cut motor reaches CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES and CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES distances away from end, activate accordingly
//! 4. Decide whether to move to yeswood or nowood states based on wood sensor pin

// External variable declarations
extern FastAccelStepper *cutMotor;
extern Bounce woodSensor;
extern Bounce wasWoodSuctionedSensor;

// State variables
static bool cuttingStateEntered = false;
static bool clampsExtended = false;
static bool cutMotorMoving = false;
static bool woodSuctionChecked = false;
static bool catcherClampActivated = false;
static bool catcherServoActivated = false;

void executeCUTTING() {
    //! ************************************************************************
    //! STEP 1: EXTEND BOTH CLAMPS
    //! ************************************************************************
    if (!cuttingStateEntered) {
        Serial.println("=== CUTTING STATE ENTERED ===");
        cuttingStateEntered = true;
        
        // Reset all state variables
        clampsExtended = false;
        cutMotorMoving = false;
        woodSuctionChecked = false;
        catcherClampActivated = false;
        catcherServoActivated = false;
        
        Serial.println("!1. Extending both clamps");
        extendClamp(POSITION_CLAMP_TYPE);
        extendClamp(WOOD_SECURE_CLAMP_TYPE);
        clampsExtended = true;
    }
    
    //! ************************************************************************
    //! STEP 2: MOVE CUT MOTOR TO CUT_TRAVEL_DISTANCE
    //! ************************************************************************
    if (clampsExtended && !cutMotorMoving) {
        Serial.println("!2. Moving cut motor to cut travel distance");
        if (cutMotor) {
            moveMotorTo(CUT_MOTOR, CUT_MOTOR_CUT_POSITION, CUT_MOTOR_NORMAL_SPEED);
            cutMotorMoving = true;
        }
    }
    
    //! ************************************************************************
    //! WOOD SUCTION CHECK AT 0.3 INCHES
    //! ************************************************************************
    if (cutMotorMoving && !woodSuctionChecked) {
        if (cutMotor) {
            float currentPosition = (float)cutMotor->getCurrentPosition() / CUT_MOTOR_STEPS_PER_INCH;
            if (currentPosition >= 0.3f) {
                wasWoodSuctionedSensor.update();
                if (wasWoodSuctionedSensor.read() == LOW) {
                    Serial.println("ERROR: Wood suction sensor reads LOW at 0.3 inches - entering waswoodsuctioned error state");
                    forceTriggerWoodSuctionError();
                    // Reset state variables for next entry
                    cuttingStateEntered = false;
                    return;
                }
                woodSuctionChecked = true;
            }
        }
    }
    
    //! ************************************************************************
    //! STEP 3: CATCHER CLAMP AND SERVO EARLY ACTIVATION
    //! ************************************************************************
    if (cutMotorMoving && cutMotor) {
        float currentPosition = (float)cutMotor->getCurrentPosition() / CUT_MOTOR_STEPS_PER_INCH;
        float distanceFromEnd = CUT_TRAVEL_DISTANCE - currentPosition;
        
        // Activate catcher clamp at early activation offset
        if (!catcherClampActivated && distanceFromEnd <= CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES) {
            Serial.println("!3a. Activating catcher clamp at early activation offset");
            extendClamp(CATCHER_CLAMP_TYPE);
            catcherClampActivated = true;
        }
        
        // Activate catcher servo at early activation offset
        if (!catcherServoActivated && distanceFromEnd <= CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES) {
            Serial.println("!3b. Activating catcher servo at early activation offset");
            activateCatcherServo();
            catcherServoActivated = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 4: DECIDE NEXT STATE BASED ON WOOD SENSOR
    //! ************************************************************************
    if (cutMotorMoving && cutMotor && !cutMotor->isRunning()) {
        woodSensor.update();
        bool woodSensorReading = woodSensor.read();
        Serial.print("!4. Wood sensor reading: ");
        Serial.print(woodSensorReading ? "HIGH" : "LOW");
        Serial.print(" (");
        Serial.print(woodSensorReading);
        Serial.println(")");
        
        if (woodSensorReading == LOW) {
            Serial.println("Wood sensor reads LOW (wood detected) - transitioning to YESWOOD state");
            changeState(YESWOOD);
        } else {
            Serial.println("Wood sensor reads HIGH (no wood detected) - transitioning to NOWOOD state");
            changeState(NOWOOD);
        }
        
        // Reset state variables for next entry
        cuttingStateEntered = false;
    }
} 