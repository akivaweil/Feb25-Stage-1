#include "StateMachine/StateMachine.h"
#include <FastAccelStepper.h>
#include <Bounce2.h>

//* ************************************************************************
//* ************************ YESWOOD STATE ***************************
//* ************************************************************************
//! YESWOOD state implementation
//! System processes detected wood piece with secure positioning and error checking

// External variable declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;
extern Bounce cutHomingSwitch;
extern Bounce startCycleSwitch;

// State variables
static bool yeswoodStateEntered = false;
static bool cutMotorReturnStarted = false;
static bool secureClampRetracted = false;
static bool positionMotorToTravelMinus = false;
static bool clampOperationsComplete = false;
static bool positionMotorToZero = false;
static bool positionClampExtended = false;
static bool cutMotorHomeChecked = false;
static bool cutMotorHomingWaitStarted = false;
static bool finalPositionMotorMove = false;
static unsigned long cutMotorZeroTime = 0;
static const unsigned long CUT_MOTOR_HOME_CHECK_DELAY = 50; // 50ms delay

void executeYESWOOD() {
    //! Step 1: Entry actions - start cut motor return and retract secure clamp simultaneously
    if (!yeswoodStateEntered) {
        Serial.println("=== YESWOOD STATE ENTERED ===");
        yeswoodStateEntered = true;
        
        // Reset all state variables
        cutMotorReturnStarted = false;
        secureClampRetracted = false;
        positionMotorToTravelMinus = false;
        clampOperationsComplete = false;
        positionMotorToZero = false;
        positionClampExtended = false;
        cutMotorHomeChecked = false;
        cutMotorHomingWaitStarted = false;
        finalPositionMotorMove = false;
        cutMotorZeroTime = 0;
        
        Serial.println("!1. Returning cut motor to position 0 and retracting secure wood clamp simultaneously");
        
        // Start cut motor return to position 0
        if (cutMotor) {
            configureCutMotorForReturn();
            moveCutMotorToHome();
            cutMotorReturnStarted = true;
        }
        
        // Retract secure wood clamp simultaneously
        retractClamp(WOOD_SECURE_CLAMP_TYPE);
        secureClampRetracted = true;
        
        Serial.println("Cut motor returning to home and secure clamp retracted");
    }
    
    //! Step 2: Move position motor to POSITION_TRAVEL_DISTANCE - 0.1
    if (secureClampRetracted && !positionMotorToTravelMinus) {
        float targetPosition = POSITION_TRAVEL_DISTANCE - 0.1f;
        Serial.print("!3. Moving position motor to ");
        Serial.print(targetPosition);
        Serial.println(" inches");
        
        if (positionMotor) {
            configurePositionMotorForNormalOperation();
            movePositionMotorToPosition(targetPosition);
        }
        positionMotorToTravelMinus = true;
    }
    
    //! Step 3: Wait for position motor to reach target, then extend secure clamp and retract position clamp
    if (positionMotorToTravelMinus && !clampOperationsComplete) {
        if (positionMotor && !positionMotor->isRunning()) {
            Serial.println("!4. Extending secure wood clamp and retracting position clamp");
            extendClamp(WOOD_SECURE_CLAMP_TYPE);
            retractClamp(POSITION_CLAMP_TYPE);
            clampOperationsComplete = true;
        }
    }
    
    //! Step 4: Move position motor to position 0
    if (clampOperationsComplete && !positionMotorToZero) {
        Serial.println("!5. Moving position motor to position 0");
        if (positionMotor) {
            movePositionMotorToHome();
        }
        positionMotorToZero = true;
    }
    
    //! Step 5: Extend position clamp as soon as position motor reaches 0
    if (positionMotorToZero && !positionClampExtended) {
        if (positionMotor && !positionMotor->isRunning()) {
            Serial.println("!6. Position motor reached 0 - extending position clamp");
            extendClamp(POSITION_CLAMP_TYPE);
            positionClampExtended = true;
        }
    }
    
    //! Step 6: Wait for cut motor to reach home and perform verification checks
    if (positionClampExtended && !cutMotorHomeChecked) {
        if (cutMotor) {
            // Check if cut motor has reached position 0
            if (!cutMotorHomingWaitStarted && cutMotor->getCurrentPosition() == 0) {
                Serial.println("!7. Cut motor reports position 0 - starting 50ms verification timer");
                cutMotorZeroTime = millis();
                cutMotorHomingWaitStarted = true;
            }
            
            // After 50ms delay, check the homing sensor
            if (cutMotorHomingWaitStarted && (millis() - cutMotorZeroTime >= CUT_MOTOR_HOME_CHECK_DELAY)) {
                cutHomingSwitch.update();
                if (cutHomingSwitch.read() == LOW) {
                    Serial.println("ERROR: Cut motor homing sensor reads LOW - entering cutmotorfailedtohome error state");
                    triggerCutMotorHomeError();
                    // Reset state variables for next entry
                    yeswoodStateEntered = false;
                    return;
                }
                Serial.println("Cut motor homing sensor verification passed");
                cutMotorHomeChecked = true;
            }
        }
    }
    
    //! Step 7: Final verification - move position motor to POSITION_TRAVEL_DISTANCE if cut motor is home
    if (cutMotorHomeChecked && !finalPositionMotorMove) {
        if (cutMotor && cutMotor->getCurrentPosition() == 0) {
            Serial.println("!8. Final cut motor home verification passed - moving position motor to travel distance");
            if (positionMotor) {
                movePositionMotorToTravel();
            }
            finalPositionMotorMove = true;
        }
    }
    
    //! Step 8: Check run cycle switch and transition to next state
    if (finalPositionMotorMove) {
        if (positionMotor && !positionMotor->isRunning()) {
            startCycleSwitch.update();
            if (startCycleSwitch.read() == HIGH) {
                Serial.println("!9. Run cycle switch is HIGH - transitioning to CUTTING state");
                changeState(CUTTING);
            } else {
                Serial.println("!9. Run cycle switch is LOW - returning to IDLE state");
                changeState(IDLE);
            }
            
            // Reset state variables for next entry
            yeswoodStateEntered = false;
        }
    }
} 