#include "StateMachine/StateMachine.h"
#include <FastAccelStepper.h>
#include <Bounce2.h>

//* ************************************************************************
//* ************************ YESWOOD STATE ********************************
//* ************************************************************************
//! YESWOOD state implementation - Processing detected wood with positioning sequence
//! 
//! Step-by-step sequence:
//! 1. Return the cut motor to position 0 (this should occur simultaneously with everything else)
//! 2. Retract the secure wood clamp
//! 3. Move the position motor to POSITION_TRAVEL_DISTANCE - 0.1
//! 4. Extend the secure wood clamp and retract the position clamp
//! 5. Move the position clamp to position 0
//! 6. As soon as the position motor reaches 0, the position clamp should extend
//! 7. Wait until the cut motor is home. 10ms after the cut motor reports that is at position 0, check the cut motor homing sensor. If it reads low then enter the cutmotorfailedtohome error state
//! 8. Do another check that the cut motor is home by putting this inside an if statement: move the position motor to POSITION_TRAVEL_DISTANCE
//! 9. Check if the run cycle switch is HIGH if yes then switch to the cutting state, if not, then return to idle state

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
static const unsigned long CUT_MOTOR_HOME_CHECK_DELAY = 10; // 10ms delay

void executeYESWOOD() {
    //! ************************************************************************
    //! STEP 1: RETURN CUT MOTOR TO 0 AND RETRACT SECURE CLAMP (SIMULTANEOUS)
    //! ************************************************************************
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
            moveMotorTo(CUT_MOTOR, 0, CUT_MOTOR_RETURN_SPEED);
            cutMotorReturnStarted = true;
        }
        
        // Retract secure wood clamp simultaneously
        retractClamp(WOOD_SECURE_CLAMP_TYPE);
        secureClampRetracted = true;
    }
    
    //! ************************************************************************
    //! STEP 3: MOVE POSITION MOTOR TO POSITION_TRAVEL_DISTANCE - 0.1
    //! ************************************************************************
    if (secureClampRetracted && !positionMotorToTravelMinus) {
        float targetPosition = POSITION_TRAVEL_DISTANCE - 0.1f;
        Serial.print("!3. Moving position motor to ");
        Serial.print(targetPosition);
        Serial.println(" inches");
        
        if (positionMotor) {
            long targetSteps = (long)(targetPosition * STEPS_PER_INCH_POSITION);
            moveMotorTo(POSITION_MOTOR, targetSteps, POSITION_MOTOR_NORMAL_SPEED);
        }
        positionMotorToTravelMinus = true;
    }
    
    //! ************************************************************************
    //! STEP 4: EXTEND SECURE CLAMP AND RETRACT POSITION CLAMP
    //! ************************************************************************
    if (positionMotorToTravelMinus && !clampOperationsComplete) {
        if (positionMotor && !positionMotor->isRunning()) {
            Serial.println("!4. Extending secure wood clamp and retracting position clamp");
            extendClamp(WOOD_SECURE_CLAMP_TYPE);
            retractClamp(POSITION_CLAMP_TYPE);
            clampOperationsComplete = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 5: MOVE POSITION MOTOR TO POSITION 0
    //! ************************************************************************
    if (clampOperationsComplete && !positionMotorToZero) {
        Serial.println("!5. Moving position motor to position 0");
        if (positionMotor) {
            moveMotorTo(POSITION_MOTOR, 0, POSITION_MOTOR_NORMAL_SPEED);
        }
        positionMotorToZero = true;
    }
    
    //! ************************************************************************
    //! STEP 6: EXTEND POSITION CLAMP AS SOON AS MOTOR REACHES 0
    //! ************************************************************************
    if (positionMotorToZero && !positionClampExtended) {
        if (positionMotor && !positionMotor->isRunning()) {
            Serial.println("!6. Position motor reached 0 - extending position clamp");
            extendClamp(POSITION_CLAMP_TYPE);
            positionClampExtended = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 7: WAIT FOR CUT MOTOR HOME AND VERIFY WITH SENSOR
    //! ************************************************************************
    if (positionClampExtended && !cutMotorHomeChecked) {
        if (cutMotor) {
            // Check if cut motor has reached position 0
            if (!cutMotorHomingWaitStarted && cutMotor->getCurrentPosition() == 0) {
                Serial.println("!7. Cut motor reports position 0 - starting 10ms verification timer");
                cutMotorZeroTime = millis();
                cutMotorHomingWaitStarted = true;
            }
            
            // After 10ms delay, check the homing sensor
            if (cutMotorHomingWaitStarted && (millis() - cutMotorZeroTime >= CUT_MOTOR_HOME_CHECK_DELAY)) {
                cutHomingSwitch.update();
                if (cutHomingSwitch.read() == LOW) {
                    Serial.println("ERROR: Cut motor homing sensor reads LOW - entering cutmotorfailedtohome error state");
                    triggerCutMotorHomeError();
                    // Reset state variables for next entry
                    yeswoodStateEntered = false;
                    return;
                } else {
                    Serial.println("Cut motor homing sensor verification passed");
                    cutMotorHomeChecked = true;
                }
            }
        }
    }
    
    //! ************************************************************************
    //! STEP 8: FINAL VERIFICATION AND MOVE TO POSITION_TRAVEL_DISTANCE
    //! ************************************************************************
    if (cutMotorHomeChecked && !finalPositionMotorMove) {
        if (cutMotor && cutMotor->getCurrentPosition() == 0) {
            Serial.println("!8. Final cut motor home verification passed - moving position motor to travel distance");
            if (positionMotor) {
                moveMotorTo(POSITION_MOTOR, POSITION_MOTOR_TRAVEL_POSITION, POSITION_MOTOR_NORMAL_SPEED);
            }
            finalPositionMotorMove = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 9: CHECK RUN CYCLE SWITCH AND TRANSITION
    //! ************************************************************************
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