#include "../../../include/StateMachine/StateMachine.h"
#include "Config/Config.h"
#include <FastAccelStepper.h>
#include <Bounce2.h>

// External variable declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;
extern Bounce cutHomingSwitch;
extern Bounce runCycleSwitch;
extern StateType currentState;

//* ************************************************************************
//* ************************ YESWOOD FUNCTIONS ***************************
//* ************************************************************************
//! YESWOOD-specific functions for handling successful wood cutting
//! These functions manage the complex sequence of returning motors,
//! advancing wood position, and preparing for the next cycle

//* ************************************************************************
//* ************************ MOTOR RETURN OPERATIONS FOR YESWOOD *********
//* ************************************************************************

void startCutMotorReturnForYeswood() {
    if (!cutMotor) {
        Serial.println("ERROR: Cut motor not initialized");
        return;
    }
    
    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_RETURN_SPEED);
    cutMotor->moveTo(0);
    Serial.println("YESWOOD: Cut motor returning to home position");
}

void retractSecureClampForYeswood() {
    digitalWrite(WOOD_SECURE_CLAMP, HIGH);
    Serial.println("YESWOOD: Secure wood clamp retracted");
}

void movePositionMotorToAdvancePositionForYeswood() {
    if (!positionMotor) {
        Serial.println("ERROR: Position motor not initialized");
        return;
    }
    
    // Move to POSITION_TRAVEL_DISTANCE - 0.1 inches
    float targetPosition = (POSITION_TRAVEL_DISTANCE - 0.1) * POSITION_MOTOR_STEPS_PER_INCH;
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_NORMAL_SPEED);
    positionMotor->moveTo(targetPosition);
    Serial.print("YESWOOD: Position motor moving to advance position: ");
    Serial.println(targetPosition);
}

//* ************************************************************************
//* ************************ CLAMP SWAP OPERATIONS FOR YESWOOD ***********
//* ************************************************************************

void swapClampPositionsForYeswood() {
    // Extend secure wood clamp
    digitalWrite(WOOD_SECURE_CLAMP, LOW);
    Serial.println("YESWOOD: Secure wood clamp extended for wood transfer");
    
    // Retract position clamp
    digitalWrite(POSITION_CLAMP, HIGH);
    Serial.println("YESWOOD: Position clamp retracted for wood transfer");
    
    Serial.println("YESWOOD: Clamp positions swapped for wood advancement");
}

void returnPositionMotorToHomeForYeswood() {
    if (!positionMotor) {
        Serial.println("ERROR: Position motor not initialized");
        return;
    }
    
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_RETURN_SPEED);
    positionMotor->moveTo(0);
    Serial.println("YESWOOD: Position motor returning to home position");
}

void extendPositionClampWhenHomeForYeswood() {
    if (positionMotor && !positionMotor->isRunning() && positionMotor->getCurrentPosition() == 0) {
        digitalWrite(POSITION_CLAMP, LOW);
        Serial.println("YESWOOD: Position clamp extended - position motor at home");
        return;
    }
}

//* ************************************************************************
//* ************************ CUT MOTOR HOME VERIFICATION FOR YESWOOD *****
//* ************************************************************************

bool verifyCutMotorHomeForYeswood() {
    if (!cutMotor) return false;
    
    // Check if cut motor is at home position
    if (!cutMotor->isRunning() && cutMotor->getCurrentPosition() == 0) {
        // Wait 10ms then check homing sensor
        delay(10);
        cutHomingSwitch.update();
        
        if (cutHomingSwitch.read() == LOW) {
            Serial.println("YESWOOD: ERROR - Cut motor failed to home properly (sensor LOW)");
            // TODO: Enter cutmotorfailedtohome error state
            return false;
        } else {
            Serial.println("YESWOOD: Cut motor home verification successful");
            return true;
        }
    }
    return false; // Motor still moving or not at home
}

void advancePositionMotorToTravelForYeswood() {
    if (!positionMotor) {
        Serial.println("ERROR: Position motor not initialized");
        return;
    }
    
    positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_NORMAL_SPEED);
    positionMotor->moveTo(POSITION_MOTOR_TRAVEL_POSITION);
    Serial.println("YESWOOD: Position motor advancing to travel position for next cycle");
}

//* ************************************************************************
//* ************************ CYCLE CONTINUATION CHECK FOR YESWOOD ********
//* ************************************************************************

bool checkRunCycleSwitchForYeswood() {
    runCycleSwitch.update();
    if (runCycleSwitch.read() == HIGH) {
        Serial.println("YESWOOD: Run cycle switch HIGH - continuing to CUTTING");
        currentState = CUTTING;
        return true;
    } else {
        Serial.println("YESWOOD: Run cycle switch not HIGH - returning to IDLE");
        currentState = IDLE;
        return true;
    }
}

//* ************************************************************************
//* ************************ MAIN YESWOOD EXECUTION FUNCTION *************
//* ************************************************************************

void executeYeswoodSequence() {
    static bool cutMotorReturnStarted = false;
    static bool secureClampRetracted = false;
    static bool positionMotorAdvanced = false;
    static bool clampsSwapped = false;
    static bool positionMotorHomeStarted = false;
    static bool positionClampExtended = false;
    static bool cutMotorHomeVerified = false;
    static bool finalAdvanceStarted = false;
    
    //! ************************************************************************
    //! STEP 1: START CUT MOTOR RETURN (ONE TIME)
    //! ************************************************************************
    if (!cutMotorReturnStarted) {
        startCutMotorReturnForYeswood();
        cutMotorReturnStarted = true;
    }
    
    //! ************************************************************************
    //! STEP 2: RETRACT SECURE CLAMP (ONE TIME)
    //! ************************************************************************
    if (!secureClampRetracted) {
        retractSecureClampForYeswood();
        secureClampRetracted = true;
    }
    
    //! ************************************************************************
    //! STEP 3: ADVANCE POSITION MOTOR (ONE TIME)
    //! ************************************************************************
    if (!positionMotorAdvanced) {
        movePositionMotorToAdvancePositionForYeswood();
        positionMotorAdvanced = true;
    }
    
    //! ************************************************************************
    //! STEP 4: WAIT FOR POSITION ADVANCE TO COMPLETE, THEN SWAP CLAMPS
    //! ************************************************************************
    if (positionMotorAdvanced && !clampsSwapped) {
        if (positionMotor && !positionMotor->isRunning()) {
            swapClampPositionsForYeswood();
            clampsSwapped = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 5: RETURN POSITION MOTOR TO HOME (ONE TIME)
    //! ************************************************************************
    if (clampsSwapped && !positionMotorHomeStarted) {
        returnPositionMotorToHomeForYeswood();
        positionMotorHomeStarted = true;
    }
    
    //! ************************************************************************
    //! STEP 6: EXTEND POSITION CLAMP WHEN HOME (CONTINUOUS CHECK)
    //! ************************************************************************
    if (positionMotorHomeStarted && !positionClampExtended) {
        extendPositionClampWhenHomeForYeswood();
        if (positionMotor && !positionMotor->isRunning() && positionMotor->getCurrentPosition() == 0) {
            positionClampExtended = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 7: VERIFY CUT MOTOR HOME (CONTINUOUS CHECK)
    //! ************************************************************************
    if (cutMotorReturnStarted && !cutMotorHomeVerified) {
        cutMotorHomeVerified = verifyCutMotorHomeForYeswood();
    }
    
    //! ************************************************************************
    //! STEP 8: FINAL ADVANCE AND STATE TRANSITION
    //! ************************************************************************
    if (cutMotorHomeVerified && positionClampExtended && !finalAdvanceStarted) {
        advancePositionMotorToTravelForYeswood();
        finalAdvanceStarted = true;
    }
    
    //! ************************************************************************
    //! STEP 9: CHECK FOR CYCLE CONTINUATION
    //! ************************************************************************
    if (finalAdvanceStarted && positionMotor && !positionMotor->isRunning()) {
        checkRunCycleSwitchForYeswood();
        
        // Reset state variables for next cycle
        cutMotorReturnStarted = false;
        secureClampRetracted = false;
        positionMotorAdvanced = false;
        clampsSwapped = false;
        positionMotorHomeStarted = false;
        positionClampExtended = false;
        cutMotorHomeVerified = false;
        finalAdvanceStarted = false;
    }
} 