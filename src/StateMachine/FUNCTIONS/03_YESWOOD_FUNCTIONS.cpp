#include "StateMachine/StateMachine.h"
#include "Config/Config.h"
#include <AccelStepper.h>
#include <Bounce2.h>

// External variable declarations
extern AccelStepper cutMotor;
extern AccelStepper positionMotor;
extern Bounce cutHomingSwitch;
extern Bounce startCycleSwitch;
extern SystemState currentState;

//* ************************************************************************
//* ************************ YESWOOD FUNCTIONS ***************************
//* ************************************************************************
//! YESWOOD-specific functions for handling successful wood cutting
//! These functions manage the complex sequence of returning motors,
//! advancing wood position, and preparing for the next cycle

//* ************************************************************************
//* ************************ MOTOR RETURN OPERATIONS FOR YESWOOD *********
//* ************************************************************************

void returnCutMotorToHomeForYeswood() {
    cutMotor.setSpeed(CUT_MOTOR_RETURN_SPEED);
    cutMotor.moveTo(0);
    Serial.println("YESWOOD: Cut motor returning to home position");
}

void retractSecureClampForYeswood() {
    digitalWrite(WOOD_SECURE_CLAMP, HIGH);
    Serial.println("YESWOOD: Secure wood clamp retracted");
}

void advancePositionMotorForYeswood() {
    // Move to POSITION_TRAVEL_DISTANCE - 0.1 inches
    float targetPosition = (POSITION_TRAVEL_DISTANCE - 0.1) * POSITION_MOTOR_STEPS_PER_INCH;
    positionMotor.setSpeed(POSITION_MOTOR_NORMAL_SPEED);
    positionMotor.moveTo(targetPosition);
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
    positionMotor.setSpeed(POSITION_MOTOR_RETURN_SPEED);
    positionMotor.moveTo(0);
    Serial.println("YESWOOD: Position motor returning to home position");
}

void extendPositionClampWhenHomeForYeswood() {
    if (positionMotor.distanceToGo() == 0 && positionMotor.currentPosition() == 0) {
        digitalWrite(POSITION_CLAMP, LOW);
        Serial.println("YESWOOD: Position clamp extended - position motor at home");
    }
}

//* ************************************************************************
//* ************************ CUT MOTOR HOME VERIFICATION FOR YESWOOD *****
//* ************************************************************************

bool checkCutMotorHomeAndSensorForYeswood() {
    // Check if cut motor is at home position
    if (cutMotor.distanceToGo() == 0 && cutMotor.currentPosition() == 0) {
        // Wait 10ms then check homing sensor
        delay(10);
        cutHomingSwitch.update();
        if (cutHomingSwitch.read() == HIGH) {
            Serial.println("YESWOOD: Cut motor confirmed at home position");
            return true;
        } else {
            Serial.println("YESWOOD: WARNING - Cut motor reports home but sensor disagrees");
            return false;
        }
    }
    return false;
}

void advancePositionMotorToTravelForYeswood() {
    positionMotor.setSpeed(POSITION_MOTOR_NORMAL_SPEED);
    positionMotor.moveTo(POSITION_MOTOR_TRAVEL_POSITION);
    Serial.println("YESWOOD: Position motor advancing to travel position for next cycle");
}

//* ************************************************************************
//* ************************ CYCLE CONTINUATION CHECK FOR YESWOOD ********
//* ************************************************************************

bool checkRunCycleSwitchForYeswood() {
    startCycleSwitch.update();
    if (startCycleSwitch.read() == HIGH) {
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
        returnCutMotorToHomeForYeswood();
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
        advancePositionMotorForYeswood();
        positionMotorAdvanced = true;
    }
    
    //! ************************************************************************
    //! STEP 4: SWAP CLAMP POSITIONS WHEN POSITION MOTOR ADVANCE COMPLETE
    //! ************************************************************************
    if (positionMotorAdvanced && !clampsSwapped) {
        if (positionMotor.distanceToGo() == 0) {
            swapClampPositionsForYeswood();
            clampsSwapped = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 5: START POSITION MOTOR RETURN WHEN CLAMPS SWAPPED
    //! ************************************************************************
    if (clampsSwapped && !positionMotorHomeStarted) {
        returnPositionMotorToHomeForYeswood();
        positionMotorHomeStarted = true;
    }
    
    //! ************************************************************************
    //! STEP 6: EXTEND POSITION CLAMP WHEN POSITION MOTOR AT HOME
    //! ************************************************************************
    if (positionMotorHomeStarted && !positionClampExtended) {
        extendPositionClampWhenHomeForYeswood();
        if (positionMotor.distanceToGo() == 0 && positionMotor.currentPosition() == 0) {
            positionClampExtended = true;
        }
    }
    
    //! ************************************************************************
    //! STEP 6.5: VERIFY CUT MOTOR HOME (CONTINUOUS CHECK)
    //! ************************************************************************
    if (cutMotorReturnStarted && !cutMotorHomeVerified) {
        cutMotorHomeVerified = checkCutMotorHomeAndSensorForYeswood();
    }
    
    //! ************************************************************************
    //! STEP 7: START FINAL ADVANCE WHEN POSITION CLAMP EXTENDED AND CUT MOTOR HOME VERIFIED
    //! ************************************************************************
    if (positionClampExtended && cutMotorHomeVerified && !finalAdvanceStarted) {
        advancePositionMotorToTravelForYeswood();
        finalAdvanceStarted = true;
    }
    
    //! ************************************************************************
    //! STEP 8: RUN MOTORS TO ENSURE THEY MOVE
    //! ************************************************************************
    cutMotor.run();
    positionMotor.run();
    
    //! ************************************************************************
    //! STEP 9: CHECK FOR CYCLE CONTINUATION
    //! ************************************************************************
    if (finalAdvanceStarted && positionMotor.distanceToGo() == 0) {
        checkRunCycleSwitchForYeswood();
        
        // Reset state variables for next cycle
        cutMotorReturnStarted = false;
        positionMotorAdvanced = false;
        clampsSwapped = false;
        cutMotorHomeVerified = false;
        positionMotorHomeStarted = false;
        positionClampExtended = false;
        finalAdvanceStarted = false;
    }
} 