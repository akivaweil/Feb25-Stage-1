#include "../../../include/StateMachine/StateMachine.h"
#include "Config/Config.h"
#include <FastAccelStepper.h>
#include <Bounce2.h>

// External variable declarations
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;
extern Bounce woodSensor;
extern Bounce waswoodsuctionedSensor;
extern StateType currentState;

//* ************************************************************************
//* ************************ CUTTING FUNCTIONS ***************************
//* ************************************************************************
//! CUTTING-specific functions for wood cutting operations
//! These functions handle all aspects of the cutting sequence with clear,
//! readable implementations specific to the CUTTING state

//* ************************************************************************
//* ************************ CLAMP OPERATIONS FOR CUTTING ****************
//* ************************************************************************

void extendBothClampsForCutting() {
    // Extend position clamp
    digitalWrite(POSITION_CLAMP, LOW);
    Serial.println("CUTTING: Position clamp extended");
    
    // Extend wood secure clamp
    digitalWrite(WOOD_SECURE_CLAMP, LOW);
    Serial.println("CUTTING: Wood secure clamp extended");
    
    Serial.println("CUTTING: Both clamps extended - wood secured for cutting");
}

//* ************************************************************************
//* ************************ MOTOR OPERATIONS FOR CUTTING ****************
//* ************************************************************************

void startCutMotorMovementForCutting() {
    if (!cutMotor) {
        Serial.println("ERROR: Cut motor not initialized");
        return;
    }
    
    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_NORMAL_SPEED);
    cutMotor->moveTo(CUT_MOTOR_CUT_POSITION);
    Serial.print("CUTTING: Cut motor started - moving to position ");
    Serial.println(CUT_MOTOR_CUT_POSITION);
}

bool checkCutMotorSafetyAt03Inches() {
    if (!cutMotor) return false;
    
    // Calculate 0.3 inch position
    float safetyCheckPosition = 0.3 * CUT_MOTOR_STEPS_PER_INCH;
    
    // Check if motor has reached 0.3 inches
    if (cutMotor->getCurrentPosition() >= safetyCheckPosition) {
        waswoodsuctionedSensor.update();
        if (waswoodsuctionedSensor.read() == LOW) {
            Serial.println("CUTTING: SAFETY VIOLATION - Wood suctioned sensor activated at 0.3 inches");
            cutMotor->stopMove();
            // TODO: Enter waswoodsuctioned error state
            return false;
        }
        Serial.println("CUTTING: Safety check passed at 0.3 inches");
    }
    return true;
}

bool checkCatcherClampActivationPoint() {
    if (!cutMotor) return false;
    
    // Calculate activation point
    float activationPosition = CUT_MOTOR_CUT_POSITION - (CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES * CUT_MOTOR_STEPS_PER_INCH);
    
    if (cutMotor->getCurrentPosition() >= activationPosition) {
        // Activate catcher clamp
        digitalWrite(CATCHER_CLAMP_PIN, LOW);
        Serial.println("CUTTING: Catcher clamp activated at early activation offset");
        return true;
    }
    return false;
}

bool checkCatcherServoActivationPoint() {
    if (!cutMotor) return false;
    
    // Calculate activation point
    float activationPosition = CUT_MOTOR_CUT_POSITION - (CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES * CUT_MOTOR_STEPS_PER_INCH);
    
    if (cutMotor->getCurrentPosition() >= activationPosition) {
        // Activate catcher servo
        // TODO: Add servo activation code
        Serial.println("CUTTING: Catcher servo activated at early activation offset");
        return true;
    }
    return false;
}

//* ************************************************************************
//* ************************ WOOD SENSOR CHECKING FOR CUTTING ************
//* ************************************************************************

bool checkWoodSensorForStateTransition() {
    woodSensor.update();
    if (woodSensor.read() == LOW) {
        Serial.println("CUTTING: Wood detected - transitioning to YESWOOD");
        currentState = YESWOOD;
        return true;
    } else {
        Serial.println("CUTTING: No wood detected - transitioning to NOWOOD");
        currentState = NOWOOD;
        return true;
    }
}

//* ************************************************************************
//* ************************ MAIN CUTTING EXECUTION FUNCTION *************
//* ************************************************************************

void executeCuttingSequence() {
    static bool clampsExtended = false;
    static bool cutMotorStarted = false;
    static bool safetyChecked = false;
    static bool catcherClampActivated = false;
    static bool catcherServoActivated = false;
    
    //! ************************************************************************
    //! STEP 1: EXTEND BOTH CLAMPS (ONE TIME)
    //! ************************************************************************
    if (!clampsExtended) {
        extendBothClampsForCutting();
        clampsExtended = true;
    }
    
    //! ************************************************************************
    //! STEP 2: START CUT MOTOR MOVEMENT (ONE TIME)
    //! ************************************************************************
    if (!cutMotorStarted) {
        startCutMotorMovementForCutting();
        cutMotorStarted = true;
    }
    
    //! ************************************************************************
    //! STEP 3: CONTINUOUS SAFETY AND ACTIVATION CHECKS
    //! ************************************************************************
    if (cutMotor && cutMotor->isRunning()) {
        // Safety check at 0.3 inches
        if (!safetyChecked) {
            if (!checkCutMotorSafetyAt03Inches()) {
                // Reset state variables on error
                clampsExtended = false;
                cutMotorStarted = false;
                safetyChecked = false;
                return;
            }
            safetyChecked = true;
        }
        
        // Catcher clamp activation check
        if (!catcherClampActivated) {
            catcherClampActivated = checkCatcherClampActivationPoint();
        }
        
        // Catcher servo activation check
        if (!catcherServoActivated) {
            catcherServoActivated = checkCatcherServoActivationPoint();
        }
    }
    
    //! ************************************************************************
    //! STEP 4: CHECK IF CUT IS COMPLETE AND ROUTE TO NEXT STATE
    //! ************************************************************************
    if (cutMotor && !cutMotor->isRunning()) {
        Serial.println("CUTTING: Cut motor movement complete - checking wood sensor");
        checkWoodSensorForStateTransition();
        
        // Reset state variables for next cycle
        clampsExtended = false;
        cutMotorStarted = false;
        safetyChecked = false;
        catcherClampActivated = false;
        catcherServoActivated = false;
    }
}