#include "ErrorStates/fix_cut_motor_position.h"
#include <Bounce2.h>
#include <FastAccelStepper.h>

// External references to global variables and objects from main.cpp
extern FastAccelStepper *cutMotor;
extern FastAccelStepper *positionMotor;
extern Bounce cutHomingSwitch;
extern bool cutMotorInYesWoodReturn;
extern SystemState currentState;
extern unsigned long errorStartTime;

// External references to constants from main.cpp
extern const float CUT_MOTOR_HOMING_SPEED;
extern const float POSITION_TRAVEL_DISTANCE;

// External references to functions from main.cpp (will need to be declared in Functions.h)
extern void extendPositionClamp();
extern void retractPositionClamp();
extern void retractWoodSecureClamp();
extern void extendWoodSecureClamp();
extern void configurePositionMotorForReturn();
extern void configureCutMotorForReturn();
extern void configurePositionMotorForNormalOperation();
extern void moveCutMotorToHome();
extern void movePositionMotorToYesWoodHome();
extern void movePositionMotorToPosition(float inches);
extern void allLedsOff();
extern void turnBlueLedOn();
extern void turnGreenLedOn();
extern void turnRedLedOn();
extern void turnYellowLedOn();
extern void stopCutMotor();

// Global variable for FIX_CUT_MOTOR_POSITION state steps
int fixCutMotorPositionStep = 0;

void initFixCutMotorPosition() {
    fixCutMotorPositionStep = 0;
}

//* ************************************************************************
//* ******************** CUT MOTOR POSITION FIX ****************************
//* ************************************************************************
// Handles manual cut motor home position verification and recalibration.
// This is a maintenance function specifically for situations where the cut motor
// may have lost its home position reference or the home sensor needs verification.
// The position motor movements are only to return the system to operational state
// after the cut motor position has been verified and recalibrated.
// 
// TRIGGERED BY: FIX_POSITION_BUTTON press while in READY state
// PURPOSE: Verify cut motor can properly detect its home position and recalibrate to 0
// 
// Step 0: Extend position clamp, start 100ms timer.
// Step 1: After 100ms, retract clamps, send both motors to home position.
// Step 2: Verify cut motor home sensor detection (3-try verification) and recalibrate position to 0.
//         If cut motor home sensor fails verification, transition to ERROR state.
// Step 3: Move position motor to operational position, engage clamps, return to READY.
void handleFixCutMotorPositionState() {
    static unsigned long step0Timer_fixCutMotorPosition = 0; 
    static int fixCutMotorPositionSubStep2 = 0;

    switch (fixCutMotorPositionStep) {
        case 0: // Initial action: extend position clamp and wait 100ms
            Serial.println("FIX_CUT_MOTOR_POSITION Step 0: Extending position clamp.");
            extendPositionClamp();
            step0Timer_fixCutMotorPosition = millis();
            fixCutMotorPositionStep = 1;
            break;

        case 1: // Wait for 100ms, then start the "yes-wood like" homing sequence
            if (millis() - step0Timer_fixCutMotorPosition >= 100) {
                Serial.println("FIX_CUT_MOTOR_POSITION Step 1: Position clamp extended. Starting cut motor home verification sequence.");
                retractPositionClamp(); 
                retractWoodSecureClamp(); 

                configurePositionMotorForReturn();
                configureCutMotorForReturn();      

                moveCutMotorToHome();
                movePositionMotorToYesWoodHome(); // Moves to 0
                cutMotorInYesWoodReturn = true; // Enable check for cut motor hitting home switch during move
                fixCutMotorPositionStep = 2;
                fixCutMotorPositionSubStep2 = 0; // Reset sub-step for case 2
            }
            break;

        case 2: // Wait for motors to reach home (position motor first, then cut)
            switch (fixCutMotorPositionSubStep2) {
                case 0: // Wait for position motor to home
                    if (positionMotor && !positionMotor->isRunning()) {
                        Serial.println("FIX_CUT_MOTOR_POSITION Step 2.0: Position motor at home. Engaging position clamp.");
                        extendPositionClamp();
                        fixCutMotorPositionSubStep2 = 1;
                    }
                    break;
                
                case 1: // Wait for cut motor to home, then check sensor
                    if (cutMotor && !cutMotor->isRunning()) {
                        Serial.println("FIX_CUT_MOTOR_POSITION Step 2.1: Cut motor at home. Starting slow recovery to verify home sensor.");
                        cutMotorInYesWoodReturn = false; // Disable special check now that motor has stopped

                        // Start slow recovery movement toward home
                        cutMotor->setSpeedInHz(CUT_MOTOR_HOMING_SPEED); // Use existing homing speed constant
                        cutMotor->setAcceleration(10000); // Moderate acceleration for controlled movement
                        cutMotor->runBackward(); // Move toward home
                        
                        unsigned long recoveryStartTime = millis();
                        bool homeFoundDuringRecovery = false;
                        const unsigned long RECOVERY_TIMEOUT_MS = 5000; // 5 second timeout
                        
                        Serial.print("FIX_CUT_MOTOR_POSITION: Slow recovery started at ");
                        Serial.print(CUT_MOTOR_HOMING_SPEED);
                        Serial.println(" steps/sec with 5-second timeout...");
                        
                        // Monitor for home sensor detection during recovery
                        while ((millis() - recoveryStartTime) < RECOVERY_TIMEOUT_MS) {
                            cutHomingSwitch.update();
                            
                            if (cutHomingSwitch.read() == HIGH) {
                                // Home sensor detected during recovery!
                                cutMotor->forceStopAndNewPosition(0);
                                homeFoundDuringRecovery = true;
                                
                                unsigned long recoveryDuration = millis() - recoveryStartTime;
                                Serial.print("SUCCESS: Cut motor home sensor detected during FIX_CUT_MOTOR_POSITION recovery after ");
                                Serial.print(recoveryDuration);
                                Serial.println(" ms. Position recalibrated to 0.");
                                break;
                            }
                            
                            delay(10); // Small delay to prevent excessive sensor polling
                        }
                        
                        if (!homeFoundDuringRecovery) {
                            // Recovery timeout - stop motor and transition to error
                            cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
                            Serial.println("FIX_CUT_MOTOR_POSITION Step 2.1 ERROR: Recovery timeout after 5 seconds. Cut motor home sensor NOT detected.");
                            stopCutMotor();
                            // positionMotor already stopped
                            allLedsOff();
                            turnRedLedOn();
                            turnYellowLedOn(); 
                            currentState = ERROR;
                            errorStartTime = millis(); 
                            fixCutMotorPositionStep = 0; // Reset main fix position step
                            fixCutMotorPositionSubStep2 = 0; // Reset sub-step
                        } else {
                            Serial.println("FIX_CUT_MOTOR_POSITION Step 2.1: Cut motor home confirmed via slow recovery. Moving position motor to final.");
                            retractWoodSecureClamp(); // Ensure wood secure clamp is retracted before final positioning
                            configurePositionMotorForNormalOperation();
                            movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
                            fixCutMotorPositionStep = 3;
                            fixCutMotorPositionSubStep2 = 0; // Reset sub-step
                        }
                    }
                    break;
            }
            break;

        case 3: // Wait for position motor to reach POSITION_TRAVEL_DISTANCE
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("FIX_CUT_MOTOR_POSITION Step 3: Position motor at final travel distance. Engaging wood secure clamp.");
                extendWoodSecureClamp();
                Serial.println("CUT MOTOR POSITION FIX: Verification complete. Cut motor home position confirmed and recalibrated.");
                allLedsOff(); 
                turnGreenLedOn();
                currentState = READY;
                fixCutMotorPositionStep = 0; // Reset for next time
            }
            break;
    }
} 