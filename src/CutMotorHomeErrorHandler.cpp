#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include "Functions.h"
#include "CutMotorHomeErrorHandler.h"

//* ************************************************************************
//* ****************** CUT MOTOR HOME ERROR HANDLER **********************
//* ************************************************************************
//! CRITICAL SAFETY SYSTEM: Cut motor home position error detection and recovery
//
//! SAFETY-CRITICAL IMPORTANCE:
//! Cut motor home detection is absolutely essential for machine safety.
//! If the cut motor fails to return to its true home position, and the position motor
//! subsequently pushes a wood board forward, the board could collide with the cut motor
//! blade or mechanism, causing:
//! 1. Catastrophic damage to the cutting blade
//! 2. Destruction of the workpiece  
//! 3. Potential injury to operators
//! 4. Misalignment of the entire cutting system
//! 5. Unpredictable machine behavior in subsequent cycles
//!
//! Therefore, this error detection system implements a recovery approach
//! that attempts to slowly move the cut motor back to home position.

// ========================================================================
//! RECOVERY SYSTEM CONFIGURATION
// ========================================================================

//! TIMEOUT AND SPEED CONSTANTS
const unsigned long CUT_MOTOR_HOME_RECOVERY_TIMEOUT_MS = 5000; // 5 second maximum recovery time
const float CUT_MOTOR_HOME_RECOVERY_SPEED = 1000; // Recovery speed (same as homing speed)

// ========================================================================
//! RESULT STRUCTURE CREATION HELPERS
// ========================================================================

//! SUCCESS RESULT - Home position detected successfully
CutMotorHomeErrorResult createSuccessResult() {
    CutMotorHomeErrorResult result;
    result.wasHomeDetected = true;
    result.shouldTransitionToError = false;
    result.shouldAttemptSlowRecovery = false;
    result.shouldContinueWithWarning = false;
    result.errorMessage = "";
    return result;
}

//! ERROR TRANSITION RESULT - Critical error requiring ERROR state
CutMotorHomeErrorResult createErrorTransitionResult(const String& message) {
    CutMotorHomeErrorResult result;
    result.wasHomeDetected = false;
    result.shouldTransitionToError = true;
    result.shouldAttemptSlowRecovery = false;
    result.shouldContinueWithWarning = false;
    result.errorMessage = message;
    return result;
}

//! WARNING RESULT - Issue detected but operation can continue
CutMotorHomeErrorResult createWarningOnlyResult(const String& message) {
    CutMotorHomeErrorResult result;
    result.wasHomeDetected = false;
    result.shouldTransitionToError = false;
    result.shouldAttemptSlowRecovery = false;
    result.shouldContinueWithWarning = true;
    result.errorMessage = message;
    return result;
}

// ========================================================================
//! REAL-TIME HOME SENSOR MONITORING SYSTEM
// ========================================================================

//! REAL-TIME CUT MOTOR HOME DETECTION DURING YES-WOOD RETURN
//! This function provides continuous monitoring of the cut motor home sensor
//! during yes-wood return sequences, implementing controlled deceleration
//! instead of abrupt stops for reliable sensor engagement.
void performCutMotorRealTimeHomeSensorCheck(FastAccelStepper* cutMotor, Bounce& cutHomingSwitch, bool& cutMotorInYesWoodReturn) {
    
    //! STATE MACHINE FOR CONTROLLED HOME DETECTION
    static enum {
        MONITORING,           // Normal monitoring state - watching for sensor activation
        DECELERATING,        // Sensor detected, motor decelerating within safety distance  
        WAITING_FOR_DELAY,   // Motor stopped, waiting for sensor stabilization
        VERIFYING_SENSOR     // Verification delay complete, confirming sensor state
    } realTimeCheckState = MONITORING;
    
    static unsigned long verificationDelayStartTime = 0;
    
    //! SAFETY DISTANCE AND TIMING CONSTANTS
    const float DECELERATION_DISTANCE_INCHES = 0.2; // Maximum 0.2 inch deceleration distance
    const unsigned long SENSOR_VERIFICATION_DELAY_MS = 30; // 30ms sensor stabilization delay
    
    //! ONLY ACTIVE DURING YES-WOOD RETURN SEQUENCES
    if (cutMotorInYesWoodReturn && cutMotor) {
        switch (realTimeCheckState) {
            
            //! MONITORING PHASE - Watch for home sensor activation during movement
            case MONITORING:
                if (cutMotor->isRunning() && cutHomingSwitch.read() == HIGH) {
                    //! HOME SENSOR DETECTED DURING MOVEMENT!
                    Serial.println("REAL-TIME DETECTION: Cut motor hit homing sensor during yes-wood return - beginning controlled deceleration...");
                    
                    //! Calculate safe target position for controlled stop
                    // Move further toward home (more negative) to ensure sensor is firmly pressed
                    long currentPosition = cutMotor->getCurrentPosition();
                    long targetPosition = currentPosition - (DECELERATION_DISTANCE_INCHES * CUT_MOTOR_STEPS_PER_INCH);
                    
                    //! Set high deceleration for quick but controlled stop within safety distance
                    cutMotor->setAcceleration(30000); // High deceleration for quick stop within 0.2 inch
                    cutMotor->moveTo(targetPosition);
                    
                    Serial.print("Decelerating from position ");
                    Serial.print(currentPosition);
                    Serial.print(" to target position ");
                    Serial.print(targetPosition);
                    Serial.print(" (");
                    Serial.print(DECELERATION_DISTANCE_INCHES);
                    Serial.println(" inch max distance)");
                    
                    realTimeCheckState = DECELERATING;
                }
                break;
                
            //! DECELERATION PHASE - Monitor motor until it comes to controlled stop
            case DECELERATING:
                // Check if motor has completed controlled deceleration
                if (!cutMotor->isRunning()) {
                    // Motor has stopped after controlled deceleration, start verification delay
                    verificationDelayStartTime = millis();
                    realTimeCheckState = WAITING_FOR_DELAY;
                    Serial.println("Cut motor deceleration complete. Starting 30ms sensor verification delay...");
                }
                //? Continue monitoring sensor during deceleration in case it goes LOW
                if (cutHomingSwitch.read() == LOW) {
                    Serial.println("WARNING: Home sensor went LOW during deceleration - possible contact issue");
                }
                break;
                
            //! DELAY PHASE - Wait for sensor to stabilize before verification
            case WAITING_FOR_DELAY:
                // Wait for the 30ms verification delay to complete
                if (millis() - verificationDelayStartTime >= SENSOR_VERIFICATION_DELAY_MS) {
                    realTimeCheckState = VERIFYING_SENSOR;
                }
                break;
                
            //! VERIFICATION PHASE - Confirm sensor is still active after delay
            case VERIFYING_SENSOR:
                cutHomingSwitch.update(); // Ensure latest sensor reading
                if (cutHomingSwitch.read() == HIGH) {
                    //! SUCCESSFUL HOME DETECTION WITH STABLE CONTACT
                    cutMotor->setCurrentPosition(0); // Recalibrate position to home
                    cutMotorInYesWoodReturn = false;  // Clear the yes-wood return flag
                    Serial.println("SUCCESS: Home sensor verified as stable after 30ms delay.");
                    Serial.println("Cut motor position recalibrated to 0, yes-wood return flag cleared.");
                } else {
                    //! FALSE TRIGGER OR INSUFFICIENT CONTACT
                    Serial.println("WARNING: Home sensor not active after 30ms verification delay.");
                    Serial.println("This was likely a false trigger or insufficient contact. Motor will continue movement.");
                    //? Note: cutMotorInYesWoodReturn flag remains true so movement can continue
                }
                realTimeCheckState = MONITORING; // Return to monitoring state
                break;
        }
    } else {
        //! RESET STATE MACHINE when not in yes-wood return mode
        if (realTimeCheckState != MONITORING) {
            Serial.println("Real-time check state reset - exiting yes-wood return mode");
            realTimeCheckState = MONITORING;
        }
    }
}

// ========================================================================
//! MAIN HOME ERROR DETECTION AND RECOVERY SYSTEM  
// ========================================================================

//! PRIMARY CUT MOTOR HOME ERROR DETECTION WITH RECOVERY CAPABILITY
//! This is the main function that handles all cut motor home position verification
//! and implements slow recovery when initial detection fails.
CutMotorHomeErrorResult handleCutMotorHomeError(
    Bounce& cutHomingSwitch, 
    FastAccelStepper* cutMotor, 
    const String& contextDescription,
    bool allowSlowRecovery
) {
    bool sensorDetectedHome = false;
    Serial.print("ERROR DETECTION: Checking cut motor home position for context: ");
    Serial.println(contextDescription);
    
    // ====================================================================
    //! PHASE 1: INITIAL HOME VERIFICATION (3-try approach)
    // ====================================================================
    
    for (int attemptNumber = 1; attemptNumber <= 3; attemptNumber++) {
        delay(30);  // Brief delay for sensor stabilization
        cutHomingSwitch.update();
        Serial.print("Initial home verification attempt ");
        Serial.print(attemptNumber);
        Serial.print(" of 3: ");
        Serial.println(cutHomingSwitch.read() == HIGH ? "HOME DETECTED" : "NO HOME");
        
        if (cutHomingSwitch.read() == HIGH) {
            sensorDetectedHome = true;
            if (cutMotor) {
                cutMotor->setCurrentPosition(0); // Recalibrate position to absolute zero
            }
            Serial.print("SUCCESS: Cut motor home position confirmed on initial check for ");
            Serial.println(contextDescription);
            return createSuccessResult();
        }
    }
    
    // ====================================================================
    //! PHASE 2: SLOW RECOVERY ATTEMPT (if enabled for this context)
    // ====================================================================
    
    if (!sensorDetectedHome && allowSlowRecovery) {
        Serial.println("INITIATING SLOW RECOVERY: Moving cut motor slowly back to home at homing speed...");
        
        if (cutMotor) {
            //! CONFIGURE MOTOR FOR RECOVERY MOVEMENT
            cutMotor->setSpeedInHz(CUT_MOTOR_HOME_RECOVERY_SPEED);
            cutMotor->setAcceleration(10000); // Moderate acceleration for controlled movement
            
            //! START RECOVERY MOVEMENT (backward toward home)
            cutMotor->runBackward();
            
            unsigned long recoveryStartTime = millis();
            bool homeFoundDuringRecovery = false;
            
            Serial.print("Slow recovery started at homing speed (");
            Serial.print(CUT_MOTOR_HOME_RECOVERY_SPEED);
            Serial.println(" steps/sec) with 5-second timeout...");
            
            //! MONITOR FOR HOME SENSOR DETECTION DURING RECOVERY
            while ((millis() - recoveryStartTime) < CUT_MOTOR_HOME_RECOVERY_TIMEOUT_MS) {
                cutHomingSwitch.update();
                
                if (cutHomingSwitch.read() == HIGH) {
                    //! HOME SENSOR DETECTED DURING RECOVERY!
                    cutMotor->forceStopAndNewPosition(0);
                    homeFoundDuringRecovery = true;
                    
                    unsigned long recoveryDuration = millis() - recoveryStartTime;
                    Serial.print("SUCCESS: Home sensor detected during slow recovery after ");
                    Serial.print(recoveryDuration);
                    Serial.println(" ms. Cut motor position recalibrated to 0.");
                    
                    String successMessage = "Recovery successful for " + contextDescription + 
                                          " after " + String(recoveryDuration) + " ms";
                    return createSuccessResult();
                }
                
                // Small delay to prevent excessive sensor polling
                delay(10);
            }
            
            //! RECOVERY TIMEOUT - Stop motor and report error
            cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
            
            if (!homeFoundDuringRecovery) {
                String timeoutErrorMessage = String("CRITICAL ERROR: Recovery timeout after 5 seconds. ") +
                                           String("Cut motor failed to find home position during slow recovery for context: ") + 
                                           contextDescription;
                Serial.println(timeoutErrorMessage);
                return createErrorTransitionResult(timeoutErrorMessage);
            }
        } else {
            String motorErrorMessage = "CRITICAL ERROR: Cut motor object is null during recovery for context: " + contextDescription;
            Serial.println(motorErrorMessage);
            return createErrorTransitionResult(motorErrorMessage);
        }
    } 
    // ====================================================================
    //! PHASE 3: HANDLE CASES WHERE SLOW RECOVERY IS DISABLED
    // ====================================================================
    
    else if (!allowSlowRecovery) {
        //? Slow recovery not allowed for this context (currently only used for no-wood sequences)
        String warningMessage = String("WARNING: Cut motor home sensor did not detect home for ") + contextDescription + 
                              String(", but slow recovery disabled. Proceeding with warning.");
        Serial.println(warningMessage);
        return createWarningOnlyResult(warningMessage);
    }
    
    // ====================================================================
    //! FINAL FALLBACK - All detection and recovery attempts failed
    // ====================================================================
    
    String finalErrorMessage = String("FAILED: Cut motor home sensor did not detect home position after 3 attempts. ") +
                             String("Context: ") + contextDescription;
    Serial.println(finalErrorMessage);
    return createErrorTransitionResult(finalErrorMessage);
}

// ========================================================================
//! ERROR STATE TRANSITION HANDLER
// ========================================================================

//! EXECUTE COMPLETE ERROR STATE TRANSITION WITH SAFETY MEASURES
//! This function handles the transition to ERROR state when cut motor home
//! detection fails, implementing all necessary safety measures.
void executeCutMotorErrorStateTransition(
    FastAccelStepper* cutMotor,
    FastAccelStepper* positionMotor,
    SystemState& currentState,
    int& cuttingStep,
    int& cuttingSubStep7,
    int& fixPositionStep,
    int& fixPositionSubStep2,
    unsigned long& errorStartTime,
    bool engageWoodSecureClamp
) {
    Serial.println("EXECUTING CUT MOTOR ERROR STATE TRANSITION");
    
    //! IMMEDIATE MOTOR SAFETY - Stop all movement immediately
    if (cutMotor) {
        cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
        Serial.println("Cut motor stopped and position locked.");
    }
    if (positionMotor) {
        positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());
        Serial.println("Position motor stopped and position locked.");
    }
    
    //! ENGAGE SAFETY CLAMPS - Secure all mechanical systems
    extendPositionClamp();  // Always engage position clamp for safety
    if (engageWoodSecureClamp) {
        extendWoodSecureClamp();
        Serial.println("Wood secure clamp engaged for safety.");
    }
    
    //! SET ERROR INDICATION LEDS - Visual status indicators
    turnRedLedOn();      // Red = Error condition
    turnYellowLedOff();  // Yellow off = Operation stopped
    Serial.println("Error LEDs activated (Red ON, Yellow OFF).");
    
    //! TRANSITION TO ERROR STATE
    currentState = ERROR;
    errorStartTime = millis();
    
    //! RESET ALL STATE MACHINE COUNTERS - Clean slate for restart
    cuttingStep = 0;
    cuttingSubStep7 = 0;
    fixPositionStep = 0;
    fixPositionSubStep2 = 0;
    
    Serial.println("System transitioned to ERROR state due to cut motor home detection failure.");
    Serial.println("User must acknowledge error with reload switch to continue.");
}

// ========================================================================
//! RESULT LOGGING AND REPORTING
// ========================================================================

//! LOG CUT MOTOR HOME ERROR RESULTS FOR DEBUGGING AND MONITORING
void logCutMotorHomeErrorResult(const CutMotorHomeErrorResult& result) {
    if (!result.errorMessage.isEmpty()) {
        Serial.print("Cut Motor Home Error Handler Result: ");
        Serial.println(result.errorMessage);
    }
    
    //! VISUAL STATUS INDICATORS FOR SERIAL MONITOR
    if (result.wasHomeDetected) {
        Serial.println("✓ Cut motor home position successfully verified.");
    } else if (result.shouldAttemptSlowRecovery) {
        Serial.println("→ Slow recovery at homing speed will be attempted.");
    } else if (result.shouldTransitionToError) {
        Serial.println("✗ ERROR STATE TRANSITION REQUIRED.");
    } else if (result.shouldContinueWithWarning) {
        Serial.println("⚠ Continuing with warning - monitor system closely.");
    }
} 