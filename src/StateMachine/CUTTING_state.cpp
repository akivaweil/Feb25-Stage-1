#include "StateMachine/CuttingState.h"
#include "StateMachine/StateManager.h"
#include "Functions.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Handles the wood cutting operation.
// This state manages a multi-step cutting process.
// It includes logic for normal cutting, deciding between a YES_WOOD Sequence and a NO_WOOD Sequence, and error handling.

void CuttingState::onEnter(StateManager& stateManager) {
    // Reset all step counters when entering cutting state
    resetSteps();
}

void CuttingState::onExit(StateManager& stateManager) {
    // Reset all step counters when exiting cutting state
    resetSteps();
}

void CuttingState::execute(StateManager& stateManager) {
    Serial.println("Current State: CUTTING");
    
    if (homePositionErrorDetected) {
        handleHomePositionError(stateManager);
        return;
    }
    
    // Handle signal timing independently of motor movements
    if (signalActive && millis() - signalStartTime >= 2000) { // Original was 2000ms
        signalActive = false;
    }

    switch (cuttingStep) {
        case 0: 
            handleCuttingStep0(stateManager);
            break;
        case 1: 
            handleCuttingStep1(stateManager);
            break;
        case 2: 
            handleCuttingStep2(stateManager);
            break;
        case 3: 
            handleCuttingStep3(stateManager);
            break;
        case 4: 
            handleCuttingStep4(stateManager);
            break;
        case 5: 
            handleCuttingStep5(stateManager);
            break;
                case 6:
            handleCuttingStep6_NO_WOOD_Sequence(stateManager);
            break;
        case 7: 
            handleCuttingStep7_YES_WOOD_Sequence(stateManager);
            break;
    }
}

void CuttingState::handleCuttingStep0(StateManager& stateManager) {
    Serial.println("Cutting Step 0: Starting cut motion."); 
    moveCutMotorToCut();
    catcherClampActivatedThisCycle = false; // Reset for this cut cycle
    cuttingStep = 1;
}

void CuttingState::handleCuttingStep1(StateManager& stateManager) {
    // Modified to use >= as per original file content for cutMotor->getCurrentPosition()
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const int CUT_MOTOR_STEPS_PER_INCH; // From main.cpp
    extern const int WAS_WOOD_SUCTIONED_SENSOR; // From main.cpp
    
    if (cutMotor && cutMotor->getCurrentPosition() >= (1.0 * CUT_MOTOR_STEPS_PER_INCH)) {
        Serial.println("Cutting Step 1: Cut motor at >= 1 inch, checking wood suction.");
        if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == LOW) { // LOW means NO SUCTION (Error condition)
            Serial.println("Wood suction error detected! Waiting for cycle switch OFF then ON to reset.");
            // Stop motors
            if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
            FastAccelStepper* positionMotor = stateManager.getPositionMotor();
            if (positionMotor) positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());

            stateManager.setCuttingCycleInProgress(false);
            resetSteps();

            // Set LEDs for suction error state
            turnRedLedOn();
            turnYellowLedOff();
            turnGreenLedOff();
            turnBlueLedOff();

            stateManager.changeState(SUCTION_ERROR_HOLD); // Transition to new error hold state
        } else { // Sensor is HIGH, suction is OK
            cuttingStep = 2;
            Serial.println("Cutting Step 1: Wood suction OK (or not present). Proceeding to step 2.");
        }
    }
}

void CuttingState::handleCuttingStep2(StateManager& stateManager) {
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const int CUT_MOTOR_STEPS_PER_INCH; // From main.cpp
    extern const float CUT_TRAVEL_DISTANCE; // From main.cpp
    extern const float CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES; // From main.cpp
    extern const int WOOD_SENSOR; // From main.cpp
    
    // Early Catcher Clamp Activation
    if (!catcherClampActivatedThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= ((CUT_TRAVEL_DISTANCE - CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH)) {
        Serial.println("Cutting Step 2: Activating Catcher Clamp (early).");
        extendCatcherClamp(); // Engage Catcher Clamp
        catcherClampActivatedThisCycle = true;
    }

    // Check for full cut completion
    if (cutMotor && !cutMotor->isRunning()) {
        Serial.println("Cutting Step 2: Cut fully complete."); 
        sendSignalToTA(); // Signal to Transfer Arm
        configureCutMotorForReturn();

        int sensorValue = digitalRead(WOOD_SENSOR);
        bool noWoodDetected = (sensorValue == HIGH);
        
        if (noWoodDetected) {
            Serial.println("Cutting Step 2: NO_WOOD state - Wood sensor reads HIGH. Entering NO_WOOD Sequence (handled in cuttingStep 6).");
            configureCutMotorForReturn();
            moveCutMotorToHome();
            configurePositionMotorForNormalOperation();
            cuttingStep = 6;
        } else {
            Serial.println("Cutting Step 2: YES_WOOD state - Wood sensor reads LOW. Entering YES_WOOD Sequence (simultaneous return, handled in cuttingStep 7)."); 
            configurePositionMotorForReturn();
            
            retractPositionClamp(); 
            retractWoodSecureClamp(); 
            Serial.println("Position and Wood Secure clamps disengaged for simultaneous return.");

            // Set flag to indicate we're in YES_WOOD return mode - enable homing sensor check
            extern bool cutMotorInYesWoodReturn; // From main.cpp
            cutMotorInYesWoodReturn = true;
            moveCutMotorToHome();
            movePositionMotorToYesWoodHome();
            
            cuttingStep = 7;
            cuttingSubStep7 = 0; // Reset sub-step for case 7
        }
    }
}

void CuttingState::handleCuttingStep3(StateManager& stateManager) {
    Serial.println("Cutting Step 3: (Should be bypassed for wood path) Initial position move complete.");
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    if (positionMotor && !positionMotor->isRunning()) {
        retractPositionClamp();
        retractWoodSecureClamp();
        Serial.println("Position clamp and wood secure clamp disengaged.");

        configurePositionMotorForReturn();
        movePositionMotorToHome();
        Serial.println("Position motor moving to home (0).");
    }
}

void CuttingState::handleCuttingStep4(StateManager& stateManager) {
    Serial.println("Cutting Step 4: (Logic moved to Step 7 for wood path) Position motor at home (0).");
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES; // From main.cpp
    extern const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES; // From main.cpp
    extern const int CUT_MOTOR_STEPS_PER_INCH; // From main.cpp
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    
    if (positionMotor && !positionMotor->isRunning()) {
        retractPositionClamp();
        Serial.println("Position clamp disengaged.");

        if (cutMotor && !cutMotor->isRunning()) {
            Serial.println("Cut motor also at home. Checking cut motor position switch.");
            bool sensorDetectedHome = false;
            for (int i = 0; i < 3; i++) {
                delay(30);
                stateManager.getCutHomingSwitch()->update();
                Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(stateManager.getCutHomingSwitch()->read());
                if (stateManager.getCutHomingSwitch()->read() == HIGH) {
                    sensorDetectedHome = true;
                    if (cutMotor) cutMotor->setCurrentPosition(0); // Recalibrate to 0 when switch is hit
                    Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
                    break;
                }
            }
            if (!sensorDetectedHome) {
                Serial.println("ERROR: Cut motor position switch did not detect home after return attempt.");
                if (cutMotorIncrementalMoveTotalInches < CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES) {
                    Serial.print("Attempting incremental move. Total moved: ");
                    Serial.print(cutMotorIncrementalMoveTotalInches);
                    Serial.println(" inches.");
                    cutMotor->move(-CUT_MOTOR_INCREMENTAL_MOVE_INCHES * CUT_MOTOR_STEPS_PER_INCH);
                    cutMotorIncrementalMoveTotalInches += CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
                    // Stay in cuttingStep 4 to re-check sensor after move
                } else {
                    Serial.println("ERROR: Cut motor position switch did not detect home after MAX incremental moves!");
                    stopCutMotor();
                    stopPositionMotor();
                    extendPositionClamp();
                    turnRedLedOn();
                    turnYellowLedOff();
                    stateManager.changeState(ERROR);
                    stateManager.setErrorStartTime(millis());
                    resetSteps();
                    cutMotorIncrementalMoveTotalInches = 0.0; // Reset for next attempt
                    Serial.println("Transitioning to ERROR state due to cut motor homing failure after cut.");
                }
            } else {
                Serial.println("Cut motor position switch confirmed home. Moving position motor to final position.");
                cutMotorIncrementalMoveTotalInches = 0.0; // Reset on success
                movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
                cuttingStep = 5; 
            }
        }
    }
}

void CuttingState::handleCuttingStep5(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    if (positionMotor && !positionMotor->isRunning()) {
        Serial.println("Cutting Step 5: Position motor at final position. Cycle complete."); 
        extendWoodSecureClamp(); 
        Serial.println("Wood secure clamp engaged."); 
        turnYellowLedOff();
        stateManager.setCuttingCycleInProgress(false);
        stateManager.changeState(READY);
        resetSteps();
        Serial.println("Transitioning to READY state."); 
    }
}

void CuttingState::handleCuttingStep6_NO_WOOD_Sequence(StateManager& stateManager) {
    // NO_WOOD sequence logic
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    const unsigned long CYLINDER_ACTION_DELAY_MS = 150;
    
    if (noWoodStep == 0) { // First time entering this specific NO_WOOD logic path
        Serial.println("NO_WOOD Step 6.0: Initiating position motor to home & retracting wood secure clamp.");
        retractWoodSecureClamp();
        if (positionMotor) {
            if (positionMotor->getCurrentPosition() != 0 || positionMotor->isRunning()) {
                positionMotor->moveTo(0);
                Serial.println("NO_WOOD Step 6.0: Position motor commanded to home.");
            } else {
                Serial.println("NO_WOOD Step 6.0: Position motor already at home.");
            }
        }
        noWoodStep = 1;
    }

    if (waitingForCylinder && (millis() - cylinderActionTime >= CYLINDER_ACTION_DELAY_MS)) {
        waitingForCylinder = false;
        noWoodStep++; 
    }
    
    if (!waitingForCylinder) {
        handleNO_WOOD_Step(stateManager, noWoodStep);
    }
}

void CuttingState::handleNO_WOOD_Step(StateManager& stateManager, int step) {
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    
    switch (step) { 
        case 1: // New Step: Wait for cut motor, then engage position clamp
            if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6.1 (new): Cut motor returned home. Engaging position clamp.");
                extendPositionClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Will cause noWoodStep to increment to 2 after delay
            }
            break;
            
        case 2: // Was original noWoodStep 1: wait for position motor, then retract position clamp
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6.2 (was 6.1): Position motor at home. Disengaging position clamp."); 
                retractPositionClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 3
            }
            break;
            
        case 3: // Was original noWoodStep 2: move position motor to 2.0 inches
            Serial.println("NO_WOOD Step 6.3 (was 6.2): Moving position motor to 2.0 inches."); 
            configurePositionMotorForNormalOperation(); // Ensure correct config
            movePositionMotorToPosition(2.0);
            noWoodStep = 4; // Directly advance step here as it's a command
            break;
            
        case 4: // Was original noWoodStep 3: wait for position motor at 2.0, engage position clamp
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6.4 (was 6.3): Position motor at 2.0 inches. Engaging position clamp."); 
                extendPositionClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 5
            }
            break;
            
        case 5: // Was original noWoodStep 4: move position motor to home
            Serial.println("NO_WOOD Step 6.5 (was 6.4): Moving position motor to home."); 
            configurePositionMotorForNormalOperation();
            movePositionMotorToHome();
            noWoodStep = 6; // Directly advance step
            break;
            
        case 6: // Was original noWoodStep 5: wait for position motor at home, retract position clamp
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6.6 (was 6.5): Position motor at home. Disengaging position clamp."); 
                retractPositionClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 7
            }
            break;
            
        case 7: // Was original noWoodStep 6: move position motor to final position
            Serial.println("NO_WOOD Step 6.7 (was 6.6): Moving position motor to final position (POSITION_TRAVEL_DISTANCE)."); 
            configurePositionMotorForNormalOperation();
            movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
            noWoodStep = 8; // Directly advance step
            break;
            
        case 8: // Was original noWoodStep 7: wait for motor, check cut home, cleanup
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6.8 (was 6.7): Position motor at final position."); 
                bool sensorDetectedHome = false;
                Serial.println("NO_WOOD Step 6.8: Checking cut motor position switch."); 
                for (int i = 0; i < 3; i++) {
                    delay(30);  
                    stateManager.getCutHomingSwitch()->update();
                    if (stateManager.getCutHomingSwitch()->read() == HIGH) {
                        sensorDetectedHome = true;
                        if (cutMotor) cutMotor->setCurrentPosition(0); 
                        Serial.println("Cut motor position switch detected HIGH during NO_WOOD sequence completion."); 
                        break;  
                    }
                }
                if (!sensorDetectedHome) {
                    Serial.println("WARNING: Cut motor position switch did NOT detect home after NO_WOOD sequence, but proceeding anyway."); 
                }

                retractWoodSecureClamp(); 
                Serial.println("Wood secure clamp disengaged (final check in NO_WOOD)."); 
                turnYellowLedOff();
                turnBlueLedOn(); 

                resetSteps();
                stateManager.setCuttingCycleInProgress(false);
                stateManager.changeState(READY);
                stateManager.setContinuousModeActive(false);
                
                // Check if cycle switch is currently ON - if yes, require cycling
                if (stateManager.getStartCycleSwitch()->read() == HIGH) {
                    stateManager.setStartSwitchSafe(false);
                    Serial.println("Cycle switch is still ON - must be cycled OFF then ON for next cycle.");
                } else {
                    Serial.println("Cycle switch is OFF - ready for next cycle.");
                }
                
                Serial.println("NO_WOOD sequence complete. Transitioning to READY state. Continuous mode OFF."); 
            }
            break;
    }
}

void CuttingState::handleCuttingStep7_YES_WOOD_Sequence(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    extern bool cutMotorInYesWoodReturn; // From main.cpp
    
    // This step handles the completion of the "YES_WOOD Sequence".
    switch (cuttingSubStep7) {
        case 0: // Wait for position motor to home
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("Cutting Step 7.0: Position motor has returned home. Engaging position clamp.");
                extendPositionClamp();
                Serial.println("Position clamp engaged (position motor home).");
                cuttingSubStep7 = 1;
            }
            break;

        case 1: // Wait for cut motor to home, then proceed with original logic
            if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("Cutting Step 7.1: Cut motor has returned home.");
                // Clear the YES_WOOD return flag since cut motor has stopped
                cutMotorInYesWoodReturn = false;

                // Check the cut motor homing switch. If not detected, transition to ERROR.
                bool sensorDetectedHome = false;
                Serial.println("Checking cut motor position switch after simultaneous return.");
                for (int i = 0; i < 3; i++) { 
                    delay(30);  
                    stateManager.getCutHomingSwitch()->update();
                    Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(stateManager.getCutHomingSwitch()->read());
                    if (stateManager.getCutHomingSwitch()->read() == HIGH) {
                        sensorDetectedHome = true;
                        if (cutMotor) cutMotor->setCurrentPosition(0); 
                        Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
                        break; 
                    }
                }

                if (!sensorDetectedHome) {
                    // Homing failed, transition to ERROR state.
                    Serial.println("ERROR: Cut motor position switch did not detect home after simultaneous return!");
                    stopCutMotor();
                    extendWoodSecureClamp(); 
                    turnRedLedOn();
                    turnYellowLedOff(); 
                    stateManager.changeState(ERROR);
                    stateManager.setErrorStartTime(millis());
                    resetSteps();
                } else {
                    // Homing successful, proceed with next steps.
                    retractWoodSecureClamp();
                    Serial.println("Wood secure clamp retracted after successful cut motor home detection.");

                    Serial.println("Cut motor position switch confirmed home. Proceeding to move position motor to final travel position.");
                    configurePositionMotorForNormalOperation();
                    movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
                    cuttingStep = 5; // Transition to original cuttingStep 5
                    cuttingSubStep7 = 0; // Reset sub-step
                }
            }
            break;
    }
}

void CuttingState::handleHomePositionError(StateManager& stateManager) {
    Serial.println("Home position error detected during cutting operation."); 
    unsigned long lastErrorBlinkTime = stateManager.getLastErrorBlinkTime();
    bool errorBlinkState = stateManager.getErrorBlinkState();
    
    if (millis() - lastErrorBlinkTime > 100) { 
        errorBlinkState = !errorBlinkState;
        stateManager.setErrorBlinkState(errorBlinkState);
        if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        stateManager.setLastErrorBlinkTime(millis());
    }
    
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
    if (positionMotor) positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());
    
    extendPositionClamp();
    extendWoodSecureClamp();
    
    if (stateManager.getReloadSwitch()->rose()) {
        homePositionErrorDetected = false;
        stateManager.changeState(ERROR_RESET);
        stateManager.setErrorAcknowledged(true);
        Serial.println("Home position error acknowledged by reload switch."); 
    }
}

void CuttingState::resetSteps() {
    cuttingStep = 0;
    signalStartTime = 0;
    signalActive = false;
    homePositionErrorDetected = false;
    catcherClampActivatedThisCycle = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
    cuttingSubStep7 = 0;
    
    noWoodStep = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
} 