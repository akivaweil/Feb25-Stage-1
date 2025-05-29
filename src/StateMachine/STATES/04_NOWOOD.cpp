#include "StateMachine/StateMachine.h"

//* ************************************************************************
//* ************************ NOWOOD STATE ***************************
//* ************************************************************************
//! NOWOOD state implementation
//! Handles the sequence when no wood is detected
//! Steps: 1) Retract secure wood clamp, 2) Move position motor to -1, 
//! 3) Return cut motor to position 0 (simultaneous), 4) Retract then extend position clamp,
//! 5) Move position motor to POSITION_TRAVEL_DISTANCE, 6) Return to IDLE state

void executeNOWOOD() {
    static int nowoodStep = 1;
    static bool stepInitiated = false;
    static unsigned long stepStartTime = 0;
    
    switch(nowoodStep) {
        case 1:
            //! Step 1: Retract the secure wood clamp
            if (!stepInitiated) {
                Serial.println("NOWOOD: Step 1 - Retract secure wood clamp");
                retractClamp(WOOD_SECURE_CLAMP_TYPE);
                stepInitiated = true;
                stepStartTime = millis();
            }
            
            // Wait for clamp operation to complete
            if (millis() - stepStartTime >= 500) {  // 500ms delay for clamp operation
                stepInitiated = false;
                nowoodStep = 2;
            }
            break;
            
        case 2:
            //! Step 2: Move position motor to -1 and simultaneously return cut motor to position 0
            if (!stepInitiated) {
                Serial.println("NOWOOD: Step 2 - Move position motor to -1 and cut motor to 0 (simultaneous)");
                
                // Move position motor to -1 step
                if (positionMotor) {
                    positionMotor->moveTo(-1);
                }
                
                // Simultaneously return cut motor to position 0
                if (cutMotor) {
                    cutMotor->moveTo(0);
                }
                
                stepInitiated = true;
                stepStartTime = millis();
            }
            
            // Wait for both motors to complete their moves
            bool positionMotorDone = !positionMotor || !positionMotor->isRunning();
            bool cutMotorDone = !cutMotor || !cutMotor->isRunning();
            
            if (positionMotorDone && cutMotorDone) {
                Serial.println("NOWOOD: Both motors completed movement");
                stepInitiated = false;
                nowoodStep = 3;
            }
            
            // Timeout safety
            if (millis() - stepStartTime >= 10000) {  // 10 second timeout
                Serial.println("NOWOOD: Motor movement timeout");
                stepInitiated = false;
                nowoodStep = 3;
            }
            break;
            
        case 3:
            //! Step 3: Retract position clamp
            if (!stepInitiated) {
                Serial.println("NOWOOD: Step 3 - Retract position clamp");
                retractClamp(POSITION_CLAMP_TYPE);
                stepInitiated = true;
                stepStartTime = millis();
            }
            
            // Wait for clamp operation to complete
            if (millis() - stepStartTime >= 500) {  // 500ms delay for clamp operation
                stepInitiated = false;
                nowoodStep = 4;
            }
            break;
            
        case 4:
            //! Step 4: Extend position clamp
            if (!stepInitiated) {
                Serial.println("NOWOOD: Step 4 - Extend position clamp");
                extendClamp(POSITION_CLAMP_TYPE);
                stepInitiated = true;
                stepStartTime = millis();
            }
            
            // Wait for clamp operation to complete
            if (millis() - stepStartTime >= 500) {  // 500ms delay for clamp operation
                stepInitiated = false;
                nowoodStep = 5;
            }
            break;
            
        case 5:
            //! Step 5: Move position motor to POSITION_TRAVEL_DISTANCE
            if (!stepInitiated) {
                Serial.println("NOWOOD: Step 5 - Move position motor to POSITION_TRAVEL_DISTANCE");
                
                if (positionMotor) {
                    long travelSteps = (long)(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
                    positionMotor->moveTo(travelSteps);
                    Serial.print("Moving to position: ");
                    Serial.print(POSITION_TRAVEL_DISTANCE);
                    Serial.print(" inches (");
                    Serial.print(travelSteps);
                    Serial.println(" steps)");
                }
                
                stepInitiated = true;
                stepStartTime = millis();
            }
            
            // Wait for motor to complete movement
            if (!positionMotor || !positionMotor->isRunning()) {
                Serial.println("NOWOOD: Position motor reached POSITION_TRAVEL_DISTANCE");
                stepInitiated = false;
                nowoodStep = 6;
            }
            
            // Timeout safety
            if (millis() - stepStartTime >= 10000) {  // 10 second timeout
                Serial.println("NOWOOD: Position motor movement timeout");
                stepInitiated = false;
                nowoodStep = 6;
            }
            break;
            
        case 6:
            //! Step 6: Return to IDLE state
            Serial.println("NOWOOD: Step 6 - Returning to IDLE state");
            
            // Reset step counter for next execution
            nowoodStep = 1;
            stepInitiated = false;
            
            // Transition to IDLE state
            changeState(IDLE);
            break;
    }
} 