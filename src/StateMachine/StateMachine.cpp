#include "StateMachine/StateMachine.h"

//* ************************************************************************
//* ************************ STATE MACHINE IMPLEMENTATION ***************************
//* ************************************************************************
//! Main state machine implementation
//! Handles state transitions and execution of state functions

// Global State Variables
SystemState currentState = IDLE;
SystemState previousState = IDLE;
bool stateChanged = false;

void initializeStateMachine() {
    //* ************************************************************************
    //* ************************ STATE MACHINE INITIALIZATION ***************************
    //* ************************************************************************
    //! Initialize the state machine to IDLE state
    
    currentState = IDLE;
    previousState = IDLE;
    stateChanged = true;
    
    Serial.println("State machine initialized to IDLE");
}

void updateStateMachine() {
    //* ************************************************************************
    //* ************************ STATE MACHINE UPDATE ***************************
    //* ************************************************************************
    //! Main state machine update function called in main loop
    
    // Check for state transitions
    if (checkTransitionConditions()) {
        // Transition logic will be handled in checkTransitionConditions
    }
    
    // Execute current state
    switch (currentState) {
        case STARTUP:
            // STARTUP state will initialize and transition to HOMING
            changeState(HOMING);
            break;
        case IDLE:
            executeIDLE();
            break;
        case HOMING:
            executeHOMING();
            break;
        case CUTTING:
            executeCUTTING();
            break;
        case YESWOOD:
            // executeYESWOOD();  // Commented out due to compilation issues
            Serial.println("In YESWOOD state");
            break;
        case NOWOOD:
            // executeNOWOOD();  // Commented out due to compilation issues
            Serial.println("In NOWOOD state");
            break;
        case pushWoodForwardOne:
            // executepushWoodForwardOne();  // Commented out due to compilation issues
            Serial.println("In pushWoodForwardOne state");
            break;
        case RELOAD:
            executeRELOAD();
            break;
        case ERROR:
            // Handle error state - blink red LED
            Serial.println("In ERROR state");
            break;
        case ERROR_RESET:
            // Reset from error state
            changeState(IDLE);
            break;
        default:
            Serial.println("ERROR: Unknown state detected, returning to IDLE");
            changeState(IDLE);
            break;
    }
    
    // Update status LED based on current state
    updateStatusLED();
}

void changeState(SystemState newState) {
    //* ************************************************************************
    //* ************************ STATE CHANGE ***************************
    //* ************************************************************************
    //! Change to a new state and handle state change logic
    
    if (newState != currentState) {
        previousState = currentState;
        currentState = newState;
        stateChanged = true;
        
        printStateChange();
    }
}

bool checkTransitionConditions() {
    //* ************************************************************************
    //* ************************ TRANSITION CONDITIONS ***************************
    //* ************************************************************************
    //! Check conditions for state transitions
    
    switch (currentState) {
        case STARTUP:
            // Transition to HOMING automatically
            changeState(HOMING);
            return true;
            
        case IDLE:
            if (areAllSystemsReady()) {
                changeState(HOMING);
                return true;
            }
            break;
            
        case HOMING:
            if (isHomingComplete()) {
                changeState(IDLE);
                return true;
            }
            break;
            
        case CUTTING:
            // Add specific cutting transition conditions here
            break;
            
        case YESWOOD:
            // Add specific YESWOOD transition conditions here
            break;
            
        case NOWOOD:
            // Add specific NOWOOD transition conditions here
            break;
            
        case pushWoodForwardOne:
            // Add specific pushWoodForwardOne transition conditions here
            break;
            
        case RELOAD:
            // RELOAD state handles its own transitions internally
            // No external transition conditions needed
            break;
            
        case ERROR:
            // Add specific error transition conditions here
            break;
            
        case ERROR_RESET:
            // Add specific error reset transition conditions here
            break;
    }
    
    return false;
}

bool areAllSystemsReady() {
    //* ************************************************************************
    //* ************************ SYSTEMS READY CHECK ***************************
    //* ************************************************************************
    //! Check if all systems are ready for operation
    
    // Add specific system ready checks here
    // For now, return true as placeholder
    return true;
}

bool isHomingComplete() {
    //* ************************************************************************
    //* ************************ HOMING COMPLETE CHECK ***************************
    //* ************************************************************************
    //! Check if homing sequence is complete
    
    // Use the new homing functions to check completion
    return isHomingSequenceComplete();
}

void printStateChange() {
    //* ************************************************************************
    //* ************************ STATE CHANGE LOGGING ***************************
    //* ************************************************************************
    //! Print state change information to serial monitor
    
    Serial.print("State changed from ");
    Serial.print(previousState);
    Serial.print(" to ");
    Serial.println(currentState);
}

void updateStatusLED() {
    //* ************************************************************************
    //* ************************ STATUS LED UPDATE ***************************
    //* ************************************************************************
    //! Update status LED based on current state
    
    static unsigned long lastLEDUpdate = 0;
    static bool ledState = false;
    unsigned long currentTime = millis();
    
    switch (currentState) {
        case STARTUP:
            // Solid blue during startup
            digitalWrite(RED_LED, LOW);
            digitalWrite(YELLOW_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(BLUE_LED, HIGH);
            break;
            
        case IDLE:
            // Solid green
            digitalWrite(RED_LED, LOW);
            digitalWrite(YELLOW_LED, LOW);
            digitalWrite(GREEN_LED, HIGH);
            digitalWrite(BLUE_LED, LOW);
            break;
            
        case HOMING:
            // Slow blink blue
            if (currentTime - lastLEDUpdate > 500) {
                ledState = !ledState;
                digitalWrite(RED_LED, LOW);
                digitalWrite(YELLOW_LED, LOW);
                digitalWrite(GREEN_LED, LOW);
                digitalWrite(BLUE_LED, ledState);
                lastLEDUpdate = currentTime;
            }
            break;
            
        case CUTTING:
            // Solid yellow
            digitalWrite(RED_LED, LOW);
            digitalWrite(YELLOW_LED, HIGH);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(BLUE_LED, LOW);
            break;
            
        case YESWOOD:
            // Fast blink yellow
            if (currentTime - lastLEDUpdate > 200) {
                ledState = !ledState;
                digitalWrite(RED_LED, LOW);
                digitalWrite(YELLOW_LED, ledState);
                digitalWrite(GREEN_LED, LOW);
                digitalWrite(BLUE_LED, LOW);
                lastLEDUpdate = currentTime;
            }
            break;
            
        case NOWOOD:
            // Solid blue for NO_WOOD
            digitalWrite(RED_LED, LOW);
            digitalWrite(YELLOW_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(BLUE_LED, HIGH);
            break;
            
        case pushWoodForwardOne:
            // Alternating yellow/blue
            if (currentTime - lastLEDUpdate > 300) {
                ledState = !ledState;
                digitalWrite(RED_LED, LOW);
                digitalWrite(YELLOW_LED, ledState);
                digitalWrite(GREEN_LED, LOW);
                digitalWrite(BLUE_LED, !ledState);
                lastLEDUpdate = currentTime;
            }
            break;
            
        case RELOAD:
            // Solid yellow for reload mode
            digitalWrite(RED_LED, LOW);
            digitalWrite(YELLOW_LED, HIGH);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(BLUE_LED, LOW);
            break;
            
        case ERROR:
            // Blink red
            if (currentTime - lastLEDUpdate > 250) {
                ledState = !ledState;
                digitalWrite(RED_LED, ledState);
                digitalWrite(YELLOW_LED, LOW);
                digitalWrite(GREEN_LED, LOW);
                digitalWrite(BLUE_LED, LOW);
                lastLEDUpdate = currentTime;
            }
            break;
            
        case ERROR_RESET:
            // Brief yellow flash then back to idle
            digitalWrite(RED_LED, LOW);
            digitalWrite(YELLOW_LED, HIGH);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(BLUE_LED, LOW);
            break;
    }
} 