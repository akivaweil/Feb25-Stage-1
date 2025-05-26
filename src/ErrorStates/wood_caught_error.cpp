#include "ErrorStates/wood_caught_error.h"
#include <Bounce2.h>

// External references to global variables and functions from main.cpp
extern SystemState currentState;
extern bool wasWoodCaughtError;
extern bool continuousModeActive;
extern bool startSwitchSafe;
extern Bounce startCycleSwitch;
extern void turnRedLedOff();
extern void stopPositionMotor();
extern void handleWoodCaughtErrorLedBlink(unsigned long& lastBlinkTimeRef, bool& blinkStateRef);

//* ************************************************************************
//* ********************* WAS_WOOD_CAUGHT_ERROR ***************************
//* ************************************************************************
// Handles the case where the wood was not properly caught by the catcher.
// Step 1: Blink red LED at a moderate pace (once per second).
// Step 2: Ensure all other LEDs are off.
// Step 3: Ensure position motor is stopped (cut motor continues to home).
// Step 4: Wait for cycle switch to be pressed (rising edge) to acknowledge the error.
// Step 5: When cycle switch is pressed, transition to HOMING state.
void handleWoodCaughtErrorState() {
    static unsigned long lastWoodCaughtErrorBlinkTime = 0;
    static bool woodCaughtErrorBlinkState = false;

    // Blink RED_LED once per second
    handleWoodCaughtErrorLedBlink(lastWoodCaughtErrorBlinkTime, woodCaughtErrorBlinkState);

    // Keep position motor stopped (cut motor continues to home)
    stopPositionMotor();
    
    // Check for cycle switch rising edge (OFF to ON transition)
    if (startCycleSwitch.rose()) { 
        Serial.println("Start cycle switch toggled ON. Resetting from wood caught error. Transitioning to HOMING.");
        turnRedLedOff();   // Turn off error LED explicitly before changing state
        
        // Clear error flags
        wasWoodCaughtError = false;
        
        // Reset mode flags
        continuousModeActive = false; // Ensure continuous mode is off
        startSwitchSafe = false;      // Require user to cycle switch OFF then ON for a new actual start
        
        currentState = HOMING;        // Go to HOMING to re-initialize
    }
    
    Serial.println("Waiting for cycle switch to acknowledge wood caught error.");
} 