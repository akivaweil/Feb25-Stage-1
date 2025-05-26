#include "ErrorStates/suction_error_hold.h"
#include <Bounce2.h>

// External references to global variables and functions from main.cpp
extern SystemState currentState;
extern bool continuousModeActive;
extern bool startSwitchSafe;
extern Bounce startCycleSwitch;
extern void turnRedLedOn();
extern void turnRedLedOff();
extern void turnYellowLedOff();
extern void turnGreenLedOff();
extern void turnBlueLedOff();

//* ************************************************************************
//* ********************* SUCTION ERROR HOLD *******************************
//* ************************************************************************
// Handles waiting for user to reset a wood suction error via cycle switch.
// This state is entered from CUTTING (Step 1) if the WAS_WOOD_SUCTIONED_SENSOR indicates an error (LOW).
// Step 1: Slowly blink the red LED (e.g., every 1.5 seconds).
// Step 2: Ensure yellow, green, and blue LEDs are off.
// Step 3: Monitor the start cycle switch.
// Step 4: If the start cycle switch shows a rising edge (OFF to ON transition):
//          - Print a message about resetting from suction error.
//          - Turn off the red LED.
//          - Set continuousModeActive to false.
//          - Set startSwitchSafe to false (requires user to cycle switch again for a new start).
//          - Transition to HOMING state to re-initialize the system.
void handleSuctionErrorHoldState() {
    static unsigned long lastSuctionErrorBlinkTime = 0;
    static bool suctionErrorBlinkState = false;

    // Blink RED_LED every 1.5 seconds
    if (millis() - lastSuctionErrorBlinkTime >= 1500) {
        lastSuctionErrorBlinkTime = millis();
        suctionErrorBlinkState = !suctionErrorBlinkState;
        if(suctionErrorBlinkState) turnRedLedOn(); else turnRedLedOff();
    }
    
    // Ensure other LEDs are off
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();

    if (startCycleSwitch.rose()) { // Check for start switch OFF to ON transition
        Serial.println("Start cycle switch toggled ON. Resetting from suction error. Transitioning to HOMING.");
        turnRedLedOff();   // Turn off error LED explicitly before changing state
        
        continuousModeActive = false; // Ensure continuous mode is off
        startSwitchSafe = false;      // Require user to cycle switch OFF then ON for a new actual start
        
        currentState = HOMING;        // Go to HOMING to re-initialize
    }
} 