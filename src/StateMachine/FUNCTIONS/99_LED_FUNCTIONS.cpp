#include "StateMachine/StateMachine.h"

//* ************************************************************************
//* ************************ LED FUNCTIONS ******************************
//* ************************************************************************
//! LED control functions for all system status indicators
//! Basic LED operations and specialized blinking patterns

//* ************************************************************************
//* ************************ BASIC LED CONTROLS **************************
//* ************************************************************************

void turnRedLedOn() {
    digitalWrite(RED_LED, HIGH);
    Serial.println("Red LED ON");
}

void turnRedLedOff() {
    digitalWrite(RED_LED, LOW);
    Serial.println("Red LED OFF");
}

void turnYellowLedOn() {
    digitalWrite(YELLOW_LED, HIGH);;
    Serial.println("Yellow LED ON");
}

void turnYellowLedOff() {
    digitalWrite(YELLOW_LED, LOW);
    Serial.println("Yellow LED OFF");
}

void turnGreenLedOn() {
    digitalWrite(GREEN_LED, HIGH);
    Serial.println("Green LED ON");
}

void turnGreenLedOff() {
    digitalWrite(GREEN_LED, LOW);
    Serial.println("Green LED OFF");
}

void turnBlueLedOn() {
    digitalWrite(BLUE_LED, HIGH);
    Serial.println("Blue LED ON");
}

void turnBlueLedOff() {
    digitalWrite(BLUE_LED, LOW);
    Serial.println("Blue LED OFF");
}

void allLedsOff() {
    turnRedLedOff();
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();
}

//* ************************************************************************
//* ************************ LED BLINKING PATTERNS ***********************
//* ************************************************************************

void handleHomingLedBlink() {
    static unsigned long blinkTimer = 0;
    static bool blinkState = false;
    if (millis() - blinkTimer > 500) {
        blinkState = !blinkState;
        if (blinkState) turnBlueLedOn(); else turnBlueLedOff();
        blinkTimer = millis();
    }
}

void handleErrorLedBlink() {
    static unsigned long lastErrorBlinkTime = 0;
    static bool errorBlinkState = false;
    if (millis() - lastErrorBlinkTime > 250) {
        errorBlinkState = !errorBlinkState;
        if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        lastErrorBlinkTime = millis();
    }
} 