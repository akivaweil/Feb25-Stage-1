#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>
#include "Functions.h"

//* ************************************************************************
//* ************************* STATE MANAGER *******************************
//* ************************************************************************
// Central state manager that coordinates all state operations
// and provides access to system resources.

class StateManager {
public:
    // Constructor
    StateManager();
    
    // Main state machine execution
    void execute();
    
    // State transition management
    void changeState(SystemState newState);
    SystemState getCurrentState() const { return currentState; }
    SystemState getPreviousState() const { return previousState; }
    
    // System resource access methods
    FastAccelStepper* getCutMotor() { return cutMotor; }
    FastAccelStepper* getPositionMotor() { return positionMotor; }
    Servo* getCatcherServo() { return &catcherServo; }
    
    // Switch access methods
    Bounce* getCutHomingSwitch() { return &cutHomingSwitch; }
    Bounce* getPositionHomingSwitch() { return &positionHomingSwitch; }
    Bounce* getReloadSwitch() { return &reloadSwitch; }
    Bounce* getStartCycleSwitch() { return &startCycleSwitch; }
    Bounce* getFixPositionButton() { return &fixPositionButton; }
    
    // System flag access methods
    bool getIsReloadMode() const { return isReloadMode; }
    void setIsReloadMode(bool value) { isReloadMode = value; }
    
    bool getWoodPresent() const { return woodPresent; }
    void setWoodPresent(bool value) { woodPresent = value; }
    
    bool getWoodSuctionError() const { return woodSuctionError; }
    void setWoodSuctionError(bool value) { woodSuctionError = value; }
    
    bool getWasWoodCaughtError() const { return wasWoodCaughtError; }
    void setWasWoodCaughtError(bool value) { wasWoodCaughtError = value; }
    
    bool getErrorAcknowledged() const { return errorAcknowledged; }
    void setErrorAcknowledged(bool value) { errorAcknowledged = value; }
    
    bool getCuttingCycleInProgress() const { return cuttingCycleInProgress; }
    void setCuttingCycleInProgress(bool value) { cuttingCycleInProgress = value; }
    
    bool getContinuousModeActive() const { return continuousModeActive; }
    void setContinuousModeActive(bool value) { continuousModeActive = value; }
    
    bool getStartSwitchSafe() const { return startSwitchSafe; }
    void setStartSwitchSafe(bool value) { startSwitchSafe = value; }
    
    bool getWoodCaughtCheckPending() const { return woodCaughtCheckPending; }
    void setWoodCaughtCheckPending(bool value) { woodCaughtCheckPending = value; }
    
    unsigned long getWoodCaughtCheckTime() const { return woodCaughtCheckTime; }
    void setWoodCaughtCheckTime(unsigned long value) { woodCaughtCheckTime = value; }
    
    // Timer access methods
    unsigned long getLastBlinkTime() const { return lastBlinkTime; }
    void setLastBlinkTime(unsigned long value) { lastBlinkTime = value; }
    
    unsigned long getLastErrorBlinkTime() const { return lastErrorBlinkTime; }
    void setLastErrorBlinkTime(unsigned long value) { lastErrorBlinkTime = value; }
    
    unsigned long getErrorStartTime() const { return errorStartTime; }
    void setErrorStartTime(unsigned long value) { errorStartTime = value; }
    
    // LED state access methods
    bool getBlinkState() const { return blinkState; }
    void setBlinkState(bool value) { blinkState = value; }
    
    bool getErrorBlinkState() const { return errorBlinkState; }
    void setErrorBlinkState(bool value) { errorBlinkState = value; }
    
    // Catcher servo timing access
    unsigned long getCatcherServoActiveStartTime() const { return catcherServoActiveStartTime; }
    void setCatcherServoActiveStartTime(unsigned long value) { catcherServoActiveStartTime = value; }
    
    bool getCatcherServoIsActiveAndTiming() const { return catcherServoIsActiveAndTiming; }
    void setCatcherServoIsActiveAndTiming(bool value) { catcherServoIsActiveAndTiming = value; }
    
    unsigned long getCatcherClampEngageTime() const { return catcherClampEngageTime; }
    void setCatcherClampEngageTime(unsigned long value) { catcherClampEngageTime = value; }
    
    bool getCatcherClampIsEngaged() const { return catcherClampIsEngaged; }
    void setCatcherClampIsEngaged(bool value) { catcherClampIsEngaged = value; }
    
    // Signal timing access
    unsigned long getSignalTAStartTime() const { return signalTAStartTime; }
    void setSignalTAStartTime(unsigned long value) { signalTAStartTime = value; }
    
    bool getSignalTAActive() const { return signalTAActive; }
    void setSignalTAActive(bool value) { signalTAActive = value; }

private:
    SystemState previousState;
    
    // Print state changes
    void printStateChange();
    
    // Update all switches - moved from main loop
    void updateSwitches();
    
    // Handle common operations that happen every loop
    void handleCommonOperations();
};

// Global state manager instance
extern StateManager stateManager;

#endif // STATE_MANAGER_H 