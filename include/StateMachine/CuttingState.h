#ifndef CUTTING_STATE_H
#define CUTTING_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Handles the wood cutting operation.
    // This state manages a multi-step cutting process including YES_WOOD and NO_WOOD sequences.

class CuttingState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return CUTTING; }

private:
    // Main cutting step tracking
    int cuttingStep = 0;
    unsigned long signalStartTime = 0;
    bool signalActive = false;
    bool homePositionErrorDetected = false;
    bool catcherClampActivatedThisCycle = false;
    float cutMotorIncrementalMoveTotalInches = 0.0;
    int cuttingSubStep7 = 0;
    
    // NO_WOOD sequence tracking
    int noWoodStep = 0;
    unsigned long cylinderActionTime = 0;
    bool waitingForCylinder = false;
    
    // Helper methods for different cutting phases
    void handleCuttingStep0(StateManager& stateManager);
    void handleCuttingStep1(StateManager& stateManager);
    void handleCuttingStep2(StateManager& stateManager);
    void handleCuttingStep3(StateManager& stateManager);
    void handleCuttingStep4(StateManager& stateManager);
    void handleCuttingStep5(StateManager& stateManager);
    void handleCuttingStep6_NO_WOOD_Sequence(StateManager& stateManager);
    void handleCuttingStep7_YES_WOOD_Sequence(StateManager& stateManager);
    
    // Helper methods for NO_WOOD sequence
    void handleNO_WOOD_Step(StateManager& stateManager, int step);
    
    // Helper method for home position error
    void handleHomePositionError(StateManager& stateManager);
    
    // Reset all step counters
    void resetSteps();
};

#endif // CUTTING_STATE_H 