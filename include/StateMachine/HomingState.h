#ifndef HOMING_STATE_H
#define HOMING_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************** HOMING STATE ********************************
//* ************************************************************************
// Handles the homing sequence for all motors.

class HomingState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    SystemState getStateType() const override { return HOMING; }

private:
    // Static variables for homing sequence tracking
    bool cutMotorHomed = false;
    bool positionMotorHomed = false;
    bool positionMotorMoved = false;
    bool positionHomingPhaseInitiated = false;
    unsigned long blinkTimer = 0;
};

#endif // HOMING_STATE_H 