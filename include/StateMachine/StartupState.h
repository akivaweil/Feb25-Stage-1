#ifndef STARTUP_STATE_H
#define STARTUP_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************** STARTUP STATE *******************************
//* ************************************************************************
// Handles the initial startup state, transitioning to HOMING.

class StartupState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    SystemState getStateType() const override { return STARTUP; }
};

#endif // STARTUP_STATE_H 