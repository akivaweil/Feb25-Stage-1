#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************** IDLE STATE **********************************
//* ************************************************************************
// Handles the idle state, awaiting user input or automatic cycle start.

class IdleState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    SystemState getStateType() const override { return IDLE; }

private:
    void handleReloadModeLogic(StateManager& stateManager);
    void checkStartConditions(StateManager& stateManager);
};

#endif // IDLE_STATE_H 