#ifndef READY_STATE_H
#define READY_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************** READY STATE *********************************
//* ************************************************************************
// Handles the ready state, awaiting user input or automatic cycle start.

class ReadyState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    SystemState getStateType() const override { return READY; }

private:
    void handleReloadModeLogic(StateManager& stateManager);
    void checkStartConditions(StateManager& stateManager);
};

#endif // READY_STATE_H 