#ifndef RETURNING_STATE_H
#define RETURNING_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************* RETURNING STATE ******************************
//* ************************************************************************
// Handles the motor return sequence after a cut (largely integrated into CUTTING).
// This state is now mostly a placeholder as the return logic is integrated directly
// within the CUTTING state's stages.

class ReturningState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    SystemState getStateType() const override { return RETURNING; }
};

#endif // RETURNING_STATE_H 