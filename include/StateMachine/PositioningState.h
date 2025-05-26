#ifndef POSITIONING_STATE_H
#define POSITIONING_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************ POSITIONING STATE *****************************
//* ************************************************************************
// Handles the motor positioning sequence (largely integrated into CUTTING).
// This state is now mostly a placeholder as the positioning logic is integrated directly
// within the CUTTING state's stages.

class PositioningState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    SystemState getStateType() const override { return POSITIONING; }
};

#endif // POSITIONING_STATE_H 