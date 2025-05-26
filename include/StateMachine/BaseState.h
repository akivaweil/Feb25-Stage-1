#ifndef BASE_STATE_H
#define BASE_STATE_H

#include "Functions.h"

//* ************************************************************************
//* ************************** BASE STATE **********************************
//* ************************************************************************
// Base interface for all state machine states.
// Each state should inherit from this and implement the execute method.

class StateManager; // Forward declaration

class BaseState {
public:
    BaseState() = default;
    virtual ~BaseState() = default;
    
    // Pure virtual method that each state must implement
    virtual void execute(StateManager& stateManager) = 0;
    
    // Optional method for state entry actions
    virtual void onEnter(StateManager& stateManager) {}
    
    // Optional method for state exit actions
    virtual void onExit(StateManager& stateManager) {}
    
    // Get the state type
    virtual SystemState getStateType() const = 0;
};

#endif // BASE_STATE_H 