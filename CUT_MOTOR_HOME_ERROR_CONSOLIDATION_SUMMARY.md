# Cut Motor Home Error Detection Consolidation Summary

## Overview
Consolidated all cut motor home error detection and handling logic into a **unified error detection system** with highly descriptive names to address ongoing issues where the cut motor fails to fully reach home position.

## Critical Safety Importance
**Why this error detection is absolutely essential:**
- If the cut motor fails to return to its true home position and the position motor subsequently pushes a wood board forward, the board could collide with the cut motor blade or mechanism
- This could cause:
  1. Catastrophic damage to the cutting blade
  2. Destruction of the workpiece  
  3. Potential injury to operators
  4. Misalignment of the entire cutting system
  5. Unpredictable machine behavior in subsequent cycles

## Files Created

### `src/CutMotorHomeErrorHandler.cpp`
- Contains all consolidated cut motor home error detection and recovery logic
- Implements **unified recovery approach** with slow movement at homing speed
- Uses highly descriptive function names for clarity

### `include/CutMotorHomeErrorHandler.h`
- Header file with function declarations and data structures
- Defines `CutMotorHomeErrorResult` structure for consistent error reporting

## Unified Error Detection System

### **Single Recovery Approach for All Scenarios**
Instead of different responses for different contexts, the system now uses a **unified recovery method**:

1. **Initial Verification:** Standard 3-attempt home sensor check
2. **Unified Recovery:** If initial verification fails:
   - Move cut motor slowly backward at **homing speed** (1000 steps/sec)
   - **5-second timeout** for recovery attempt
   - **Real-time monitoring** of home sensor during movement
   - **Immediate stop** when home sensor detects position
   - **Automatic position recalibration** to 0 when found

### **Recovery Parameters:**
- **Speed:** 1000 steps/sec (same as homing speed)
- **Timeout:** 5 seconds maximum
- **Acceleration:** 10,000 steps/sec² for controlled movement
- **Direction:** Backward (negative) toward home position

## Error Detection Scenarios

### 1. Real-Time Detection During Yes-Wood Return
- **Function:** `performCutMotorRealTimeHomeSensorCheck()`
- **Trigger:** Cut motor hits home sensor while actively moving
- **Response:** Immediate stop and position recalibration
- **Severity:** Safest scenario (immediate detection)

### 2. Legacy Cutting Step 4
- **Function:** `handleCutMotorHomeErrorForLegacyCuttingStep()`
- **Response:** **Unified slow recovery** at homing speed with 5-second timeout
- **Severity:** Moderate (recovery attempt before error)

### 3. Yes-Wood Simultaneous Return
- **Function:** `handleCutMotorHomeErrorForYesWoodSimultaneousReturn()`
- **Response:** **Unified slow recovery** at homing speed with 5-second timeout
- **Severity:** Critical (precise positioning essential)

### 4. No-Wood Sequence Completion
- **Function:** `handleCutMotorHomeErrorForNoWoodSequenceCompletion()`
- **Response:** **Warning only** (slow recovery disabled since no workpiece present)
- **Severity:** Least severe (no workpiece to damage)

### 5. Fix Position Operation
- **Function:** `handleCutMotorHomeErrorForFixPositionOperation()`
- **Response:** **Unified slow recovery** at homing speed with 5-second timeout
- **Severity:** Critical (high precision required)

## Error Recovery System

### Result Structure
```cpp
struct CutMotorHomeErrorResult {
    bool wasHomeDetected;                    // True if home position successfully detected
    bool shouldTransitionToError;            // True if system should enter ERROR state
    bool shouldAttemptSlowRecovery;          // True if slow recovery at homing speed should be attempted
    bool shouldContinueWithWarning;          // True if operation should continue with warning only
    String errorMessage;                     // Descriptive message about the error or success
};
```

### Unified Recovery Function
```cpp
CutMotorHomeErrorResult handleCutMotorHomeErrorWithUnifiedRecovery(
    Bounce& cutHomingSwitch, 
    FastAccelStepper* cutMotor, 
    const String& contextDescription,
    bool allowSlowRecovery  // Controls whether slow recovery is attempted
);
```

### Error State Transition
- **Function:** `executeCutMotorErrorStateTransition()`
- Stops all motors immediately
- Engages safety clamps
- Sets error indication LEDs (Red ON, Yellow OFF)
- Transitions to ERROR state
- Resets all state machine steps
- Requires user acknowledgment via reload switch

## Changes Made to main.cpp

### Updated Function Calls
- **Cutting Step 4** → `handleCutMotorHomeErrorForLegacyCuttingStep()`
- **Yes-wood simultaneous return** → `handleCutMotorHomeErrorForYesWoodSimultaneousReturn()`
- **No-wood sequence completion** → `handleCutMotorHomeErrorForNoWoodSequenceCompletion()`
- **FIX_POSITION error checking** → `handleCutMotorHomeErrorForFixPositionOperation()`

### Removed Old Incremental System
- Removed all incremental step logic (0.1" steps, 0.4" max)
- Replaced with unified slow recovery at homing speed
- Simplified logic with single recovery approach

## Benefits of Unified System

1. **Consistent Behavior:** All scenarios use the same recovery method
2. **Faster Recovery:** Continuous movement instead of discrete steps
3. **Real-Time Detection:** Immediate response when home is found
4. **Timeout Protection:** 5-second limit prevents infinite recovery attempts
5. **Safer Movement:** Uses proven homing speed and acceleration
6. **Simpler Logic:** Single recovery function instead of multiple approaches
7. **Better Debugging:** Consistent logging and error messages

## Usage Example
```cpp
CutMotorHomeErrorResult result = handleCutMotorHomeErrorForYesWoodSimultaneousReturn(cutHomingSwitch, cutMotor);
logCutMotorHomeErrorResult(result);

if (result.wasHomeDetected) {
    // Continue normal operation - home found either initially or during recovery
} else if (result.shouldTransitionToError) {
    // Recovery failed or timed out - transition to error state
    executeCutMotorErrorStateTransition(/* parameters */);
} else if (result.shouldContinueWithWarning) {
    // Warning only mode (no-wood sequences)
}
```

## Key Improvements

### **Before (Multiple Approaches):**
- Incremental steps (0.1" at a time)
- Different recovery strategies per scenario
- Complex state tracking for recovery attempts
- Inconsistent timeout handling

### **After (Unified Approach):**
- **Single recovery method** for all scenarios
- **Continuous movement** at homing speed until home found
- **5-second timeout** for all recovery attempts
- **Real-time detection** during recovery movement
- **Immediate position recalibration** when successful

This unified system provides more reliable and faster cut motor home detection recovery while maintaining all safety features and simplifying the codebase for easier debugging and maintenance. 