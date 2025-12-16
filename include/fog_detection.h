/**
 * @file fog_detection.h
 * @brief Freezing of Gait (FOG) detection system - Header file

 * 
 * @description
 * Implements a 4-state machine for detecting Freezing of Gait episodes in
 * Parkinson's Disease patients. FOG is characterized by sudden inability to
 * initiate or continue walking despite the intention to do so.
 * 
 * State machine: NOT_WALKING → WALKING → POTENTIAL_FREEZE → FREEZE_CONFIRMED
 * 
 * Detection criteria:
 * - FOG can only occur AFTER normal walking is established
 * - Requires very low cadence (<10 steps/min) AND low variance (<0.015)
 * - Multi-window confirmation reduces false positives
 * - Automatic timeout and recovery detection
 */

#ifndef FOG_DETECTION_H
#define FOG_DETECTION_H

#include "mbed.h"
#include "config.h"

// =============================================================================
// FOG State Machine Definitions
// =============================================================================

/**
 * @brief FOG detection states
 * 
 * State transitions:
 * NOT_WALKING: Initial state, no movement detected
 *    → WALKING: When consistent walking pattern detected
 * 
 * WALKING: Normal walking confirmed and sustained
 *    → POTENTIAL_FREEZE: When freeze indicators appear after ≥2s of walking
 *    → NOT_WALKING: When walking stops without freeze indicators
 * 
 * POTENTIAL_FREEZE: Freeze indicators detected, awaiting confirmation
 *    → FREEZE_CONFIRMED: After ≥1.5s of sustained freeze indicators
 *    → WALKING: If patient resumes walking (false alarm)
 *    → NOT_WALKING: If indicators disappear without walking
 * 
 * FREEZE_CONFIRMED: FOG episode confirmed and active
 *    → WALKING: When patient recovers and resumes walking
 *    → NOT_WALKING: After 15-second timeout (patient stopped/needs assistance)
 */
enum FOGState {
    FOG_NOT_WALKING,       // No movement or insufficient activity
    FOG_WALKING,           // Normal walking detected
    FOG_POTENTIAL_FREEZE,  // Freeze indicators present, not yet confirmed
    FOG_FREEZE_CONFIRMED   // FOG episode confirmed (alarm state)
};

/**
 * @brief FOG detector state machine data structure
 */
struct FOGDetector {
    FOGState state;                          // Current state in the state machine
    uint32_t walking_start_time;             // Timestamp when walking began (ms)
    uint32_t freeze_start_time;              // Timestamp when freeze indicators first appeared (ms)
    uint32_t freeze_confirmed_start;         // Timestamp when freeze was confirmed (ms)
    float previous_cadence;                  // Cadence from previous window (steps/min)
    uint8_t consecutive_walking_windows;     // Count of consecutive walking windows
    uint8_t consecutive_freeze_windows;      // Count of consecutive freeze windows
};

// =============================================================================
// Global Variables (Exported)
// =============================================================================

// State machine instance
extern FOGDetector fog_detector;

// Step detection variables (shared with signal_processing.cpp)
extern uint16_t steps_in_window;        // Steps detected in current 3-second window
extern bool above_step_threshold;       // Current state of step detection threshold
extern uint32_t last_step_time_ms;      // Timestamp of most recent detected step (ms)
extern float accel_baseline_ema;        // Adaptive baseline for step threshold (g)
extern uint16_t fog_status;             // BLE characteristic: 0=Normal, 1=FOG detected

// =============================================================================
// Function Declarations
// =============================================================================

/**
 * @brief Initialize FOG detection system
 * 
 * Resets all state machine variables to initial values. Must be called once
 * at system startup before entering the main detection loop.
 * 
 * Initializes:
 * - State machine to FOG_NOT_WALKING
 * - All timestamps to 0
 * - Step counter to 0
 * - BLE FOG status to 0 (no FOG)
 */
void init_fog_detection();

/**
 * @brief Process FOG detection for the current window
 * 
 * Analyzes current window data and updates the FOG state machine.
 * Should be called once per window (every ~3 seconds) after FFT analysis.
 * 
 * @param variance Movement variance from accelerometer Z-axis (0.0-1.0 typical range)
 *                 Calculated as standard deviation of accel samples in window
 * @param current_time Current system timestamp in milliseconds (from Kernel::get_ms_count())
 * 
 * Updates:
 * - fog_detector.state (state machine progression)
 * - fog_status (BLE characteristic: 0 or 1)
 * - steps_in_window (reset to 0 for next window)
 * 
 * Prints status to serial console for debugging and monitoring.
 */
void process_fog_detection(float variance, uint32_t current_time);

#endif // FOG_DETECTION_H