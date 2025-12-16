/**
 * @file fog_detection.h
 * @brief Freezing of Gait (FOG) detection system
 * @author RTES Challenge Implementation
 * @date December 2025
 */

#ifndef FOG_DETECTION_H
#define FOG_DETECTION_H

#include "mbed.h"
#include "config.h"

// =============================================================================
// FOG State Machine
// =============================================================================

enum FOGState {
    FOG_NOT_WALKING,
    FOG_WALKING,
    FOG_POTENTIAL_FREEZE,
    FOG_FREEZE_CONFIRMED
};

struct FOGDetector {
    FOGState state;
    uint32_t walking_start_time;
    uint32_t freeze_start_time;
    float previous_cadence;
    uint8_t consecutive_walking_windows;
    uint8_t consecutive_freeze_windows;
};

// =============================================================================
// Step Detection Variables
// =============================================================================

extern FOGDetector fog_detector;
extern uint16_t steps_in_window;
extern bool above_step_threshold;
extern uint32_t last_step_time_ms;
extern float accel_baseline_ema;
extern uint16_t fog_status;

// =============================================================================
// Function Declarations
// =============================================================================

/**
 * @brief Initialize FOG detection system
 */
void init_fog_detection();

/**
 * @brief Process FOG detection for current window
 * @param variance Movement variance from accelerometer data
 * @param current_time Current timestamp in milliseconds
 */
void process_fog_detection(float variance, uint32_t current_time);

#endif // FOG_DETECTION_H