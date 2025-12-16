/**
 * @file fog_detection.h
 * @brief Freezing of Gait (FOG) detection system
 */

#ifndef FOG_DETECTION_H
#define FOG_DETECTION_H

#include "mbed.h"
#include "config.h"

// FOG state machine states
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
    uint32_t freeze_confirmed_start;
    float previous_cadence;
    uint8_t consecutive_walking_windows;
    uint8_t consecutive_freeze_windows;
};

extern FOGDetector fog_detector;
extern uint16_t steps_in_window;
extern bool above_step_threshold;
extern uint32_t last_step_time_ms;
extern float accel_baseline_ema;
extern uint8_t fog_status;

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