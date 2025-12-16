/**
 * @file fog_detection.cpp
 * @brief Freezing of Gait (FOG) detection system implementation
 * @author RTES Challenge Implementation
 * @date December 2025
 */

#include "fog_detection.h"
#include "signal_processing.h"  // For tremor_intensity and dysk_intensity
#include "config.h"
#include <cstdio>   // Required for printf
#include <cstdint>  // Required for uint32_t, uint16_t
#include <cstdbool> // Good practice for boolean types (or just built-in for C++)

// =============================================================================
// External Variables from Signal Processing
// =============================================================================
extern uint16_t tremor_intensity;
extern uint16_t dysk_intensity;

// =============================================================================
// FOG Detection State
// =============================================================================

// Ensure FOGDetector struct is defined in fog_detection.h
FOGDetector fog_detector = {FOG_NOT_WALKING, 0, 0, 0.0f, 0, 0};

uint16_t steps_in_window = 0;
bool above_step_threshold = false;
uint32_t last_step_time_ms = 0;
float accel_baseline_ema = 1.0f;
uint16_t fog_status = 0;

// =============================================================================
// FOG Detection Functions
// =============================================================================

void init_fog_detection()
{
    fog_detector.state = FOG_NOT_WALKING;
    fog_detector.walking_start_time = 0;
    fog_detector.freeze_start_time = 0;
    fog_detector.previous_cadence = 0.0f;
    fog_detector.consecutive_walking_windows = 0;
    fog_detector.consecutive_freeze_windows = 0;

    steps_in_window = 0;
    above_step_threshold = false;
    last_step_time_ms = 0;
    accel_baseline_ema = 1.0f;
    fog_status = 0;
}

void process_fog_detection(float variance, uint32_t current_time)
{
    // Calculate cadence (steps per minute)
    // NOTE: Ensure WINDOW_SIZE and TARGET_SAMPLE_RATE_HZ are defined in config.h
    float window_duration_sec = (float)WINDOW_SIZE / TARGET_SAMPLE_RATE_HZ;
    float cadence = (steps_in_window / window_duration_sec) * 60.0f; // steps/min

    // Define thresholds - MUCH STRICTER to avoid false walking detection
    const float WALKING_CADENCE_MIN = 40.0f;     // Increased from 20 - require actual walking pace
    const float WALKING_CADENCE_MAX = 140.0f;    // Decreased from 180 - realistic walking only
    const float FREEZE_CADENCE_MAX = 10.0f;
    const float WALKING_VARIANCE_MIN = 0.05f;    // Increased from 0.02 - need clear movement
    const float WALKING_VARIANCE_MAX = 0.30f;    // Add upper limit - tremor can be high variance
    const float FREEZE_VARIANCE_MAX = 0.015f;
    const uint32_t MIN_WALKING_DURATION_MS = 2000;
    const uint32_t FREEZE_CONFIRMATION_MS = 1500;
    const uint32_t MIN_STEPS_FOR_WALKING = 2;    // Require at least 2 steps in window

    // STRICT walking detection: require ALL conditions
    // 1. Actual steps detected (not zero)
    // 2. Steps in realistic walking cadence range
    // 3. Variance in walking range (not too low, not too high like tremor)
    // 4. NO tremor or dyskinesia currently active (they trigger false steps)
    bool currently_walking = (steps_in_window >= MIN_STEPS_FOR_WALKING &&
                              cadence >= WALKING_CADENCE_MIN &&
                              cadence <= WALKING_CADENCE_MAX &&
                              variance >= WALKING_VARIANCE_MIN &&
                              variance <= WALKING_VARIANCE_MAX &&
                              tremor_intensity == 0 &&
                              dysk_intensity == 0);

    // Freeze indicators: require BOTH very low activity AND recent walking context
    bool freeze_indicators = (cadence < FREEZE_CADENCE_MAX &&
                              variance < FREEZE_VARIANCE_MAX &&
                              fog_detector.walking_start_time > 0);

    // Print FOG status based on actual state machine
    printf(" | FOG: ");
    switch (fog_detector.state)
    {
    case FOG_NOT_WALKING:
        printf("NotWalking");
        break;
    case FOG_WALKING:
        printf("Walk");
        break;
    case FOG_POTENTIAL_FREEZE:
        printf("Freeze?");
        break;
    case FOG_FREEZE_CONFIRMED:
        printf("FOG!");
        break;
    }

    // Safety check: Can't be in freeze states if never walked
    if ((fog_detector.state == FOG_POTENTIAL_FREEZE || fog_detector.state == FOG_FREEZE_CONFIRMED) &&
        fog_detector.walking_start_time == 0)
    {
        printf("   ⚠️  WARNING: Invalid FOG state detected, resetting\n");
        fog_detector.state = FOG_NOT_WALKING;
        fog_detector.consecutive_walking_windows = 0;
        fog_detector.consecutive_freeze_windows = 0;
    }

    // State machine
    switch (fog_detector.state)
    {
    case FOG_NOT_WALKING:
    {
        if (currently_walking)
        {
            fog_detector.consecutive_walking_windows++;

            if (fog_detector.consecutive_walking_windows >= 1)
            {
                fog_detector.state = FOG_WALKING;
                fog_detector.walking_start_time = current_time;
                fog_detector.consecutive_freeze_windows = 0;
            }
        }
        else
        {
            fog_detector.consecutive_walking_windows = 0;
        }
        break;
    }

    case FOG_WALKING:
    {
        uint32_t walking_duration = current_time - fog_detector.walking_start_time;

        if (currently_walking)
        {
            // Continue walking
        }
        else if (freeze_indicators)
        {
            if (walking_duration >= MIN_WALKING_DURATION_MS)
            {
                fog_detector.state = FOG_POTENTIAL_FREEZE;
                fog_detector.freeze_start_time = current_time;
                fog_detector.consecutive_freeze_windows = 1;
            }
            else
            {
                fog_detector.state = FOG_NOT_WALKING;
                fog_detector.consecutive_walking_windows = 0;
            }
        }
        else
        {
            fog_detector.state = FOG_NOT_WALKING;
            fog_detector.consecutive_walking_windows = 0;
        }
        break;
    }

    case FOG_POTENTIAL_FREEZE:
    {
        uint32_t freeze_duration = current_time - fog_detector.freeze_start_time;

        if (currently_walking)
        {
            fog_detector.state = FOG_WALKING;
            fog_detector.consecutive_freeze_windows = 0;
        }
        else if (freeze_indicators)
        {
            fog_detector.consecutive_freeze_windows++;

            if (freeze_duration >= FREEZE_CONFIRMATION_MS)
            {
                fog_detector.state = FOG_FREEZE_CONFIRMED;
            }
        }
        else
        {
            // Not walking and no freeze indicators - reset
            fog_detector.state = FOG_NOT_WALKING;
            fog_detector.consecutive_walking_windows = 0;
            fog_detector.consecutive_freeze_windows = 0;
            fog_detector.walking_start_time = 0;
        }
        break;
    }

    case FOG_FREEZE_CONFIRMED:
    {
        // Note: Scoped block {} is required here because we are declaring variables
        static uint32_t freeze_confirmed_start = 0;

        if (freeze_confirmed_start == 0)
        {
            freeze_confirmed_start = current_time;
        }

        uint32_t confirmed_duration = current_time - freeze_confirmed_start;
        const uint32_t MAX_FOG_DURATION_MS = 15000;

        if (currently_walking)
        {
            // Walking resumes - clear FOG
            fog_detector.state = FOG_WALKING;
            fog_detector.consecutive_freeze_windows = 0;
            freeze_confirmed_start = 0;
            printf(" | Recovered");
        }
        else if (confirmed_duration >= MAX_FOG_DURATION_MS)
        {
            // Timeout - reset to initial state
            fog_detector.state = FOG_NOT_WALKING;
            fog_detector.consecutive_freeze_windows = 0;
            fog_detector.consecutive_walking_windows = 0;
            fog_detector.walking_start_time = 0;
            freeze_confirmed_start = 0;
            printf(" | Timeout-Reset");
        }
        break;
    }
    }

    fog_detector.previous_cadence = cadence;

    // Reset step counter for next window
    steps_in_window = 0;

    // Update FOG status for BLE
    fog_status = (fog_detector.state == FOG_FREEZE_CONFIRMED) ? 1 : 0;
}