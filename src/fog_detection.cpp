/**
 * @file fog_detection.cpp
 * @brief Freezing of Gait (FOG) detection system
 */

#include "fog_detection.h"
#include "signal_processing.h"  // For tremor_intensity and dysk_intensity
#include "config.h"
#include <cstdio>   // Required for printf
#include <cstdint>  // Required for uint32_t, uint16_t
#include <cstdbool> // Good practice for boolean types (or just built-in for C++)

// External variables from signal_processing.cpp
extern uint16_t tremor_intensity;
extern uint16_t dysk_intensity;

// FOG state machine
FOGDetector fog_detector = {FOG_NOT_WALKING, 0, 0, 0, 0.0f, 0, 0};

// Step detection variables
uint16_t steps_in_window = 0;
bool above_step_threshold = false;
uint32_t last_step_time_ms = 0;
float accel_baseline_ema = 1.0f;
uint8_t fog_status = 0;

void init_fog_detection()
{
    // Reset state machine to initial state
    fog_detector.state = FOG_NOT_WALKING;
    fog_detector.walking_start_time = 0;
    fog_detector.freeze_start_time = 0;
    fog_detector.freeze_confirmed_start = 0;
    fog_detector.previous_cadence = 0.0f;
    fog_detector.consecutive_walking_windows = 0;
    fog_detector.consecutive_freeze_windows = 0;

    // Reset step detection variables
    steps_in_window = 0;
    above_step_threshold = false;
    last_step_time_ms = 0;
    accel_baseline_ema = 1.0f;  // Start with baseline of 1g
    fog_status = 0;             // No FOG at startup
}

void process_fog_detection(float variance, uint32_t current_time)
{
    // Calculate cadence (steps/min)
    float window_duration_sec = (float)WINDOW_SIZE / TARGET_SAMPLE_RATE_HZ;
    float cadence = (steps_in_window / window_duration_sec) * 60.0f;

    // Detection thresholds
    const float WALKING_CADENCE_MIN = 10.0f;
    const float WALKING_CADENCE_MAX = 250.0f;
    const float WALKING_VARIANCE_MIN = 0.002f;
    const float WALKING_VARIANCE_MAX = 0.50f;
    const uint32_t MIN_STEPS_FOR_WALKING = 2;
    
    const float FREEZE_CADENCE_MAX = 12.0f;
    const float FREEZE_VARIANCE_MAX = 0.020f;
    
    const uint32_t MIN_WALKING_DURATION_MS = 1000;
    const uint32_t FREEZE_CONFIRMATION_MS = 1250;

    // Walking detection
    bool currently_walking = (steps_in_window >= MIN_STEPS_FOR_WALKING &&
                              cadence >= WALKING_CADENCE_MIN &&
                              cadence <= WALKING_CADENCE_MAX &&
                              variance >= WALKING_VARIANCE_MIN &&
                              variance <= WALKING_VARIANCE_MAX);

    // Freeze detection
    bool freeze_indicators = (cadence < FREEZE_CADENCE_MAX &&
                              variance < FREEZE_VARIANCE_MAX &&
                              fog_detector.walking_start_time > 0);
    
    // Time gating
    uint32_t time_since_last_step = (last_step_time_ms > 0) 
                                    ? (current_time - last_step_time_ms) 
                                    : 9999999;
    
    const uint32_t MAX_TIME_SINCE_STEP_MS = 15000;
    
    if (time_since_last_step > MAX_TIME_SINCE_STEP_MS) {
        freeze_indicators = false;
    }

    printf(" [S:%d C:%.0f V:%.3f T:%.1fs FI:%d CW:%d]", 
           steps_in_window, cadence, variance, 
           time_since_last_step/1000.0f, freeze_indicators, 
           currently_walking);

    // Safety check
    if ((fog_detector.state == FOG_POTENTIAL_FREEZE || fog_detector.state == FOG_FREEZE_CONFIRMED) &&
        fog_detector.walking_start_time == 0)
    {
        printf("   WARNING: Invalid state, resetting\n");
        fog_detector.state = FOG_NOT_WALKING;
        fog_detector.consecutive_walking_windows = 0;
        fog_detector.consecutive_freeze_windows = 0;
    }

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
                fog_detector.walking_start_time = current_time;  // Record when walking started
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
            fog_detector.consecutive_walking_windows++;
            fog_detector.consecutive_freeze_windows = 0;
        }
        else if (freeze_indicators)
        {
            fog_detector.consecutive_freeze_windows++;
            fog_detector.consecutive_walking_windows = 0;
            
            if (fog_detector.consecutive_freeze_windows >= 1 && 
                walking_duration >= MIN_WALKING_DURATION_MS)
            {
                fog_detector.state = FOG_POTENTIAL_FREEZE;
                fog_detector.freeze_start_time = current_time;
                fog_detector.consecutive_freeze_windows = 1;
            }
            else if (walking_duration < MIN_WALKING_DURATION_MS)
            {
                fog_detector.state = FOG_NOT_WALKING;
                fog_detector.consecutive_walking_windows = 0;
            }
            // Else: Still in WALKING state, accumulating freeze evidence
        }
        else
        {
            fog_detector.consecutive_freeze_windows++;
            fog_detector.consecutive_walking_windows = 0;
            
            if (fog_detector.consecutive_freeze_windows >= 1)
            {
                fog_detector.state = FOG_NOT_WALKING;
                fog_detector.consecutive_walking_windows = 0;
                fog_detector.walking_start_time = 0;  // Clear walking start time
            }
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
            fog_detector.state = FOG_NOT_WALKING;
            fog_detector.consecutive_walking_windows = 0;
            fog_detector.consecutive_freeze_windows = 0;
            fog_detector.walking_start_time = 0;
        }
        break;
    }

    case FOG_FREEZE_CONFIRMED:
    {
        if (fog_detector.freeze_confirmed_start == 0)
        {
            fog_detector.freeze_confirmed_start = current_time;
        }

        bool recovery_movement = (steps_in_window > 0 || variance > FREEZE_VARIANCE_MAX);
        
        if (recovery_movement)
        {
            fog_detector.state = FOG_WALKING;
            fog_detector.consecutive_freeze_windows = 0;
            fog_detector.consecutive_walking_windows = 1;
            fog_detector.walking_start_time = current_time;
            fog_detector.freeze_confirmed_start = 0;
            printf(" | Recovered");
        }
        else
        {
            printf(" | 🧊");
        }
        break;
    }
    }

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

    fog_detector.previous_cadence = cadence;
    steps_in_window = 0;
    fog_status = (fog_detector.state == FOG_FREEZE_CONFIRMED) ? 1 : 0;
}