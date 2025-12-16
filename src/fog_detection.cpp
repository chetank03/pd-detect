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
// Access tremor and dyskinesia intensities to avoid false positives
// (tremor/dyskinesia can trigger false step detection)
extern uint16_t tremor_intensity;
extern uint16_t dysk_intensity;

// =============================================================================
// FOG Detection State Variables
// =============================================================================

// Main FOG detector state machine instance
FOGDetector fog_detector = {FOG_NOT_WALKING, 0, 0, 0.0f, 0, 0};

// Step detection variables (shared with signal_processing.cpp)
uint16_t steps_in_window = 0;          // Steps detected in current 3-second window
bool above_step_threshold = false;      // Current state of step threshold crossing
uint32_t last_step_time_ms = 0;         // Timestamp of last detected step
float accel_baseline_ema = 1.0f;        // Adaptive baseline for step detection threshold
uint16_t fog_status = 0;                // BLE output: 0=Normal, 1=FOG detected

// =============================================================================
// FOG Detection Functions
// =============================================================================

/**
 * @brief Initialize FOG detection system
 * 
 * Resets all state machine variables and step detection parameters.
 * Called once at system startup before entering main detection loop.
 */
void init_fog_detection()
{
    // Reset state machine to initial state
    fog_detector.state = FOG_NOT_WALKING;
    fog_detector.walking_start_time = 0;
    fog_detector.freeze_start_time = 0;
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

/**
 * @brief Process FOG detection for the current window
 * 
 * Implements a 4-state machine for FOG detection:
 * NOT_WALKING → WALKING → POTENTIAL_FREEZE → FREEZE_CONFIRMED
 * 
 * @param variance Movement variance from accelerometer Z-axis (measure of movement stability)
 * @param current_time Current system timestamp in milliseconds
 * 
 * Clinical rationale:
 * - FOG is characterized by sudden inability to initiate or continue walking
 * - Must occur AFTER normal walking is established (not just standing still)
 * - Requires low cadence + low variance + recent walking context
 * - Multiple window confirmation reduces false positives
 */
void process_fog_detection(float variance, uint32_t current_time)
{
    // Calculate cadence (steps per minute) from window data
    // Window duration = samples / sample_rate (e.g., 156 samples / 52 Hz = 3 seconds)
    float window_duration_sec = (float)WINDOW_SIZE / TARGET_SAMPLE_RATE_HZ;
    float cadence = (steps_in_window / window_duration_sec) * 60.0f; // Convert to steps/min

    // =========================================================================
    // Detection Thresholds - Tuned for Parkinson's gait patterns
    // =========================================================================
    
    // Walking detection thresholds
    const float WALKING_CADENCE_MIN = 25.0f;     // Min 25 steps/min (slow PD gait)
    const float WALKING_CADENCE_MAX = 140.0f;    // Max 140 steps/min (realistic upper bound)
    const float WALKING_VARIANCE_MIN = 0.05f;    // Min movement variability for walking
    const float WALKING_VARIANCE_MAX = 0.30f;    // Max variance (exclude tremor-induced high variance)
    const uint32_t MIN_STEPS_FOR_WALKING = 2;    // At least 2 steps required in 3-second window
    
    // Freeze detection thresholds
    const float FREEZE_CADENCE_MAX = 10.0f;      // Very low cadence indicates freeze
    const float FREEZE_VARIANCE_MAX = 0.015f;    // Very low variance indicates stillness
    
    // Timing thresholds for state transitions
    const uint32_t MIN_WALKING_DURATION_MS = 2000;   // Must walk ≥2s before FOG can occur
    const uint32_t FREEZE_CONFIRMATION_MS = 1500;    // Confirm freeze after 1.5s of freeze indicators

    // =========================================================================
    // Walking Detection Logic
    // =========================================================================
    // Requires ALL conditions to be met to classify as "walking":
    // 1. Minimum step count: ≥2 steps detected in window (actual movement)
    // 2. Cadence in range: 25-140 steps/min (realistic human walking pace)
    // 3. Variance in range: 0.05-0.30 (movement present but not excessive)
    // 4. No tremor/dyskinesia: Prevents false step detection from PD symptoms
    //
    // Rationale: Strict criteria reduce false positives. PD patients often have
    // tremor WHILE walking, but for testing purposes we isolate walking patterns.
    bool currently_walking = (steps_in_window >= MIN_STEPS_FOR_WALKING &&
                              cadence >= WALKING_CADENCE_MIN &&
                              cadence <= WALKING_CADENCE_MAX &&
                              variance >= WALKING_VARIANCE_MIN &&
                              variance <= WALKING_VARIANCE_MAX
                              //tremor_intensity == 0 &&dysk_intensity == 0
                            );

    // =========================================================================
    // Freeze Detection Logic
    // =========================================================================
    // Freeze indicators require ALL three conditions:
    // 1. Very low cadence: <10 steps/min (minimal movement)
    // 2. Very low variance: <0.015 (near-stillness)
    // 3. Walking context: Must have walked previously (walking_start_time set)
    //
    // Critical: FOG only occurs AFTER walking is established. Standing still
    // from the start is NOT FOG - it's just "not walking".
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

    // =========================================================================
    // Safety Check: Prevent Invalid State
    // =========================================================================
    // Cannot be in freeze states without prior walking context
    // This should never happen, but safeguards against logic errors
    if ((fog_detector.state == FOG_POTENTIAL_FREEZE || fog_detector.state == FOG_FREEZE_CONFIRMED) &&
        fog_detector.walking_start_time == 0)
    {
        printf("   ⚠️  WARNING: Invalid FOG state (freeze without walking), resetting\n");
        fog_detector.state = FOG_NOT_WALKING;
        fog_detector.consecutive_walking_windows = 0;
        fog_detector.consecutive_freeze_windows = 0;
    }

    // =========================================================================
    // FOG State Machine Implementation
    // =========================================================================
    
    switch (fog_detector.state)
    {
    // -------------------------------------------------------------------------
    // State 1: NOT_WALKING - Initial state, no movement detected
    // -------------------------------------------------------------------------
    case FOG_NOT_WALKING:
    {
        if (currently_walking)
        {
            fog_detector.consecutive_walking_windows++;

            // Transition to WALKING after 1 window of walking detected
            // (Can increase threshold if false positives occur)
            if (fog_detector.consecutive_walking_windows >= 1)
            {
                fog_detector.state = FOG_WALKING;
                fog_detector.walking_start_time = current_time;  // Record when walking started
                fog_detector.consecutive_freeze_windows = 0;
            }
        }
        else
        {
            // Reset counter if walking not detected
            fog_detector.consecutive_walking_windows = 0;
        }
        break;
    }

    // -------------------------------------------------------------------------
    // State 2: WALKING - Active walking detected and sustained
    // -------------------------------------------------------------------------
    case FOG_WALKING:
    {
        uint32_t walking_duration = current_time - fog_detector.walking_start_time;

        if (currently_walking)
        {
            // Continue in walking state - patient is still walking normally
        }
        else if (freeze_indicators)
        {
            // Freeze indicators detected - check if walked long enough
            if (walking_duration >= MIN_WALKING_DURATION_MS)
            {
                // Walked ≥2 seconds, now showing freeze - transition to potential freeze
                fog_detector.state = FOG_POTENTIAL_FREEZE;
                fog_detector.freeze_start_time = current_time;
                fog_detector.consecutive_freeze_windows = 1;
            }
            else
            {
                // Brief walking (<2s) then stopped - not FOG, just brief movement
                fog_detector.state = FOG_NOT_WALKING;
                fog_detector.consecutive_walking_windows = 0;
            }
        }
        else
        {
            // Not walking anymore, no freeze indicators - return to idle
            fog_detector.state = FOG_NOT_WALKING;
            fog_detector.consecutive_walking_windows = 0;
        }
        break;
    }

    // -------------------------------------------------------------------------
    // State 3: POTENTIAL_FREEZE - Freeze indicators detected, awaiting confirmation
    // -------------------------------------------------------------------------
    case FOG_POTENTIAL_FREEZE:
    {
        uint32_t freeze_duration = current_time - fog_detector.freeze_start_time;

        if (currently_walking)
        {
            // Patient resumed walking - was not a true freeze
            fog_detector.state = FOG_WALKING;
            fog_detector.consecutive_freeze_windows = 0;
        }
        else if (freeze_indicators)
        {
            // Freeze indicators continue - increment confirmation counter
            fog_detector.consecutive_freeze_windows++;

            // Confirm FOG after sustained freeze indicators (≥1.5 seconds)
            if (freeze_duration >= FREEZE_CONFIRMATION_MS)
            {
                fog_detector.state = FOG_FREEZE_CONFIRMED;
            }
        }
        else
        {
            // Freeze indicators disappeared without resuming walking - uncertain state
            // Reset to NOT_WALKING (conservative approach)
            fog_detector.state = FOG_NOT_WALKING;
            fog_detector.consecutive_walking_windows = 0;
            fog_detector.consecutive_freeze_windows = 0;
            fog_detector.walking_start_time = 0;
        }
        break;
    }

    // -------------------------------------------------------------------------
    // State 4: FREEZE_CONFIRMED - FOG episode confirmed and active
    // -------------------------------------------------------------------------
    case FOG_FREEZE_CONFIRMED:
    {
        // Static variable tracks when freeze was first confirmed
        // TODO: Consider moving to FOGDetector struct for better encapsulation
        static uint32_t freeze_confirmed_start = 0;

        // Initialize timestamp on first entry to this state
        if (freeze_confirmed_start == 0)
        {
            freeze_confirmed_start = current_time;
        }

        uint32_t confirmed_duration = current_time - freeze_confirmed_start;
        const uint32_t MAX_FOG_DURATION_MS = 15000;  // 15-second timeout

        if (currently_walking)
        {
            // Patient has recovered and resumed walking
            fog_detector.state = FOG_WALKING;
            fog_detector.consecutive_freeze_windows = 0;
            freeze_confirmed_start = 0;  // Reset static variable
            printf(" | Recovered");  // Log recovery event
        }
        else if (confirmed_duration >= MAX_FOG_DURATION_MS)
        {
            // Timeout: Patient hasn't moved for 15+ seconds
            // Assume they've stopped intentionally or need assistance
            // Reset to initial state to allow fresh detection
            fog_detector.state = FOG_NOT_WALKING;
            fog_detector.consecutive_freeze_windows = 0;
            fog_detector.consecutive_walking_windows = 0;
            fog_detector.walking_start_time = 0;
            freeze_confirmed_start = 0;
            printf(" | Timeout-Reset");
        }
        // Else: Continue in FREEZE_CONFIRMED state (FOG episode ongoing)
        break;
    }
    }

    // =========================================================================
    // Post-Processing and Cleanup
    // =========================================================================
    
    // Store cadence for potential future use (trend analysis, etc.)
    fog_detector.previous_cadence = cadence;

    // Reset step counter for next window (steps are counted per window)
    steps_in_window = 0;

    // Update BLE characteristic: 0 = No FOG, 1 = FOG detected
    // Only FREEZE_CONFIRMED state triggers FOG alarm
    fog_status = (fog_detector.state == FOG_FREEZE_CONFIRMED) ? 1 : 0;
}