/**
 * @file signal_processing.h
 * @brief FFT analysis and frequency domain processing for tremor/dyskinesia detection
 * @author RTES Challenge Implementation
 * @date December 2025
 */

#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include "mbed.h"
#include "arm_math.h"
#include "config.h"

// =============================================================================
// FFT Processing Arrays
// =============================================================================

extern arm_rfft_fast_instance_f32 fft_instance;
extern bool fft_initialized;
extern float hann_window[WINDOW_SIZE];
extern bool hann_computed;
extern float combined_data[WINDOW_SIZE];
extern float accel_norm[WINDOW_SIZE], gyro_norm[WINDOW_SIZE];
extern float fft_input[FFT_SIZE];
extern float fft_output[FFT_SIZE];
extern float magnitude_spectrum[FFT_SIZE/2];

// =============================================================================
// Detection State
// =============================================================================

struct DetectionConfirmation {
    char last_raw_detection[16];    // Last raw detection from FFT
    uint8_t tremor_consecutive;      // Consecutive tremor detections
    uint8_t dysk_consecutive;        // Consecutive dyskinesia detections
    uint8_t none_consecutive;        // Consecutive "NONE" detections
    float tremor_ema_intensity;      // EMA smoothed intensity for tremor
    float dysk_ema_intensity;        // EMA smoothed intensity for dyskinesia
};

extern DetectionConfirmation detection_state;
extern uint16_t tremor_intensity;
extern uint16_t dysk_intensity;

// =============================================================================
// Function Declarations
// =============================================================================

/**
 * @brief Perform FFT analysis and detect tremor/dyskinesia
 * @param accel_data Array of accelerometer magnitude samples
 * @param gyro_data Array of gyroscope magnitude samples
 * @param size Number of samples (should be WINDOW_SIZE = 156)
 * @param sample_rate Sampling frequency in Hz
 * @param raw_condition Raw detection result ("NONE", "TREMOR", "DYSK")
 * @param raw_intensity Raw intensity value
 */
void analyze_frequency_content(float* accel_data, float* gyro_data, size_t size, float sample_rate,
                               char* raw_condition, float* raw_intensity);

/**
 * @brief Process a complete 3-second data window
 */
void process_window();

#endif // SIGNAL_PROCESSING_H