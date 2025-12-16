/**
 * @file signal_processing.cpp
 * @brief FFT analysis and frequency domain processing implementation
 * @author RTES Challenge Implementation
 * @date December 2025
 */

#include "signal_processing.h"
#include "fog_detection.h"
#include <cstring>

// =============================================================================
// FFT Processing Arrays
// =============================================================================

arm_rfft_fast_instance_f32 fft_instance;
bool fft_initialized = false;
float hann_window[WINDOW_SIZE];
bool hann_computed = false;
float combined_data[WINDOW_SIZE];
float accel_norm[WINDOW_SIZE], gyro_norm[WINDOW_SIZE];
float fft_input[FFT_SIZE];
float fft_output[FFT_SIZE];
float magnitude_spectrum[FFT_SIZE/2];

// =============================================================================
// Detection State
// =============================================================================

DetectionConfirmation detection_state = {"NONE", 0, 0, 0, 0.0f, 0.0f};
uint16_t tremor_intensity = 0;
uint16_t dysk_intensity = 0;

// =============================================================================
// Signal Processing Functions
// =============================================================================

void analyze_frequency_content(float* accel_data, float* gyro_data, size_t size, float sample_rate,
                               char* raw_condition, float* raw_intensity) {
    // Defaults
    strcpy(raw_condition, "NONE");
    *raw_intensity = 0.0f;

    // FFT init once
    if (!fft_initialized) {
        arm_status st = arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
        if (st != ARM_MATH_SUCCESS) {
            printf("❌ FFT init failed\n");
            return;
        }
        fft_initialized = true;
    }

    // Hann once
    if (!hann_computed) {
        const float pi = 3.14159265359f;
        for (size_t i = 0; i < WINDOW_SIZE; i++) {
            hann_window[i] = 0.5f * (1.0f - cosf(2.0f * pi * i / (WINDOW_SIZE - 1)));
        }
        hann_computed = true;
    }

    // --- 1) DC remove + std for accel/gyro ---
    float accel_sum = 0.0f, gyro_sum = 0.0f;
    for (size_t i = 0; i < size; i++) { 
        accel_sum += accel_data[i]; 
        gyro_sum += gyro_data[i]; 
    }
    const float accel_mean = accel_sum / (float)size;
    const float gyro_mean  = gyro_sum  / (float)size;

    float accel_var = 0.0f, gyro_var = 0.0f;
    for (size_t i = 0; i < size; i++) {
        accel_norm[i] = accel_data[i] - accel_mean;
        gyro_norm[i]  = gyro_data[i]  - gyro_mean;
        accel_var += accel_norm[i] * accel_norm[i];
        gyro_var  += gyro_norm[i]  * gyro_norm[i];
    }

    const float eps = 1e-6f;
    const float accel_std = sqrtf(accel_var / (float)size) + eps;
    const float gyro_std  = sqrtf(gyro_var  / (float)size) + eps;

    // Combine (z-score each, then weighted sum)
    for (size_t i = 0; i < size; i++) {
        float az = accel_norm[i] / accel_std;
        float gz = gyro_norm[i]  / gyro_std;
        combined_data[i] = 0.7f * az + 0.3f * gz;
    }

    // --- 2) Window + zero pad ---
    for (size_t i = 0; i < size; i++) fft_input[i] = combined_data[i] * hann_window[i];
    for (size_t i = size; i < FFT_SIZE; i++) fft_input[i] = 0.0f;

    // --- 3) FFT ---
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

    // Real FFT layout: fft_output[0] = DC, fft_output[1] = Nyquist
    // fft_output[2..] = [Re(1), Im(1), Re(2), Im(2), ... , Re(N/2-1), Im(N/2-1)]
    arm_cmplx_mag_f32(&fft_output[2], magnitude_spectrum, (FFT_SIZE/2 - 1)); // 127 mags

    const float freq_res = sample_rate / (float)FFT_SIZE; // ~0.203 Hz

    // --- 4) Noise floor from 0.5–2.0 Hz (avoid DC / ultra-low drift) ---
    size_t k0 = (size_t)ceilf(0.5f / freq_res);
    size_t k1 = (size_t)floorf(2.0f / freq_res);
    if (k0 < 1) k0 = 1;
    if (k1 > (FFT_SIZE/2 - 1)) k1 = (FFT_SIZE/2 - 1); // max 127

    float noise_sum = 0.0f;
    size_t noise_cnt = 0;
    for (size_t k = k0; k <= k1; k++) {
        noise_sum += magnitude_spectrum[k - 1]; // k=1 maps to index 0
        noise_cnt++;
    }
    float noise_floor = (noise_cnt > 0) ? (noise_sum / (float)noise_cnt) : 0.25f;

    // Clamp noise floor so thresholds don't collapse
    if (noise_floor < 0.25f) noise_floor = 0.25f;

    // --- 5) Compute peaks in bands ---
    float tremor_peak = 0.0f;
    float tremor_freq = 0.0f;
    float dysk_peak   = 0.0f;
    float dysk_freq   = 0.0f;

    // Only search bins that matter (>=2.0 Hz)
    for (size_t k = 1; k <= (FFT_SIZE/2 - 1); k++) {
        float f = k * freq_res;
        if (f < 2.0f) continue;

        float mag = magnitude_spectrum[k - 1];

        if (f >= 3.0f && f <= 5.0f) {
            if (mag > tremor_peak) { tremor_peak = mag; tremor_freq = f; }
        } else if (f >= 5.0f && f <= 7.0f) {
            if (mag > dysk_peak)   { dysk_peak   = mag; dysk_freq   = f; }
        }
    }

    // --- 6) More sensitive adaptive thresholds ---
    const float tremor_threshold = noise_floor * 3.0f;   // Reduced from 10.0f
    const float dysk_threshold   = noise_floor * 4.0f;   // Reduced from 12.0f

    // --- 7) Decide condition using band dominance ---
    const float DOM_RATIO = 1.1f;

    bool tremor_detected = (tremor_peak > tremor_threshold) &&
                           (tremor_peak > dysk_peak * DOM_RATIO);

    bool dysk_detected   = (dysk_peak > dysk_threshold) &&
                           (dysk_peak > tremor_peak * DOM_RATIO);

    const char* condition = "NONE";
    float intensity_score = 0.0f;

    if (tremor_detected) {
        condition = "TREMOR";
        intensity_score = (tremor_peak - tremor_threshold) / tremor_threshold;
    } else if (dysk_detected) {
        condition = "DYSK";
        intensity_score = (dysk_peak - dysk_threshold) / dysk_threshold;
    }

    // Cap raw score
    if (intensity_score < 0.0f) intensity_score = 0.0f;
    if (intensity_score > 3.0f) intensity_score = 3.0f;


    // Return values
    strncpy(raw_condition, condition, 15);
    raw_condition[15] = '\0';
    *raw_intensity = intensity_score;

    // Print frequency analysis with correct labels and precise frequency
    if (strcmp(condition, "TREMOR") == 0) {
        printf("🔴 TREMOR %.2fHz ", tremor_freq);
    } else if (strcmp(condition, "DYSK") == 0) {
        printf("🟠 DYSK %.2fHz ", dysk_freq);
    } else {
        // Show the dominant frequency even when nothing detected
        // Search entire spectrum for actual peak
        float max_mag = 0.0f;
        float max_freq = 0.0f;
        for (size_t k = 1; k <= (FFT_SIZE/2 - 1); k++) {
            float f = k * freq_res;
            if (f < 2.0f || f > 10.0f) continue;  // Only show 2-10 Hz range
            float mag = magnitude_spectrum[k - 1];
            if (mag > max_mag) {
                max_mag = mag;
                max_freq = f;
            }
        }
        printf("⚪ %.2fHz ", max_freq);
    }
}

void process_window() {
    extern volatile bool window_ready;
    extern uint32_t window_count;
    extern bool ble_connected;
    extern uint16_t fog_status;
    
    window_ready = false;  // Clear flag
    window_count++;
    
    // Calculate actual window timing to verify 3-second intervals
    uint32_t current_time = Kernel::get_ms_count();
    static uint32_t last_window_time = 0;
    float window_interval_sec = 0.0f;
    
    if (last_window_time > 0) {
        window_interval_sec = (current_time - last_window_time) / 1000.0f;
    }
    last_window_time = current_time;

    printf("\n>>> [3-SEC WINDOW #%-4lu] ", (unsigned long)window_count);
    if (window_interval_sec > 0.0f) {
        printf("(%.1fs interval) | ", window_interval_sec);
    }
    
    // Calculate statistics on the raw data
    extern float accel_magnitude_buffer[WINDOW_SIZE];
    extern float gyro_magnitude_buffer[WINDOW_SIZE];
    
    float sum = 0.0f;
    for (size_t i = 0; i < WINDOW_SIZE; i++) {
        sum += accel_magnitude_buffer[i];
    }
    float mean = sum / WINDOW_SIZE;
    
    // Calculate variance (measure of movement)
    float variance = 0.0f;
    for (size_t i = 0; i < WINDOW_SIZE; i++) {
        float diff = accel_magnitude_buffer[i] - mean;
        variance += diff * diff;
    }
    variance /= WINDOW_SIZE;
    float std_dev = sqrtf(variance);
        
    // Perform FFT analysis and detection (only if there's enough movement)
    char raw_detection[16] = "NONE";
    float raw_intensity = 0.0f;
    
    if (std_dev >= 0.005f) {  // Lowered from 0.05f to be more sensitive
        analyze_frequency_content(accel_magnitude_buffer, gyro_magnitude_buffer, WINDOW_SIZE, TARGET_SAMPLE_RATE_HZ, 
                                  raw_detection, &raw_intensity);
    } else {
        printf("Still ");
        strcpy(raw_detection, "NONE");
        raw_intensity = 0.0f;
    }
    
    // Multi-window confirmation logic
    if (strcmp(raw_detection, "TREMOR") == 0) {
        detection_state.tremor_consecutive++;
        detection_state.dysk_consecutive = 0;
        detection_state.none_consecutive = 0;
        
        // Apply EMA smoothing to tremor intensity
        detection_state.tremor_ema_intensity = EMA_ALPHA * raw_intensity + 
                                             (1.0f - EMA_ALPHA) * detection_state.tremor_ema_intensity;
    } else if (strcmp(raw_detection, "DYSK") == 0) {
        detection_state.dysk_consecutive++;
        detection_state.tremor_consecutive = 0;
        detection_state.none_consecutive = 0;
        
        // Apply EMA smoothing to dyskinesia intensity
        detection_state.dysk_ema_intensity = EMA_ALPHA * raw_intensity + 
                                           (1.0f - EMA_ALPHA) * detection_state.dysk_ema_intensity;
    } else {  // "NONE"
        detection_state.none_consecutive++;
        detection_state.tremor_consecutive = 0;
        detection_state.dysk_consecutive = 0;
    }
    
    // Determine confirmed intensities based on consecutive windows
    // Confirm tremor if detected in 2 consecutive windows (~6 sec)
    if (detection_state.tremor_consecutive >= DETECTION_CONFIRM_WINDOWS) {
        tremor_intensity = (uint16_t)(detection_state.tremor_ema_intensity * 500.0f);  // Scale to 0-1000
        if (tremor_intensity > 1000) tremor_intensity = 1000;
        dysk_intensity = 0;  // Clear other condition
    }
    // Confirm dyskinesia if detected in 2 consecutive windows (~6 sec)
    else if (detection_state.dysk_consecutive >= DETECTION_CONFIRM_WINDOWS) {
        dysk_intensity = (uint16_t)(detection_state.dysk_ema_intensity * 500.0f);  // Scale to 0-1000
        if (dysk_intensity > 1000) dysk_intensity = 1000;
        tremor_intensity = 0;  // Clear other condition
    }
    // Clear to NONE only after 3 consecutive windows (~9 sec)
    else if (detection_state.none_consecutive >= CLEAR_CONFIRM_WINDOWS) {
        tremor_intensity = 0;
        dysk_intensity = 0;
        detection_state.tremor_ema_intensity = 0.0f;
        detection_state.dysk_ema_intensity = 0.0f;
    }
    
    // Display confirmed result
    if (tremor_intensity > 0) {
        printf("→ 🔴 CONFIRMED [%u]", tremor_intensity);
    } else if (dysk_intensity > 0) {
        printf("→ 🟠 CONFIRMED [%u]", dysk_intensity);
    } else {
        printf("→ ✅ Normal");
    }
    
    // Process FOG detection
    process_fog_detection(variance, current_time);
    
    printf("\n");  // End window processing line
    
    // BLE and LED updates would be called here
    // These will be handled in main.cpp or respective modules
}