/**
 * @file config.h
 * @brief Configuration constants and parameters for Parkinson's Disease Detection System
 * @author RTES Challenge Implementation
 * @date December 2025
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/UUID.h"

// =============================================================================
// Hardware Configuration
// =============================================================================

// LSM6DSL Sensor Register Map
#define LSM6DSL_ADDR        (0x6A << 1)  // I2C device address (7-bit left-shifted)
#define WHO_AM_I            0x0F  // Device identification register
#define CTRL1_XL            0x10  // Accelerometer control register
#define CTRL2_G             0x11  // Gyroscope control register  
#define CTRL3_C             0x12  // Common control register
#define INT1_CTRL           0x0D  // Interrupt 1 control register
#define STATUS_REG          0x1E  // Data status register
#define OUTX_L_XL           0x28  // Accelerometer data start address
#define OUTX_L_G            0x22  // Gyroscope data start address
#define LSM6DSL_WHO_AM_I_VAL  0x6A // Device identification value

// =============================================================================
// Signal Processing Configuration
// =============================================================================

/** Target sampling frequency for motion data acquisition */
const float TARGET_SAMPLE_RATE_HZ = 52.0f;

/** Analysis window parameters for frequency domain processing */
const size_t WINDOW_SIZE = 156;  // 3.0 seconds × 52 Hz = 156 samples
const size_t FFT_SIZE = 256;     // Zero-padded to next power of 2 for efficiency

// =============================================================================
// Detection Parameters
// =============================================================================

/** Multi-window confirmation system */
const uint8_t DETECTION_CONFIRM_WINDOWS = 2;  // Need 2 consecutive windows (~6 sec) for realistic confirmation
const uint8_t CLEAR_CONFIRM_WINDOWS = 3;      // Need 3 consecutive windows (~9 sec) to clear symptoms
const float EMA_ALPHA = 0.3f;                 // More stable smoothing for medical reliability

/** Step detection parameters (tuned for vertical axis sensitivity) */
const float STEP_THRESHOLD = 0.05f;          // Lowered from 0.10f - more sensitive step detection
const uint32_t MIN_STEP_INTERVAL_MS = 100;   // Lowered from 150ms - faster step detection

/** LED pattern timing constants for different detection states */
const uint32_t TREMOR_TOTAL_PERIOD_MS = 500;     // 2Hz pattern (500ms cycle)
const uint32_t DYSK_TOTAL_PERIOD_MS = 250;       // 4Hz pattern (250ms cycle)  
const uint32_t FOG_CYCLE_PERIOD_MS = 1000;       // Triple-blink alarm cycle
const uint32_t HEARTBEAT_PERIOD_MS = 2000;       // System alive indicator

// =============================================================================
// BLE Configuration
// =============================================================================

// UUIDs for service and characteristics
extern const char* PD_SERVICE_UUID_STR;
extern const char* TREMOR_CHAR_UUID_STR;
extern const char* DYSK_CHAR_UUID_STR;
extern const char* FOG_CHAR_UUID_STR;

#endif // CONFIG_H