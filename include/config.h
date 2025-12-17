/**
 * @file config.h
 * @brief Configuration constants and parameters
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/UUID.h"

// Hardware configuration
#define LSM6DSL_ADDR        (0x6A << 1)
#define WHO_AM_I            0x0F
#define CTRL1_XL            0x10
#define CTRL2_G             0x11
#define CTRL3_C             0x12
#define INT1_CTRL           0x0D
#define STATUS_REG          0x1E
#define OUTX_L_XL           0x28
#define OUTX_L_G            0x22
#define LSM6DSL_WHO_AM_I_VAL  0x6A

// Signal processing
const float TARGET_SAMPLE_RATE_HZ = 52.0f;
const size_t WINDOW_SIZE = 156;
const size_t FFT_SIZE = 256;

// Detection parameters
const uint8_t DETECTION_CONFIRM_WINDOWS = 3;
const uint8_t CLEAR_CONFIRM_WINDOWS = 3;
const float EMA_ALPHA = 0.3f;

const float STEP_THRESHOLD = 0.03f;
const uint32_t MIN_STEP_INTERVAL_MS = 100;

const uint32_t TREMOR_TOTAL_PERIOD_MS = 500;
const uint32_t DYSK_TOTAL_PERIOD_MS = 250;
const uint32_t FOG_CYCLE_PERIOD_MS = 1000;
const uint32_t HEARTBEAT_PERIOD_MS = 2000;

// BLE configuration
extern const char* PD_SERVICE_UUID_STR;
extern const char* TREMOR_CHAR_UUID_STR;
extern const char* DYSK_CHAR_UUID_STR;
extern const char* FOG_CHAR_UUID_STR;

#endif // CONFIG_H