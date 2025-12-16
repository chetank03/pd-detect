/**
 * @file sensor.cpp
 * @brief LSM6DSL sensor interface and data acquisition implementation

 */

#include "sensor.h"
#include "fog_detection.h"

// =============================================================================
// Hardware Objects
// =============================================================================

I2C i2c(PB_11, PB_10);  // SDA=PB_11, SCL=PB_10 (400kHz fast mode)
InterruptIn data_ready_pin(PD_11, PullDown);

// =============================================================================
// System State Variables
// =============================================================================

volatile bool new_data_available = false;
volatile uint32_t interrupt_count = 0;
volatile uint32_t pending_samples = 0;
uint32_t sample_count = 0;
uint32_t last_sample_time_ms = 0;

// =============================================================================
// Data Buffers
// =============================================================================

float accel_magnitude_buffer[WINDOW_SIZE];
float gyro_magnitude_buffer[WINDOW_SIZE];
size_t buffer_index = 0;
volatile bool window_ready = false;
uint32_t window_count = 0;

// =============================================================================
// I2C Communication Functions
// =============================================================================

bool write_register(uint8_t reg, uint8_t value) {
    char data[2] = {(char)reg, (char)value};
    int result = i2c.write(LSM6DSL_ADDR, data, 2);
    return (result == 0);  // 0 = success
}

bool read_register(uint8_t reg, uint8_t &value) {
    char reg_addr = (char)reg;
    
    // Write register address
    if (i2c.write(LSM6DSL_ADDR, &reg_addr, 1, true) != 0) {
        return false;
    }
    
    // Read data
    char data;
    if (i2c.read(LSM6DSL_ADDR, &data, 1) != 0) {
        return false;
    }
    
    value = (uint8_t)data;
    return true;
}

bool read_burst(uint8_t start_reg, uint8_t *buffer, uint8_t length) {
    char reg_addr = (char)start_reg;
    
    // Write register address
    if (i2c.write(LSM6DSL_ADDR, &reg_addr, 1, true) != 0) {
        return false;
    }
    
    // Read data
    if (i2c.read(LSM6DSL_ADDR, (char*)buffer, length) != 0) {
        return false;
    }
    
    return true;
}

// =============================================================================
// Sensor Initialization
// =============================================================================

/**
 * @brief Initialize and configure LSM6DSL IMU sensor
 * 
 * Configuration sequence:
 * 1. Verify device identity via WHO_AM_I register (0x6A expected)
 * 2. Configure common settings (BDU + auto-increment)
 * 3. Enable accelerometer: 52 Hz ODR, ±2g full scale
 * 4. Enable gyroscope: 52 Hz ODR, ±250 dps full scale
 * 5. Configure INT1 pin for data-ready interrupts
 * 
 * @return true if initialization successful, false on any error
 * 
 * @note Sample rate of 52 Hz chosen for:
 *       - Nyquist coverage of 3-7 Hz PD symptom frequencies
 *       - Efficient 3-second windows (156 samples)
 *       - Low power consumption
 */
bool init_lsm6dsl() {
    printf("\n=== Initializing LSM6DSL Sensor ===\n");
    
    // Step 1: Check WHO_AM_I register
    printf("1. Checking WHO_AM_I register...\n");
    uint8_t who_am_i = 0;
    if (!read_register(WHO_AM_I, who_am_i)) {
        printf("   ❌ ERROR: Cannot read WHO_AM_I register\n");
        return false;
    }

    printf("   WHO_AM_I = 0x%02X (expected 0x%02X)\n", who_am_i, LSM6DSL_WHO_AM_I_VAL);
    
    if (who_am_i != LSM6DSL_WHO_AM_I_VAL) {
        printf("   ❌ ERROR: Wrong device ID\n");
        return false;
    }
    printf("   ✓ Correct device detected\n\n");
    
    // Step 2: Configure CTRL3_C (Common settings)
    printf("2. Configuring common settings (CTRL3_C)...\n");
    if (!write_register(CTRL3_C, 0x44)) {
        printf("   ❌ ERROR: Cannot write CTRL3_C\n");
        return false;
    }
    printf("   ✓ BDU and auto-increment enabled\n\n");
    
    // Step 3: Configure Accelerometer (CTRL1_XL)
    printf("3. Configuring accelerometer (CTRL1_XL)...\n");
    if (!write_register(CTRL1_XL, 0x30)) {
        printf("   ❌ ERROR: Cannot write CTRL1_XL\n");
        return false;
    }
    printf("   ✓ Accelerometer: 52Hz, ±2g\n\n");
    
    // Step 4: Configure Gyroscope (CTRL2_G)
    printf("4. Configuring gyroscope (CTRL2_G)...\n");
    if (!write_register(CTRL2_G, 0x30)) {
        printf("   ❌ ERROR: Cannot write CTRL2_G\n");
        return false;
    }
    printf("   ✓ Gyroscope: 52Hz, ±250dps\n\n");
    
    // Step 5: Configure INT1 pin for data-ready
    printf("5. Configuring INT1 pin (INT1_CTRL)...\n");
    if (!write_register(INT1_CTRL, 0x03)) {
        printf("   ❌ ERROR: Cannot write INT1_CTRL\n");
        return false;
    }
    printf("   ✓ INT1 configured for accel+gyro data-ready\n\n");
    
    // Step 6: Clear any pending data by reading STATUS_REG
    uint8_t dummy;
    read_register(STATUS_REG, dummy);

    printf("=== LSM6DSL Initialization Complete ===\n\n");
    return true;
}

// =============================================================================
// Interrupt Service Routine
// =============================================================================

/**
 * @brief Hardware interrupt handler for LSM6DSL data-ready signal
 * 
 * Triggered by INT1 pin when both accelerometer and gyroscope data are ready.
 * Sets flags for main loop to process. Keeps ISR minimal for low latency.
 * 
 * @note Runs in interrupt context - no printf, no blocking operations
 * @note Uses volatile flags for thread-safe communication with main loop
 */
void data_ready_isr() {
    new_data_available = true;
    interrupt_count++;
    pending_samples++;  // Count how many samples are waiting
}

// =============================================================================
// Data Acquisition
// =============================================================================

/**
 * @brief Read sensor data and perform step detection
 * 
 * Processing pipeline:
 * 1. Read 12 bytes via I2C burst: 6 for accel (X,Y,Z), 6 for gyro (X,Y,Z)
 * 2. Convert raw 16-bit values to physical units (g, dps)
 * 3. Calculate magnitude vectors for both sensors
 * 4. Perform real-time step detection using Z-axis threshold crossing
 * 5. Buffer magnitude data for 3-second window FFT analysis
 * 
 * Step detection algorithm:
 * - Uses Z-axis acceleration only (vertical movement characteristic of steps)
 * - Adaptive threshold based on EMA baseline tracking
 * - Rising edge detection with debounce (minimum 100ms interval)
 * - Increments steps_in_window counter for FOG cadence calculation
 * 
 * @note Called from main loop when data_ready_isr() sets flag
 * @note Fills circular buffer; sets window_ready when WINDOW_SIZE samples collected
 */
void read_sensor_data() {
    // Read raw accelerometer data (6 bytes: X,Y,Z as 16-bit each) - BURST READ
    uint8_t accel_data[6];
    if (!read_burst(OUTX_L_XL, accel_data, 6)) return;
    
    int16_t accel_x_raw = (int16_t)((accel_data[1] << 8) | accel_data[0]);  // X: [1][0]
    int16_t accel_y_raw = (int16_t)((accel_data[3] << 8) | accel_data[2]);  // Y: [3][2]
    int16_t accel_z_raw = (int16_t)((accel_data[5] << 8) | accel_data[4]);  // Z: [5][4]
    
    // Read raw gyroscope data (6 bytes: X,Y,Z as 16-bit each) - BURST READ
    uint8_t gyro_data[6];
    if (!read_burst(OUTX_L_G, gyro_data, 6)) return;
    
    int16_t gyro_x_raw = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);  // X: [1][0]
    int16_t gyro_y_raw = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);  // Y: [3][2]
    int16_t gyro_z_raw = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);  // Z: [5][4]
    
    // Convert to physical units
    // Accelerometer: ±2g range, 16-bit resolution
    // Sensitivity = 0.061 mg/LSB = 0.000061 g/LSB (from datasheet)
    const float ACCEL_SCALE = 0.000061f;  // g per LSB
    float accel_x = accel_x_raw * ACCEL_SCALE;
    float accel_y = accel_y_raw * ACCEL_SCALE;
    float accel_z = accel_z_raw * ACCEL_SCALE;
    
    // Gyroscope: ±250dps range, 16-bit resolution  
    // Sensitivity = 8.75 mdps/LSB (from datasheet)
    const float GYRO_SCALE = 0.00875f;  // dps per LSB
    float gyro_x = gyro_x_raw * GYRO_SCALE;
    float gyro_y = gyro_y_raw * GYRO_SCALE;
    float gyro_z = gyro_z_raw * GYRO_SCALE;
    
    // Calculate magnitudes (what we'll use for tremor/dyskinesia detection)
    float accel_magnitude = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    float gyro_magnitude = sqrtf(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
    
    // Calculate actual sampling rate
    uint32_t current_time = Kernel::get_ms_count();
    float actual_period_ms = 0.0f;
    
    if (sample_count > 0) {
        actual_period_ms = current_time - last_sample_time_ms;
    }
    last_sample_time_ms = current_time;
    
    sample_count++;
    
    // Store accelerometer and gyroscope magnitudes in circular buffers
    accel_magnitude_buffer[buffer_index] = accel_magnitude;
    gyro_magnitude_buffer[buffer_index] = gyro_magnitude;
    buffer_index++;
    
    // Check if we've collected a full 3-second window
    if (buffer_index >= WINDOW_SIZE) {
        buffer_index = 0;  // Wrap around (circular buffer)
        window_ready = true;  // Signal that window is ready for processing
    }
    
    // Step Detection for FOG Analysis (Vertical Axis Only)
    // Use only Z-axis (vertical) to avoid tremor false triggers from hand movement
    // Walking steps primarily show up as vertical acceleration patterns
    
    const float BASELINE_EMA_ALPHA = 0.001f;
    
    // Update EMA baseline for vertical axis only
    accel_baseline_ema = BASELINE_EMA_ALPHA * accel_z + 
                        (1.0f - BASELINE_EMA_ALPHA) * accel_baseline_ema;
    
    // Detect steps using vertical axis deviation from adaptive baseline
    float vertical_deviation = fabsf(accel_z - accel_baseline_ema);
    uint32_t now = Kernel::get_ms_count();

    // Peak detection: rising edge above threshold (vertical axis only)
    if (vertical_deviation > STEP_THRESHOLD && !above_step_threshold) {
        // Check minimum time between steps (avoid double-counting)
        if (now - last_step_time_ms > MIN_STEP_INTERVAL_MS) {
            steps_in_window++;
            last_step_time_ms = now;
        }
        above_step_threshold = true;
    } 
    // Falling edge below threshold (reset for next peak)
    else if (vertical_deviation < STEP_THRESHOLD * 0.5f) {
        above_step_threshold = false;
    }
}