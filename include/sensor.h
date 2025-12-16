/**
 * @file sensor.h
 * @brief LSM6DSL sensor interface and data acquisition functions

 */

#ifndef SENSOR_H
#define SENSOR_H

#include "mbed.h"
#include "config.h"

// =============================================================================
// Hardware Objects
// =============================================================================

extern I2C i2c;
extern InterruptIn data_ready_pin;

// =============================================================================
// System State Variables
// =============================================================================

extern volatile bool new_data_available;
extern volatile uint32_t interrupt_count;
extern volatile uint32_t pending_samples;
extern uint32_t sample_count;
extern uint32_t last_sample_time_ms;

// =============================================================================
// Data Buffers
// =============================================================================

extern float accel_magnitude_buffer[WINDOW_SIZE];
extern float gyro_magnitude_buffer[WINDOW_SIZE];
extern size_t buffer_index;
extern volatile bool window_ready;
extern uint32_t window_count;

// =============================================================================
// Function Declarations
// =============================================================================

/**
 * @brief Write a single byte to LSM6DSL register
 * @param reg Register address
 * @param value Data byte to write
 * @return true if successful, false on I2C error
 */
bool write_register(uint8_t reg, uint8_t value);

/**
 * @brief Read a single byte from LSM6DSL register
 * @param reg Register address
 * @param value Reference to store read data
 * @return true if successful, false on I2C error
 */
bool read_register(uint8_t reg, uint8_t &value);

/**
 * @brief Read multiple bytes from LSM6DSL (auto-increment enabled)
 * @param start_reg Starting register address
 * @param buffer Destination buffer for read data
 * @param length Number of bytes to read
 * @return true if successful, false on I2C error
 */
bool read_burst(uint8_t start_reg, uint8_t *buffer, uint8_t length);

/**
 * @brief Initialize and configure LSM6DSL sensor for motion detection
 * @return true if initialization successful, false on error
 */
bool init_lsm6dsl();

/**
 * @brief ISR for sensor data-ready interrupt
 */
void data_ready_isr();

/**
 * @brief Read and process one sensor sample
 */
void read_sensor_data();

#endif // SENSOR_H