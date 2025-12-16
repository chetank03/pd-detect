/**
 * @file sensor.h
 * @brief LSM6DSL sensor interface and data acquisition
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "mbed.h"
#include "config.h"

extern I2C i2c;
extern InterruptIn data_ready_pin;

extern volatile bool new_data_available;
extern volatile uint32_t interrupt_count;
extern volatile uint32_t pending_samples;
extern uint32_t sample_count;
extern uint32_t last_sample_time_ms;

extern float accel_magnitude_buffer[WINDOW_SIZE];
extern float gyro_magnitude_buffer[WINDOW_SIZE];
extern size_t buffer_index;
extern volatile bool window_ready;
extern uint32_t window_count;

bool write_register(uint8_t reg, uint8_t value);
bool read_register(uint8_t reg, uint8_t &value);
bool read_burst(uint8_t start_reg, uint8_t *buffer, uint8_t length);
bool init_lsm6dsl();
void data_ready_isr();
void read_sensor_data();

#endif // SENSOR_H