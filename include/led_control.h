/**
 * @file led_control.h
 * @brief LED pattern control
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "mbed.h"
#include "config.h"

extern DigitalOut led;

void update_led_indication();

#endif // LED_CONTROL_H