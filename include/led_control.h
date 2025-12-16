/**
 * @file led_control.h
 * @brief LED pattern control for visual indication of detection states
 * @author RTES Challenge Implementation
 * @date December 2025
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "mbed.h"
#include "config.h"

// =============================================================================
// Hardware Objects
// =============================================================================

extern DigitalOut led;

// =============================================================================
// Function Declarations
// =============================================================================

/**
 * @brief Update LED1 to visually indicate current detection state
 * @note Uses phase-based timing for consistent patterns:
 *       - Tremor: 2Hz blink with intensity-proportional duty cycle
 *       - Dyskinesia: 4Hz blink with intensity-proportional duty cycle  
 *       - FOG: Fast triple-blink alarm pattern
 *       - Normal: Slow heartbeat (system alive indicator)
 */
void update_led_indication();

#endif // LED_CONTROL_H