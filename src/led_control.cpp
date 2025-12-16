/**
 * @file led_control.cpp
 * @brief LED pattern control
 */

#include "led_control.h"

// Hardware
DigitalOut led(LED1);

void update_led_indication() {
    extern uint16_t tremor_intensity;
    extern uint16_t dysk_intensity;
    extern uint16_t fog_status;
    
    uint32_t now = Kernel::get_ms_count();
    
    if (fog_status == 1) {
        uint32_t phase = now % FOG_CYCLE_PERIOD_MS;
        bool blink_on = ((phase < 100) || (phase >= 200 && phase < 300) || (phase >= 400 && phase < 500));
        led = blink_on;
    }
    else if (tremor_intensity > 0) {
        // TREMOR: 2Hz blink with intensity-proportional duty cycle
        uint32_t phase = now % TREMOR_TOTAL_PERIOD_MS;
        uint32_t duty_cycle_percent = 20 + ((tremor_intensity * 60) / 1000);  // 20-80% duty cycle
        uint32_t on_time_ms = (TREMOR_TOTAL_PERIOD_MS * duty_cycle_percent) / 100;
        led = (phase < on_time_ms);
    }
    else if (dysk_intensity > 0) {
        // DYSKINESIA: 4Hz blink with intensity-proportional duty cycle
        uint32_t phase = now % DYSK_TOTAL_PERIOD_MS;
        uint32_t duty_cycle_percent = 20 + ((dysk_intensity * 60) / 1000);  // 20-80% duty cycle
        uint32_t on_time_ms = (DYSK_TOTAL_PERIOD_MS * duty_cycle_percent) / 100;
        led = (phase < on_time_ms);
    }
    else {
        // NO CONDITIONS: Slow heartbeat (0.5Hz = 2000ms period, 10% duty cycle)
        uint32_t phase = now % HEARTBEAT_PERIOD_MS;
        led = (phase < 200);  // 200ms on, 1800ms off
    }
}