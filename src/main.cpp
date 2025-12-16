/**
 * @file main.cpp
 * @brief Parkinson's Disease Movement Detection System
 */

#include "mbed.h"
#include "config.h"
#include "sensor.h"
#include "signal_processing.h"
#include "fog_detection.h"
#include "ble_comm.h"
#include "led_control.h"

// Serial console

BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) {
    return &serial_port;
}

int main() {
    // Clear screen and position cursor at top
    printf("\033[2J\033[H");
    ThisThread::sleep_for(100ms);
    
    // Startup banner
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                                                               ║\n");
    printf("║   PARKINSON'S DISEASE MOVEMENT DETECTOR                       ║\n");
    printf("║   Modular Architecture - Multiple Files                       ║\n");
    printf("║                                                               ║\n");
    printf("║   Board: STM32 DISCO-L475VG-IOT01A                            ║\n");
    printf("║   Sensor: LSM6DSL (Accel + Gyro)                              ║\n");
    printf("║   Sample Rate: %.0f Hz | Window: 3 sec | FFT: %zu points        ║\n", 
        TARGET_SAMPLE_RATE_HZ, FFT_SIZE);
    printf("║                                                               ║\n");
    ThisThread::sleep_for(150ms);
    
    printf("║   Detection Capabilities:                                     ║\n");
    printf("║   🔴 Tremor: 3-5 Hz rhythmic oscillations                     ║\n");
    printf("║   🟠 Dyskinesia: 5-7 Hz dance-like movements                  ║\n");
    printf("║   ❄️  FOG: Freezing after walking (step detection)             ║\n");
    printf("║                                                               ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    ThisThread::sleep_for(200ms);

    // Configure I2C frequency (400kHz = fast mode)
    i2c.frequency(400000);
    printf("I2C configured at 400kHz\n\n");
    ThisThread::sleep_for(100ms);

    // Initialize sensor
    if (!init_lsm6dsl()) {
        printf("\n");
        printf("╔═══════════════════════════════════════════════════════════════╗\n");
        printf("║                    ❌ INITIALIZATION FAILED ❌                 ║\n");
        printf("║                                                               ║\n");
        printf("║  Check:                                                       ║\n");
        printf("║  1. Sensor connections (I2C: PB_11=SDA, PB_10=SCL)            ║\n");
        printf("║  2. Power supply                                              ║\n");
        printf("║  3. I2C address (0x6A)                                        ║\n");
        printf("╚═══════════════════════════════════════════════════════════════╝\n");
        
        // Blink LED rapidly to indicate error
        while (true) { 
            led = !led;
            ThisThread::sleep_for(200ms);
        }
    }

    // Initialize subsystems
    init_fog_detection();
    
    // Attach interrupt handler
    data_ready_pin.rise(&data_ready_isr);
    printf("\n✓ Interrupt handler attached to INT1 pin\n\n");
    ThisThread::sleep_for(200ms);

    // Initialize BLE
    printf("Initializing BLE...\n");
    ThisThread::sleep_for(100ms);
    init_ble();
    
    // Wait a bit for BLE to initialize
    ThisThread::sleep_for(300ms);
    
    printf("✓ BLE initialized successfully\n");
    printf("✓ BLE advertising started\n");
    printf("✓ Device name: PD_Detector\n");
    printf("✓ Ready to connect from phone!\n\n");
    ThisThread::sleep_for(200ms);
        
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                🚀 STARTING DETECTION SYSTEM 🚀                ║\n");
    printf("║                                                               ║\n");
    printf("║  ACTIVE FEATURES:                                             ║\n");
    ThisThread::sleep_for(100ms);
    
    printf("║  ✓ 52 Hz interrupt-driven sampling                            ║\n");
    printf("║  ✓ 3-second windowed data collection                          ║\n");
    printf("║  ✓ FFT-based frequency analysis                               ║\n");
    printf("║  ✓ Tremor detection (3-5 Hz)                                  ║\n");
    printf("║  ✓ Dyskinesia detection (5-7 Hz)                              ║\n");
    printf("║  ✓ FOG detection (step tracking + gait analysis)              ║\n");
    printf("║  ✓ BLE data transmission (Phase 5)                            ║\n");
    printf("║                                                               ║\n");
    ThisThread::sleep_for(150ms);
    
    printf("║  MODULAR ARCHITECTURE:                                        ║\n");
    printf("║  📁 sensor.cpp - LSM6DSL interface & data acquisition         ║\n");
    printf("║  📁 signal_processing.cpp - FFT analysis & detection          ║\n");
    printf("║  📁 fog_detection.cpp - Freezing of Gait detection            ║\n");
    printf("║  📁 ble_comm.cpp - Bluetooth communication                    ║\n");
    printf("║  📁 led_control.cpp - Visual indication patterns              ║\n");
    printf("║  📁 config.h - System configuration & constants               ║\n");
    printf("║                                                               ║\n");
    ThisThread::sleep_for(150ms);
    
    printf("║  BLE ADVERTISING:                                             ║\n");
    printf("║  Device Name: PD_Detector                                     ║\n");
    printf("║  (Note: Full BLE transmission available via app)              ║\n");
    printf("║                                                               ║\n");
    ThisThread::sleep_for(100ms);
    
    printf("║  LED VISUAL INDICATION:                                       ║\n");
    printf("║  🔴 Tremor: Blink ~2Hz (duty cycle ∝ intensity)               ║\n");
    printf("║  🟠 Dyskinesia: Blink ~4Hz (duty cycle ∝ intensity)           ║\n");
    printf("║  ❄️  FOG: Fast triple-blink alarm pattern                      ║\n");
    printf("║  💚 None: Slow heartbeat (system alive)                       ║\n");
    printf("║                                                               ║\n");
    ThisThread::sleep_for(100ms);
    
    printf("║  BLE DATA FORMAT (3 characteristics):                         ║\n");
    printf("║  📊 Tremor Intensity: 0-1000 scale                            ║\n");
    printf("║  📊 Dyskinesia Intensity: 0-1000 scale                        ║\n");
    printf("║  📊 FOG Status: 0=NO_FOG, 1=FOG_DETECTED                      ║\n");
    printf("║                                                               ║\n");
    ThisThread::sleep_for(100ms);
    
    printf("║  TEST SCENARIOS (watch LED patterns):                         ║\n");
    printf("║  🔴 Shake slowly (4 Hz) → Tremor                              ║\n");
    printf("║  🟠 Shake fast (6 Hz) → Dyskinesia                            ║\n");
    printf("║  ❄️  Walk in place, then stop → FOG                            ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    ThisThread::sleep_for(200ms);
    
    printf(">>> System ready - Starting detection...\n\n");
    ThisThread::sleep_for(200ms);
        
    uint32_t last_diagnostic_time = 0;
    uint32_t last_poll_time = 0;
    uint32_t last_interrupt_time = Kernel::get_ms_count();  // Initialize to current time
    uint32_t last_status_time = 0;  // Control BLE/LED status output
    bool last_ble_connected = false;  // Track BLE connection changes
    uint32_t last_tremor = 0, last_dysk = 0, last_fog = 0;  // Track detection changes
        
    // Main loop
    while (true) {
        uint32_t now = Kernel::get_ms_count();
        
        // Update LED indication patterns continuously
        update_led_indication();
            
        // Diagnostic output every 10 seconds (reduced frequency)
        if (now - last_diagnostic_time >= 10000) {
            printf("\n[Health] %lu samples, %lu windows, %.1fs/window\n\n", 
                sample_count, (unsigned long)window_count, 
                (window_count > 0) ? (now / 1000.0f) / window_count : 0.0f);
            last_diagnostic_time = now;
        }
            
        // Method 1: Process ALL pending samples (prevents sample loss)
        if (new_data_available) {
            // Process all pending samples to prevent accumulation
            while (pending_samples > 0) {
                new_data_available = false;
                last_interrupt_time = now;
                read_sensor_data();
                
                // Atomically decrement pending count
                __disable_irq();
                if (pending_samples > 0) pending_samples--;
                __enable_irq();
                
                // Check if more samples arrived during processing
                if (pending_samples == 0) {
                    new_data_available = false;
                    break;
                }
            }
        }
        // Method 2: Polling fallback - only if no interrupts for >100ms
        else if ((now - last_interrupt_time > 100) && (now - last_poll_time >= 19)) {
            last_poll_time = now;
            
            // Check if data is actually ready
            uint8_t status = 0;
            if (read_register(STATUS_REG, status)) {
                // Bit 0 = XLDA (accel data available)
                // Bit 1 = GDA (gyro data available)
                bool accel_ready = (status & 0x01) != 0;
                bool gyro_ready = (status & 0x02) != 0;
                
                if (accel_ready && gyro_ready) {
                    // Data is available but interrupt didn't fire!
                    // Read it anyway using polling mode
                    read_sensor_data();
                }
            }
        }
            
        // Check if a complete window is ready for processing
        if (window_ready) {
            process_window();
        }
        
        // Process BLE events
        ble_event_queue.dispatch_once();
        
        // Check for status changes or periodic updates (every 5 seconds)
        bool status_changed = (ble_connected != last_ble_connected) || 
                             (tremor_intensity != last_tremor) ||
                             (dysk_intensity != last_dysk) ||
                             (fog_status != last_fog);
        
        bool periodic_update = (now - last_status_time >= 5000);
        
        if (status_changed || periodic_update) {
            last_status_time = now;
            last_ble_connected = ble_connected;
            last_tremor = tremor_intensity;
            last_dysk = dysk_intensity; 
            last_fog = fog_status;
            
            // Update BLE characteristics if connected
            if (ble_connected) {
                printf("📡 BLE TX: Tremor=%u/1000, Dysk=%u/1000, FOG=%s ✓\n", 
                       tremor_intensity, dysk_intensity, (fog_status == 1) ? "ALARM" : "OK");
                update_ble_characteristics();
            } else {
                printf("📡 BLE: Not connected (advertising...)\n");
            }
            

        }
        
        // Always update BLE when connected (but don't spam console)
        if (ble_connected && !status_changed && !periodic_update) {
            update_ble_characteristics();
        }
        
        // Small delay to prevent busy-waiting
        ThisThread::sleep_for(1ms);
    }
}