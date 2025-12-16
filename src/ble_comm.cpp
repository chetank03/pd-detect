/**
 * @file ble_comm.cpp
 * @brief Bluetooth Low Energy communication
 */

#include "ble_comm.h"
#include "signal_processing.h"
#include "fog_detection.h"

// BLE objects and state
events::EventQueue ble_event_queue(16 * EVENTS_EVENT_SIZE);  // Event processing queue
BLE &ble_instance = BLE::Instance();
GattCharacteristic *tremor_char = nullptr;
GattCharacteristic *dysk_char = nullptr;
GattCharacteristic *fog_char = nullptr;
GattServer *gatt_server = nullptr;
bool ble_connected = false;

// Previous values for change detection
static uint16_t previous_tremor = 0;
static uint16_t previous_dysk = 0;
static uint16_t previous_fog = 0;

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    ble_event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

// GAP event handler for connection/disconnection
class PDGapEventHandler : public ble::Gap::EventHandler {
    void onConnectionComplete(const ble::ConnectionCompleteEvent &event) override {
        if (event.getStatus() == BLE_ERROR_NONE) {
            ble_connected = true;
            printf("\n📱 BLE Device Connected!\n\n");
        }
    }
    
    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) override {
        ble_connected = false;
        printf("\n📱 BLE Device Disconnected\n\n");
        
        // Restart advertising to allow reconnection
        ble_instance.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        printf("✓ Advertising restarted\n\n");
    }
};

static PDGapEventHandler gap_event_handler;

void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params) {
    extern uint16_t tremor_intensity;
    extern uint16_t dysk_intensity;
    extern uint8_t fog_status;

    if (params->error != BLE_ERROR_NONE) {
        printf("❌ BLE initialization failed\n");
        return;
    }
    
    printf("✓ BLE initialized successfully\n");

    BLE &ble = params->ble;
    gatt_server = &ble.gattServer();
    
    // Create three GATT characteristics: tremor, dyskinesia, FOG
    tremor_char = new GattCharacteristic(
        TREMOR_CHAR_UUID_STR,
        (uint8_t*)&tremor_intensity,
        sizeof(tremor_intensity),
        sizeof(tremor_intensity),
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
    );
    
    dysk_char = new GattCharacteristic(
        DYSK_CHAR_UUID_STR,
        (uint8_t*)&dysk_intensity,
        sizeof(dysk_intensity),
        sizeof(dysk_intensity),
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
    );
    
    fog_char = new GattCharacteristic(
        FOG_CHAR_UUID_STR,
        (uint8_t*)&fog_status,
        sizeof(fog_status),
        sizeof(fog_status),
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
    );
    
    // Register GATT service with all characteristics
    GattCharacteristic *char_table[] = {tremor_char, dysk_char, fog_char};
    GattService pd_service(PD_SERVICE_UUID_STR, char_table, 3);
    
    gatt_server->addService(pd_service);
    
    // Configure advertising parameters
    ble::AdvertisingParameters adv_params(
        ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
        ble::adv_interval_t(ble::millisecond_t(1000))
    );
    
    ble.gap().setAdvertisingParameters(ble::LEGACY_ADVERTISING_HANDLE, adv_params);
    
    uint8_t adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder adv_data_builder(adv_buffer);
    
    adv_data_builder.setFlags();
    adv_data_builder.setName("PD_Detector");
    
    ble_error_t error = ble.gap().setAdvertisingPayload(
        ble::LEGACY_ADVERTISING_HANDLE,
        adv_data_builder.getAdvertisingData()
    );
    
    if (error != BLE_ERROR_NONE) {
        printf("❌ Failed to set advertising payload\n");
        return;
    }
    
    error = ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    
    if (error != BLE_ERROR_NONE) {
        printf("❌ Failed to start advertising\n");
        return;
    }

    printf("✓ BLE advertising started\n");
    printf("✓ Device name: PD_Detector\n");
    printf("✓ Ready to connect from phone!\n\n");
}

void init_ble() {
    ble_instance.onEventsToProcess(schedule_ble_events);
    ble_instance.gap().setEventHandler(&gap_event_handler);
    ble_instance.init(on_ble_init_complete);
}

// Update BLE characteristics when values change
void update_ble_characteristics() {
    extern uint16_t tremor_intensity;
    extern uint16_t dysk_intensity;
    extern uint8_t fog_status;
    
    if (!ble_connected || gatt_server == nullptr) return;

    // Check which values changed
    bool tremor_changed = (tremor_intensity != previous_tremor);
    bool dysk_changed = (dysk_intensity != previous_dysk);
    bool fog_changed = (fog_status != previous_fog);

    if (tremor_changed) {
        gatt_server->write(
            tremor_char->getValueHandle(),
            (uint8_t*)&tremor_intensity,
            sizeof(tremor_intensity)
        );

        if (tremor_intensity > 0) {
            printf("   📢 BLE NOTIFICATION: TREMOR intensity = %u\n", tremor_intensity);
        } else {
            printf("   📢 BLE NOTIFICATION: TREMOR cleared\n");
        }

        previous_tremor = tremor_intensity;
    }

    if (dysk_changed) {
        gatt_server->write(
            dysk_char->getValueHandle(),
            (uint8_t*)&dysk_intensity,
            sizeof(dysk_intensity)
        );

        if (dysk_intensity > 0) {
            printf("   📢 BLE NOTIFICATION: DYSK intensity = %u\n", dysk_intensity);
        } else {
            printf("   📢 BLE NOTIFICATION: DYSK cleared\n");
        }

        previous_dysk = dysk_intensity;
    }

    if (fog_changed) {
        gatt_server->write(
            fog_char->getValueHandle(),
            (uint8_t*)&fog_status,
            sizeof(fog_status)
        );

        if (fog_status == 1) {
            printf("   📢 BLE NOTIFICATION: FOG detected!\n");
        } else {
            printf("   📢 BLE NOTIFICATION: FOG cleared\n");
        }

        previous_fog = fog_status;
    }

    if (tremor_changed || dysk_changed || fog_changed) {
        printf("   BLE characteristics updated and notifications sent!\n");
    }
}