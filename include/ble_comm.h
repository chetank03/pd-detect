/**
 * @file ble_comm.h
 * @brief Bluetooth Low Energy communication interface

 */

#ifndef BLE_COMM_H
#define BLE_COMM_H

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattServer.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/gatt/GattService.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "events/EventQueue.h"
#include "config.h"

// =============================================================================
// BLE Objects and State
// =============================================================================

extern events::EventQueue ble_event_queue;
extern BLE &ble_instance;
extern GattCharacteristic *tremor_char;
extern GattCharacteristic *dysk_char;
extern GattCharacteristic *fog_char;
extern GattServer *gatt_server;
extern bool ble_connected;

// =============================================================================
// Function Declarations
// =============================================================================

/**
 * @brief Schedule BLE event processing on event queue
 */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context);

/**
 * @brief BLE initialization completion callback
 * @param params BLE initialization context with error status
 */
void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params);

/**
 * @brief Update BLE characteristics and send notifications on changes
 */
void update_ble_characteristics();

/**
 * @brief Initialize BLE communication system
 */
void init_ble();

#endif // BLE_COMM_H