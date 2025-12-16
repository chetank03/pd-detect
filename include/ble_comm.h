/**
 * @file ble_comm.h
 * @brief Bluetooth Low Energy communication
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

extern events::EventQueue ble_event_queue;
extern BLE &ble_instance;
extern GattCharacteristic *tremor_char;
extern GattCharacteristic *dysk_char;
extern GattCharacteristic *fog_char;
extern GattServer *gatt_server;
extern bool ble_connected;

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context);
void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params);
void update_ble_characteristics();
void init_ble();

#endif // BLE_COMM_H