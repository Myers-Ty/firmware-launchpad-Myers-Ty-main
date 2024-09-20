#include "api.h"

void update_ble_value(bool deviceConnected, BLECharacteristic* pCharacteristic, int value ) {
    if (deviceConnected) {
            if (pCharacteristic) {
                pCharacteristic->setValue(value);
                pCharacteristic->notify();
            }
        }
}

void read_hall_sensor(int* hall_sensor_value) {
    
    *hall_sensor_value = hallRead();
}

