#include <Arduino.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "api.h"

// Define LED GPIO pin using gpio_num_t type
#define LED_GPIO_PIN GPIO_NUM_16

// Define a mutex to protect shared resources
SemaphoreHandle_t mut; 

int hall_sensor_value = 0;
int blink_rate_ms = 500; // Default blink rate

// FreeRTOS timer handle
static TimerHandle_t blink_timer;

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

// BLE setup
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;


// Timer callback function
void IRAM_ATTR blink_timer_callback(TimerHandle_t xTimer) {
    // TODO - Toggle the LED
    gpio_set_level(LED_GPIO_PIN, !gpio_get_level(LED_GPIO_PIN));
}

// BLEServerCallbacks class
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

// Task to poll Hall effect sensor and adjust blink rate
void hall_sensor_task(void *pvParameter) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t delay = pdMS_TO_TICKS(100);

    while (1) {
        // TODO - Read the ADC value from the Hall effect sensor
        xSemaphoreTake(mut, portMAX_DELAY);
        read_hall_sensor(&hall_sensor_value);
        hall_sensor_value = hall_sensor_value;

        // TODO - Update the blink rate based on the sensor value
        blink_rate_ms = hall_sensor_value * 10;
        

        // TODO - Change timer period if needed
        xTimerChangePeriod(blink_timer, blink_rate_ms, portMAX_DELAY);

         // TODO - Update BLE value
        update_ble_value(deviceConnected, pCharacteristic, hall_sensor_value);
        

        // TODO - Delay to allow other tasks to run
        xSemaphoreGive(mut);
        vTaskDelayUntil(&last_wake_time, delay);
    }
}

// Task to print the ADC value
void print_data_task(void *pvParameter) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t delay = pdMS_TO_TICKS(1000);

    while (1) {
        // TODO - Print the ADC value
        xSemaphoreTake(mut, portMAX_DELAY);
        Serial.println(hall_sensor_value);

        xSemaphoreGive(mut);
        // TODO - Delay to allow other tasks to run
        vTaskDelayUntil(&last_wake_time, delay);
    }
}

// Setup function
void setup() {
    Serial.begin(115200); // Initialize Serial communication

    // Initialize LED GPIO
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LED_GPIO_PIN, 0);

    // Initialize FreeRTOS

    mut = xSemaphoreCreateMutex();
    xTaskCreate(hall_sensor_task, "hall_sensor_task", 2048, NULL, 5, NULL);
    xTaskCreate(print_data_task, "print_data_task", 2048, NULL, 5, NULL);

    // Create the FreeRTOS timer
    blink_timer = xTimerCreate(
        "BlinkTimer",
        pdMS_TO_TICKS(blink_rate_ms),
        pdTRUE,
        (void *)0,
        blink_timer_callback
    );

    if (blink_timer == NULL) {
        Serial.println("Failed to create the timer.");
        return;
    }

    // TODO - Start the FreeRTOS timer
    xTimerStart(blink_timer, 0); 

    // Initialize BLE
    BLEDevice::init("ESP32_Hall_Sensor_Tyler"); // Set the BLE device name
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->setValue("Initial Value");
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // Functions that are more responsive to the value of this flag are placed on the link layer and scan response is used to return more information
    BLEDevice::startAdvertising();

    Serial.println("Bluetooth setup complete and advertising started");
}

// Loop function
void loop() {
    // Nothing to do here, since FreeRTOS kernel is running
}
