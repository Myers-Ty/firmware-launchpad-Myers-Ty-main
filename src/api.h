#ifndef API_H

#define API_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

void update_ble_value(bool deviceConnected, BLECharacteristic* pCharacteristic, int value);

void read_hall_sensor(int* hall_sensor_value);

/* OTHER HELPFUL API SIGNATURES: 

void vTaskDelayUntil( TickType_t *pxPreviousWakeTime,
                      const TickType_t xTimeIncrement );

void vTaskDelay( const TickType_t xTicksToDelay );

TimerHandle_t* xTimerCreate( const char * const pcTimerName,
                  const TickType_t xTimerPeriodInTicks,
                  const UBaseType_t uxAutoReload,
                  void * const pvTimerID,
                  TimerCallbackFunction_t pxCallbackFunction );

BaseType_t xTimerStart( TimerHandle_t xTimer,
                        const TickType_t xTicksToWait );

BaseType_t xTimerChangePeriod( TimerHandle_t xTimer,
                                TickType_t xNewPeriod,
                                TickType_t xBlockTime );

void gpio_set_level(gpio_num_t gpio_num, uint32_t level);
bool gpio_get_level(gpio_num_t gpio_num);

*/

#endif // API_H