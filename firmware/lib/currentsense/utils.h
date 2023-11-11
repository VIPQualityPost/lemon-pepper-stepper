#ifndef __UTILS_H__
#define __UTILS_H__

#if defined(SIMPLEFOC_STM32_CUSTOMCURRENTSENSE)

#include "stm32g4xx_hal.h"

#include "communication/SimpleFOCDebug.h"
// #include "current_sense/hardware_api.h"
#include "current_sense/hardware_specific/stm32/stm32_mcu.h"
#include "drivers/hardware_specific/stm32/stm32_mcu.h"
float _readADCVoltageInline(const int pinA, const void* cs_params);
// float _readADCVoltageInline(const uint8_t pin, const void *cs_params);
float _readVoltageLowSide(const int pinA, const void *cs_params);
void *_configureADCInline(const void *driver_params, const int pinA, const int pinB, const int pinC);
void *_configureADCLowSide(const void *driver_params, const int pinA, const int pinB, const int pinC);
void _driverSyncLowSide(void *_driver_params, void *_cs_params);


#endif
#endif