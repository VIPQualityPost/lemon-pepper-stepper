#ifndef __ADC_H__
#define __ADC_H__

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "communication/SimpleFOCDebug.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void ADC_DMA_Init(ADC_HandleTypeDef* adcHandle);

#endif 

