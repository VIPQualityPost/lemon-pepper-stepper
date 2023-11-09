#ifndef __DMA_H__
#define __DMA_H__

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_dma.h>

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

void MX_DMA_Init(void);

#endif 

