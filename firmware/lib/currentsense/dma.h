#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dma.h"

void MX_DMA_Init(void);
void DMA1_Channel1_IRQHandler(void);
void DMA2_Channel2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif 

