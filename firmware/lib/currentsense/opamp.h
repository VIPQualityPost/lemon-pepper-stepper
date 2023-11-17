#ifndef __OPAMP_H__
#define __OPAMP_H__

#if defined(SIMPLEFOC_STM32_CUSTOMCURRENTSENSE)

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_opamp.h"
#include "communication/SimpleFOCDebug.h"

extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

void configureOPAMPs(void);

#endif 

#endif

