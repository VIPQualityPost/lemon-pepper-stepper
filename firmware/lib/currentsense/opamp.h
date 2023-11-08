#ifndef __OPAMP_H__
#define __OPAMP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_opamp.h"

extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

void configureOPAMP(void);

#ifdef __cplusplus
}
#endif

#endif

