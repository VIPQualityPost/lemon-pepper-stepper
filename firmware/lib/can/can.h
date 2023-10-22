/** 
 * CAN header
*/

#ifndef AIOLI_CAN_H
#define AIOLI_CAN_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

void FDCAN_Start(uint16_t canID);
void FDCAN_SendMessage();
void FDCAN_ChangeID(uint16_t newID);
uint16_t FDCAN_FindUniqueID(void);

#endif 