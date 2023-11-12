#include "opamp.h"

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

void initOPAMP(OPAMP_HandleTypeDef *hopamp, OPAMP_TypeDef *opamp)
{
  #ifdef USE_OPAMP_PGA
  hopamp->Instance = opamp;
  hopamp->Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp->Init.Mode = OPAMP_PGA_MODE;
  hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;
  hopamp->Init.InternalOutput = ENABLE;
  hopamp->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp->Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15; // Adjust this to change the gains of the opamp.
  hopamp->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  #else
  hopamp->Instance = opamp;
  hopamp->Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp->Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;
  hopamp->Init.InternalOutput = ENABLE;
  hopamp->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  #endif

  if (HAL_OPAMP_Init(hopamp) != HAL_OK)
    SIMPLEFOC_DEBUG("HAL OPAMP Init failed!");
}

void configureOPAMPs(void)
{
  initOPAMP(&hopamp1, OPAMP1); // PA3
  initOPAMP(&hopamp2, OPAMP2); // PB0
  initOPAMP(&hopamp3, OPAMP3); // PA1
}

void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef* opampHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(opampHandle->Instance==OPAMP1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(opampHandle->Instance==OPAMP2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if(opampHandle->Instance==OPAMP3)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef* opampHandle)
{

  if(opampHandle->Instance==OPAMP1)
  {
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
  }
  else if(opampHandle->Instance==OPAMP2)
  {
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);
  }
  else if(opampHandle->Instance==OPAMP3)
  {
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
  }
}

