#include "opamp.h"

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

void opamp_init(OPAMP_HandleTypeDef *hopamp, OPAMP_TypeDef *opamp)
{
  hopamp1.Instance = opamp;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;
  hopamp1.Init.InternalOutput = ENABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15; // Adjust this to change the gains of the opamp.
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp) != HAL_OK)
  {
    Error_Handler();
  }
}

void configureOPAMPs(void)
{
  opamp_init(&hopamp1, OPAMP1); // PA3
  opamp_init(&hopamp2, OPAMP2); // PB0
  opamp_init(&hopamp3, OPAMP3); // PA1
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

