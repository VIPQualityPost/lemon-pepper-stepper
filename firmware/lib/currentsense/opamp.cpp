#include "opamp.h"

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp3;

void configureOPAMPs(void)
{
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;
  hopamp1.Init.InternalOutput = ENABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
    SIMPLEFOC_DEBUG("OPAMP1 init failed.");

  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
    SIMPLEFOC_DEBUG("OPAMP3 init failed.");
}

void OPAMP_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_2;         //PA2 -> ADC1_IN3
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;         //PA3 -> OPAMP1_VINP1
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_13;        //PB13 -> OPAMP3_VINP1
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef *opampHandle)
{

  if (opampHandle->Instance == OPAMP1)
  {
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
  }
  else if (opampHandle->Instance == OPAMP3)
  {
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
  }
}
