#include "can.h"

enum canAction
{
  WaitForCmd,
  TxDeviceInfo,
  SearchForID
};

FDCAN_HandleTypeDef hfdcan1;
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;

canAction FDCAN_State = WaitForCmd;
const uint16_t FDCAN_GlobalID = 0x7CC;
uint16_t FDCAN_TempID;
uint8_t foundExtDevice = 0;
uint8_t JunkBuf[8];
uint8_t TxData[8];
uint8_t RxData[8];

extern "C" void FDCAN1_IT0_IRQHandler(void);

void FDCAN_Error_Handler(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
}

void FDCAN_Init(void)
{
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;

  // transceiver and peripheral clock specific values
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 21;
  hfdcan1.Init.NominalTimeSeg2 = 2;

  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 5;
  hfdcan1.Init.DataTimeSeg1 = 6;
  hfdcan1.Init.DataTimeSeg2 = 5;
  //

  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    FDCAN_Error_Handler();
  }
}

void FDCAN_ConfigFilter(uint16_t canID)
{
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = canID;
  sFilterConfig.FilterID2 = FDCAN_GlobalID;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    FDCAN_Error_Handler();
  }
}

void FDCAN_ConfigTxHeader(uint16_t canID)
{
  TxHeader.Identifier = canID;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (hfdcan->Instance == FDCAN1)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{

  if (hfdcan->Instance == FDCAN1)
  {
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  }
}

extern "C" void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
  }

  if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME)
  {
    // Reply globally but put the replying ID in the data packet. 
    TxHeader.Identifier = FDCAN_GlobalID;
    memset(TxData, 0x00, 8 * sizeof(uint8_t));
    //memcpy(&TxData, (sFilterConfig.FilterID1), sizeof(uint16_t));
    FDCAN_SendMessage();
  }
  else
  {
    switch (FDCAN_State)
    {
    case WaitForCmd:
      // Set some flag to indicate that sfoc has a new command.
      if (RxHeader.Identifier = FDCAN_GlobalID)
      {

      }
      else
      {

      }
      break;

    case SearchForID:
      foundExtDevice = 1;
      break;

    default:
      break;
    }
  }
}

void FDCAN_SendMessage(void)
{
  switch (FDCAN_State)
  {
  case TxDeviceInfo:
    break;

  case SearchForID:
    TxHeader.Identifier = FDCAN_TempID;
    TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_0;
    break;

  default:
    break;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
  {
    FDCAN_Error_Handler();
  }

  // Set the tx parameters back to normal. 
  TxHeader.Identifier = sFilterConfig.FilterID1;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
}

uint16_t FDCAN_FindUniqueID(void)
{
  FDCAN_TempID = 0x000;
  FDCAN_State = SearchForID;

  while (1)
  {
    foundExtDevice = 0;
    FDCAN_SendMessage();
    delay(100);
    if (foundExtDevice)
    {
      // Try the next address.
      FDCAN_TempID++;
    }
    else
    {
      // We found a unique device ID!
      FDCAN_State = WaitForCmd;
      FDCAN_ChangeID(FDCAN_TempID);
      digitalWrite(PA7, HIGH);
      return FDCAN_TempID;
    }
  }
}

void FDCAN_ChangeID(uint16_t newID)
{
  // HAL_FDCAN_Stop();
  FDCAN_ConfigFilter(newID);
  FDCAN_ConfigTxHeader(newID);
  // HAL_FDCAN_Start();
}

void FDCAN_Start(uint16_t canID)
{
  FDCAN_Init();
  FDCAN_ConfigFilter(canID);
  FDCAN_ConfigTxHeader(canID);

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    FDCAN_Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    FDCAN_Error_Handler();
  }
}