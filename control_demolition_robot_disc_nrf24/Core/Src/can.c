/**
  ******************************************************************************
  * @file    can.c
  * @brief   CAN1 peripheral setup for the wired control link.
  ******************************************************************************
  */
#include "can.h"

CAN_HandleTypeDef hcan1;

/**
  * @brief  CAN1 at 500 kbps
  * @note   APB1 = 42 MHz. Tq = 1/(42 MHz / 6) and one bit is
  *         1 (SYNC) + 11 (BS1) + 2 (BS2) = 14 Tq -> 500 kbit/s, sample point
  *         85.7%. Identical to control_demolition_robot and the transmitter,
  *         so all three interoperate.
  *
  *         For a single-board loopback test with no transceiver attached,
  *         change CAN_MODE_NORMAL to CAN_MODE_LOOPBACK.
  * @retval None
  */
void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = ENABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  CAN1 clock and GPIO
  * @note   No NVIC setup on purpose: ctrl_can.c polls the RX FIFO from the
  *         free-running main loop, so no CAN interrupt is ever enabled and
  *         stm32f4xx_it.c needs no handler.
  * @retval None
  */
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (canHandle->Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    /* Pull-up on PD0 (CAN1_RX): with no transceiver fitted the pin would
     * float, and bxCAN needs a recessive RX line to leave initialisation
     * mode - HAL_CAN_Start() would time out and CAN would stay dead even
     * after the cable is plugged in later. A pull-up reads as bus-idle. */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
}

/**
  * @brief  CAN1 deinit
  * @retval None
  */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
  if (canHandle->Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);
  }
}
