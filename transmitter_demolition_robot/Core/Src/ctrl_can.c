/**
  ******************************************************************************
  * @file    ctrl_can.c
  * @brief   Wired CAN transport for the control packet (transmitter side).
  ******************************************************************************
  */
#include "ctrl_can.h"
#include "ctrl_link.h"
#include "can.h"

/* Last time a heartbeat arrived from the control board. Starts at 0, which
 * reads as "very old" once HAL_GetTick() passes the timeout, so we boot in
 * radio mode until the cable actually proves itself. */
static uint32_t hb_last_ms = 0;
static uint8_t  hb_seen = 0;

/**
  * @brief  Accept only the control board's heartbeat, then start the peripheral
  * @retval None
  */
void CtrlCAN_Init(void)
{
    CAN_FilterTypeDef f = {0};

    /* 16-bit ID list mode: both entries are the heartbeat ID, so anything else
     * on the bus (including our own 0x181 echoed by another node) is dropped in
     * hardware and never reaches the FIFO. */
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDLIST;
    f.FilterScale = CAN_FILTERSCALE_16BIT;
    f.FilterIdHigh = (uint16_t)(CTRL_CAN_HB_COB_ID << 5);
    f.FilterIdLow = (uint16_t)(CTRL_CAN_HB_COB_ID << 5);
    f.FilterMaskIdHigh = (uint16_t)(CTRL_CAN_HB_COB_ID << 5);
    f.FilterMaskIdLow = (uint16_t)(CTRL_CAN_HB_COB_ID << 5);
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation = ENABLE;
    f.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &f);
    HAL_CAN_Start(&hcan1);
}

/**
  * @brief  Drain the RX FIFO and timestamp any heartbeat found
  * @note   Polled, not interrupt-driven: the main loop is free-running so the
  *         3-deep FIFO cannot fall behind a 10 Hz heartbeat.
  * @retval None
  */
void CtrlCAN_Poll(void)
{
    CAN_RxHeaderTypeDef h;
    uint8_t d[8];

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &h, d) != HAL_OK) break;

        if (h.IDE == CAN_ID_STD && h.StdId == CTRL_CAN_HB_COB_ID)
        {
            hb_last_ms = HAL_GetTick();
            hb_seen = 1;
        }
    }
}

/**
  * @brief  Is the CAN cable connected?
  * @retval true if a heartbeat arrived within CTRL_CAN_HB_TIMEOUT_MS
  */
bool CtrlCAN_IsLinkUp(void)
{
    if (!hb_seen) return false;
    return (HAL_GetTick() - hb_last_ms) < CTRL_CAN_HB_TIMEOUT_MS;
}

/**
  * @brief  Put the 8-byte control packet on the bus as ID 0x181
  * @param  data: the same buffer the radio sends (Var_GetBinaryData())
  * @retval true if a mailbox accepted the frame
  */
bool CtrlCAN_Send(const uint8_t data[8])
{
    CAN_TxHeaderTypeDef h;
    uint32_t mailbox;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) return false;

    h.StdId = CTRL_LINK_COB_ID;
    h.ExtId = 0;
    h.IDE = CAN_ID_STD;
    h.RTR = CAN_RTR_DATA;
    h.DLC = 8;
    h.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan1, &h, (uint8_t *)data, &mailbox) == HAL_OK);
}
