/**
  ******************************************************************************
  * @file    ctrl_can.c
  * @brief   Wired CAN transport for the control packet (control board side).
  ******************************************************************************
  */
#include "ctrl_can.h"
#include "ctrl_link.h"
#include "can.h"

/**
  * @brief  Accept only the control packet, then start the peripheral
  * @retval None
  */
void CtrlCAN_Init(void)
{
    CAN_FilterTypeDef f = {0};

    /* 16-bit ID list mode, all four slots set to the control-packet ID: our own
     * heartbeat and anything else on the bus is dropped in hardware and never
     * reaches the FIFO. */
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDLIST;
    f.FilterScale = CAN_FILTERSCALE_16BIT;
    f.FilterIdHigh = (uint16_t)(CTRL_LINK_COB_ID << 5);
    f.FilterIdLow = (uint16_t)(CTRL_LINK_COB_ID << 5);
    f.FilterMaskIdHigh = (uint16_t)(CTRL_LINK_COB_ID << 5);
    f.FilterMaskIdLow = (uint16_t)(CTRL_LINK_COB_ID << 5);
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation = ENABLE;
    f.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &f);
    HAL_CAN_Start(&hcan1);
}

/**
  * @brief  Fetch the newest control packet, if one arrived
  * @param  out: receives the 8 payload bytes
  * @note   Polled, not interrupt-driven: the main loop is free-running so the
  *         3-deep FIFO cannot fall behind a 50 Hz sender. When more than one
  *         frame is queued we drain to the newest - stale stick positions are
  *         worse than none.
  * @retval true if a frame was read
  */
bool CtrlCAN_GetData(uint8_t out[8])
{
    CAN_RxHeaderTypeDef h;
    uint8_t d[8];
    bool got = false;

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &h, d) != HAL_OK) break;

        if (h.IDE == CAN_ID_STD && h.StdId == CTRL_LINK_COB_ID && h.DLC == 8)
        {
            for (uint8_t i = 0; i < 8; i++) out[i] = d[i];
            got = true;
        }
    }

    return got;
}

/**
  * @brief  Announce that the cable is connected, every CTRL_CAN_HB_PERIOD_MS
  * @note   This is the transmitter's only signal to switch to CAN, so it must
  *         keep running even while the control packet stream is idle.
  * @retval None
  */
void CtrlCAN_HeartbeatTask(void)
{
    static uint32_t last_ms = 0;

    CAN_TxHeaderTypeDef h;
    uint8_t d = CTRL_CAN_HB_STATE;
    uint32_t mailbox;

    if ((HAL_GetTick() - last_ms) < CTRL_CAN_HB_PERIOD_MS) return;
    last_ms = HAL_GetTick();

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) return;

    h.StdId = CTRL_CAN_HB_COB_ID;
    h.ExtId = 0;
    h.IDE = CAN_ID_STD;
    h.RTR = CAN_RTR_DATA;
    h.DLC = 1;
    h.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan1, &h, &d, &mailbox);
}
