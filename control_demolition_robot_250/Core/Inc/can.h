/**
  ******************************************************************************
  * @file    can.h
  * @brief   CAN1 peripheral setup for the wired control link.
  *
  *          PD0 = CAN1_RX, PD1 = CAN1_TX, 500 kbps. Same peripheral, pins and
  *          bit timing as control_demolition_robot, which runs this on the same
  *          F407 Discovery board.
  ******************************************************************************
  */
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern CAN_HandleTypeDef hcan1;

void MX_CAN1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
