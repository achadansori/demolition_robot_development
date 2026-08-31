/**
  ******************************************************************************
  * @file    ctrl_can.h
  * @brief   Wired CAN transport for the control packet (control board side).
  *
  *          Receives the same 8-byte packet the radio carries (see ctrl_link.h)
  *          as a plain standard CAN frame, and answers with a heartbeat so the
  *          transmitter knows the cable is plugged in.
  *
  *          Raw bxCAN, no CANopen stack. The two frames are bit-for-bit what
  *          CANopen puts on the wire (TPDO1 = ID 0x181 DLC 8, heartbeat =
  *          ID 0x702 DLC 1), so this board can still be driven by the CANopen
  *          bridge in receiver_demolition_robot without carrying its middleware.
  ******************************************************************************
  */
#ifndef __CTRL_CAN_H
#define __CTRL_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Heartbeat we produce = "the cable is connected". Kept out of ctrl_link.h so
 * that header stays byte-identical across all four projects. */
#define CTRL_CAN_HB_COB_ID      0x702u  /* control board -> transmitter */
#define CTRL_CAN_HB_PERIOD_MS   100u
#define CTRL_CAN_HB_STATE       0x05u   /* CANopen OPERATIONAL */

void CtrlCAN_Init(void);
bool CtrlCAN_GetData(uint8_t out[8]);
void CtrlCAN_HeartbeatTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_CAN_H */
