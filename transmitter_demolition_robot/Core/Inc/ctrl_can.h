/**
  ******************************************************************************
  * @file    ctrl_can.h
  * @brief   Wired CAN transport for the control packet (transmitter side).
  *
  *          Sends the same 8-byte packet as the radio (see ctrl_link.h) as a
  *          plain standard CAN frame, and listens for the control board's
  *          heartbeat to know whether the cable is plugged in.
  *
  *          Raw bxCAN, no CANopen stack: the two frames we need are bit-for-bit
  *          what CANopen would put on the wire anyway (TPDO1 = ID 0x181 DLC 8,
  *          heartbeat = ID 0x702 DLC 1), so this stays interoperable with the
  *          CANopen bridge in receiver_demolition_robot without carrying its
  *          middleware.
  ******************************************************************************
  */
#ifndef __CTRL_CAN_H
#define __CTRL_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Heartbeat from the control board = "the cable is connected". Not part of
 * ctrl_link.h so that header stays byte-identical across all four projects. */
#define CTRL_CAN_HB_COB_ID     0x702u  /* control board -> transmitter */
#define CTRL_CAN_HB_TIMEOUT_MS 300u    /* no heartbeat for this long = unplugged */
#define CTRL_CAN_SEND_PERIOD_MS 20u    /* 50 Hz; the main loop is free-running */

void CtrlCAN_Init(void);
void CtrlCAN_Poll(void);
bool CtrlCAN_IsLinkUp(void);
bool CtrlCAN_Send(const uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_CAN_H */
