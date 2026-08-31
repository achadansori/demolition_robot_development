/**
  * Host-side layout check for the 8-byte control packet.
  * Not part of the firmware build (excluded via #ifdef).
  *
  *   gcc -I../Inc -DTEST_PACKET_LAYOUT test_packet_layout.c -o /tmp/t && /tmp/t
  *
  * The radio and the wired CAN link both carry Transmitter_Data_t verbatim -
  * NRF24_SendBinary() and CtrlCAN_Send() are handed the same buffer with no
  * repacking. The receiver decodes both with the single NRF24_DecodePayload(),
  * which reads the byte offsets and switch bit positions that ctrl_link.h
  * names. This asserts that the struct really does land where ctrl_link.h
  * says, so the two transports cannot silently disagree.
  */
#ifdef TEST_PACKET_LAYOUT

#include <assert.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "var.h"
#include "ctrl_link.h"

static uint16_t switch_word(Switch_Data_t s)
{
    uint8_t b[2];
    memcpy(b, &s, 2);
    return (uint16_t)((b[1] << 8) | b[0]);
}

int main(void)
{
    assert(sizeof(Transmitter_Data_t) == 8);
    assert(sizeof(Switch_Data_t) == 2);
    assert(sizeof(Joystick_Data_t) == 6);

    /* Byte offsets: what CtrlCAN_Send() puts in each CAN data byte. */
    assert(offsetof(Transmitter_Data_t, joystick.left_x)  == CTRL_BYTE_LEFT_X);
    assert(offsetof(Transmitter_Data_t, joystick.left_y)  == CTRL_BYTE_LEFT_Y);
    assert(offsetof(Transmitter_Data_t, joystick.right_x) == CTRL_BYTE_RIGHT_X);
    assert(offsetof(Transmitter_Data_t, joystick.right_y) == CTRL_BYTE_RIGHT_Y);
    assert(offsetof(Transmitter_Data_t, joystick.battery_percent) == CTRL_BYTE_R8);
    assert(offsetof(Transmitter_Data_t, joystick.reserved) == CTRL_BYTE_R1);
    assert(offsetof(Transmitter_Data_t, switches) == CTRL_BYTE_SW_LO);

    /* Switch bit positions within (SW_HI << 8 | SW_LO). */
    Switch_Data_t s;
    s = (Switch_Data_t){0};
    s.joy_left_btn1 = 1;  assert(switch_word(s) == (1u << CTRL_SW_JOY_LEFT_BTN1));
    s = (Switch_Data_t){0};
    s.s0 = 1;             assert(switch_word(s) == (1u << CTRL_SW_S0));
    s = (Switch_Data_t){0};
    s.s1_1 = 1;           assert(switch_word(s) == (1u << CTRL_SW_S1_1));
    s = (Switch_Data_t){0};
    s.s1_2 = 1;           assert(switch_word(s) == (1u << CTRL_SW_S1_2));
    s = (Switch_Data_t){0};
    s.s2_1 = 1;           assert(switch_word(s) == (1u << CTRL_SW_S2_1));
    s = (Switch_Data_t){0};
    s.s5_2 = 1;           assert(switch_word(s) == (1u << CTRL_SW_S5_2));
    s = (Switch_Data_t){0};
    s.motor_active = 1;   assert(switch_word(s) == (1u << CTRL_SW_MOTOR_ACTIVE));

    /* A CAN frame is at most 8 bytes, so the packet has to fit exactly. */
    assert(sizeof(Transmitter_Data_t) <= 8);

    printf("packet layout OK\n");
    return 0;
}

#endif /* TEST_PACKET_LAYOUT */
