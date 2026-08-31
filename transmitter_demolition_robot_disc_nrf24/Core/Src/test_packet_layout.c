/**
  * Host-side layout check for the 8-byte NRF24 packet.
  * Not part of the firmware build (excluded via #ifdef).
  *
  *   gcc -I../Inc -DTEST_PACKET_LAYOUT test_packet_layout.c -o /tmp/t && /tmp/t
  *
  * Guards the bit positions that control_demolition_robot_disc_nrf24/Core/Src/nrf24.c
  * decodes with (payload[7] << 8) | payload[6].
  */
#ifdef TEST_PACKET_LAYOUT

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "var.h"

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

    Switch_Data_t s = {0};
    s.s0 = 1;            assert(switch_word(s) == (1u << 4));
    s = (Switch_Data_t){0};
    s.motor_active = 1;  assert(switch_word(s) == (1u << 13));
    s = (Switch_Data_t){0};
    s.unlocked = 1;      assert(switch_word(s) == (1u << 14));

    printf("packet layout OK\n");
    return 0;
}

#endif /* TEST_PACKET_LAYOUT */
