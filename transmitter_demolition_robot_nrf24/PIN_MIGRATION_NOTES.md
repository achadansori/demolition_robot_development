# Pin Migration Notes - LoRa to NRF24L01 with Hardware SPI2

## Original Pin Configuration (LoRa)

### Communication (UART1):
- PA9:  USART1_TX (to LoRa RX)
- PA10: USART1_RX (from LoRa TX)

### Mode Control:
- PB8:  Lora_M1 (GPIO Output)
- PB9:  Lora_M0 (GPIO Output)

### Switches (Blocking SPI2):
- PB13: S4_1 (GPIO Input)
- PB14: S4_2 (GPIO Input)
- PB15: S5_1 (GPIO Input)

## New Pin Configuration (NRF24L01 with Hardware SPI2)

### NRF24L01 SPI2:
- PB13: SPI2_SCK  (was S4_1)
- PB14: SPI2_MISO (was S4_2)
- PB15: SPI2_MOSI (was S5_1)

### NRF24L01 Control:
- PB8:  NRF_CE    (was Lora_M1)
- PB9:  NRF_CSN   (was Lora_M0)
- PC13: NRF_IRQ   (Optional, new pin)

### Relocated Switches:
- PA9:  S4_1 (was USART1_TX, now free)
- PA10: S4_2 (was USART1_RX, now free)
- PB10: S5_1 (was free, now used)

## Pin Status Summary

### Freed Pins (from LoRa removal):
- PA9:  USART1_TX → now S4_1
- PA10: USART1_RX → now S4_2
- PB8:  Lora_M1   → now NRF_CE
- PB9:  Lora_M0   → now NRF_CSN

### Repurposed Pins (from switches to SPI2):
- PB13: S4_1 → SPI2_SCK
- PB14: S4_2 → SPI2_MISO
- PB15: S5_1 → SPI2_MOSI

### Newly Used Pins:
- PB10: (was free) → now S5_1
- PC13: (was free) → now NRF_IRQ (optional)

## Hardware Wiring Changes Required

1. Remove LoRa module connections from PA9, PA10, PB8, PB9
2. Move switch S4_1 from PB13 to PA9
3. Move switch S4_2 from PB14 to PA10
4. Move switch S5_1 from PB15 to PB10
5. Connect NRF24L01:
   - VCC  → 3.3V (IMPORTANT: NOT 5V!)
   - GND  → GND
   - CE   → PB8
   - CSN  → PB9
   - SCK  → PB13
   - MOSI → PB15
   - MISO → PB14
   - IRQ  → PC13 (optional)
