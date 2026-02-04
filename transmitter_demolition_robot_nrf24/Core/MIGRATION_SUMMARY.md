# Migration Summary: LoRa to NRF24L01+ (Hardware SPI2)

## ✅ COMPLETED TASKS

### 1. Pin Configuration Changes (.ioc file)
- **Removed USART1** (PA9, PA10 no longer used for LoRa)
- **Added SPI2** on PB13/PB14/PB15
- **Relocated switches:**
  - S4_1: PB13 → PA9
  - S4_2: PB14 → PA10
  - S5_1: PB15 → PB10
- **Repurposed NRF24 control pins:**
  - PB8: Lora_M1 → NRF_CE
  - PB9: Lora_M0 → NRF_CSN
- **Added NRF24 IRQ:** PC13 (optional interrupt pin)

### 2. Header Files Updated
- **main.h**: Updated with new pin definitions for switches and NRF24
- **var.h**: Added `Var_GetNRF24Data()` function prototype
- **Created nrf24.h**: NRF24L01+ driver header
- **Created spi.h**: SPI2 initialization header

### 3. Source Files Created/Modified
- **Created nrf24.c**: Full NRF24L01+ driver implementation with:
  - Hardware SPI2 support
  - Auto-ACK enabled
  - 2Mbps data rate
  - 0dBm transmit power
  - Channel 76 (2476 MHz)
  - Binary payload format (8 bytes)
- **Created spi.c**: SPI2 initialization (3.75 MHz)
- **Modified main.c**:
  - Replaced LoRa includes with NRF24
  - Added SPI2 initialization
  - Replaced LoRa_Init() with NRF24_Init()
  - Replaced LoRa transmission with NRF24_TransmitData()
  - Updated welcome messages

### 4. Disabled Files
- **Renamed to .bak** (won't compile):
  - Core/Inc/lora.h.bak
  - Core/Src/lora.c.bak
  - Core/Inc/usart.h.bak
  - Core/Src/usart.c.bak

### 5. Documentation Created
- **PIN_MIGRATION_NOTES.md**: Detailed pin migration guide
- **transmitter_demolition_robot_nrf24.ioc.backup**: Backup of original .ioc file

## 📋 HARDWARE WIRING CHANGES REQUIRED

### Remove LoRa Connections:
1. Disconnect LoRa module from PA9 (TX)
2. Disconnect LoRa module from PA10 (RX)
3. Disconnect LoRa module from PB8 (M1)
4. Disconnect LoRa module from PB9 (M0)

### Move Switch Connections:
1. **S4_1**: Move from PB13 to PA9
2. **S4_2**: Move from PB14 to PA10
3. **S5_1**: Move from PB15 to PB10

### Connect NRF24L01+:
```
NRF24L01+ Pin    →   STM32F401CEU6
─────────────────────────────────────
VCC (3.3V only!) →   3.3V
GND              →   GND
CE               →   PB8
CSN              →   PB9
SCK              →   PB13
MOSI             →   PB15
MISO             →   PB14
IRQ (optional)   →   PC13
```

**⚠️ CRITICAL: NRF24L01+ is 3.3V ONLY! Do NOT connect to 5V!**

## 🔧 NEXT STEPS FOR USER

### 1. Open Project in STM32CubeIDE
```bash
File → Open Projects from File System
Select: transmitter_demolition_robot_nrf24
```

### 2. Regenerate Code from .ioc (Optional but Recommended)
```
1. Open transmitter_demolition_robot_nrf24.ioc in STM32CubeMX
2. Click "Generate Code" button
3. Keep user code sections (our custom code is protected)
```

### 3. Build Project
```
Project → Build Project (Ctrl+B)
```

### 4. Fix Any Compilation Errors
Common issues and fixes:
- If missing hspi2: Ensure spi.c is included in build
- If NRF24_Data_t undefined: Check nrf24.h is included
- If link errors: Check all .c files are in build path

### 5. Test on Hardware
1. Wire hardware according to section above
2. Flash firmware to STM32F401CEU6
3. Open USB serial monitor (115200 baud)
4. Check for initialization messages:
   ```
   DEMOLITION ROBOT TRANSMITTER (NRF24)
   ========================================
   Configuring NRF24L01+...
   NRF24L01+ configured successfully!
   NRF24L01+ module detected!
   ```

### 6. Verify NRF24 Communication
- Check that transmit LED blinks (if available)
- Monitor status via USB serial output
- Test with receiver to confirm data transmission

## 📊 TECHNICAL SPECIFICATIONS

### NRF24L01+ Configuration:
- **Frequency**: 2476 MHz (Channel 76)
- **Data Rate**: 2 Mbps
- **TX Power**: 0 dBm (maximum)
- **Auto-ACK**: Enabled
- **Auto-Retransmit**: 500µs delay, 15 retries
- **Payload Size**: 8 bytes (binary format)
- **Address**: 0xE7E7E7E7E7 (5 bytes)
- **CRC**: 2 bytes

### SPI2 Configuration:
- **Mode**: Master
- **Speed**: 3.75 MHz (APB1 30MHz / prescaler 8)
- **Data Size**: 8-bit
- **Clock Polarity**: Low
- **Clock Phase**: 1 Edge
- **NSS**: Software (manual CSN control)

### Binary Packet Format (8 bytes):
```
Byte 0: joy_left_x  (0-255)
Byte 1: joy_left_y  (0-255)
Byte 2: joy_right_x (0-255)
Byte 3: joy_right_y (0-255)
Byte 4: r8          (0-255)
Byte 5: r1/battery  (0-255)
Byte 6-7: switches  (bit-packed, 14 bits)
```

## 🔍 TROUBLESHOOTING

### Issue: NRF24 not detected
**Solutions:**
1. Check 3.3V power supply (must be stable)
2. Verify SPI wiring (especially SCK, MOSI, MISO)
3. Check CE and CSN connections
4. Try adding 10µF capacitor near NRF24 VCC pin
5. Use short wires (<10cm) to reduce noise

### Issue: Transmission fails (MAX_RT)
**Solutions:**
1. Check receiver is powered and configured
2. Verify addresses match on TX and RX
3. Check antenna is connected
4. Reduce distance between TX and RX
5. Check for RF interference

### Issue: Compilation errors
**Solutions:**
1. Clean and rebuild project
2. Check all new files are in build path
3. Verify .ioc regeneration didn't overwrite custom code
4. Check USER CODE sections are preserved

## 📁 PROJECT FILES STRUCTURE

```
transmitter_demolition_robot_nrf24/
├── Core/
│   ├── Inc/
│   │   ├── main.h (✓ UPDATED)
│   │   ├── nrf24.h (✓ NEW)
│   │   ├── spi.h (✓ NEW)
│   │   ├── var.h (✓ UPDATED)
│   │   ├── lora.h.bak (✗ DISABLED)
│   │   └── usart.h.bak (✗ DISABLED)
│   └── Src/
│       ├── main.c (✓ UPDATED)
│       ├── nrf24.c (✓ NEW)
│       ├── spi.c (✓ NEW)
│       ├── lora.c.bak (✗ DISABLED)
│       └── usart.c.bak (✗ DISABLED)
├── transmitter_demolition_robot_nrf24.ioc (✓ UPDATED)
├── transmitter_demolition_robot_nrf24.ioc.backup (backup)
├── PIN_MIGRATION_NOTES.md (documentation)
└── MIGRATION_SUMMARY.md (this file)
```

## 🎯 COMPATIBILITY NOTES

- Binary packet format is **SAME** as LoRa version (8 bytes)
- Receiver must also be updated to NRF24L01+
- Channel and address must match on TX and RX
- Data structure (NRF24_Data_t) is compatible with LoRa_ReceivedData_t

## ✅ MIGRATION CHECKLIST

- [x] Pin configuration updated (.ioc file)
- [x] NRF24 driver created (nrf24.h/c)
- [x] SPI2 driver created (spi.h/c)
- [x] Main code updated (main.c)
- [x] LoRa code disabled (.bak files)
- [ ] **Hardware rewiring (USER ACTION REQUIRED)**
- [ ] **Test compilation (USER ACTION REQUIRED)**
- [ ] **Flash and test on hardware (USER ACTION REQUIRED)**

---

**Ready for compilation and hardware testing!**

For questions or issues, refer to:
- PIN_MIGRATION_NOTES.md (hardware wiring details)
- nrf24.h (driver API documentation)
- This file (overall migration summary)
