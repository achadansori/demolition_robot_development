# USB CDC Debug Setup untuk Transmitter Demolition Robot

Panduan ini menjelaskan cara menambahkan USB CDC (Virtual COM Port) untuk debugging via serial monitor pada project `transmitter_demolition_robot_disc_nrf24`.

## Persyaratan

- STM32CubeIDE
- STM32F407VGT6 Discovery Board
- Kabel USB (untuk port USB User/OTG)

## Langkah 1: Konfigurasi di STM32CubeMX (.ioc file)

### 1.1 Enable USB_OTG_FS

1. Buka file `.ioc` di STM32CubeIDE
2. Di **Pinout & Configuration** > **Connectivity** > **USB_OTG_FS**
3. Set Mode: **Device_Only**
4. Parameter Settings:
   - Speed: Full Speed 12MBit/s
   - Signal start of frame: Disable
   - Low power: Disable
   - VBUS sensing: Disable
   - SOF output: Disable
   - Battery charging: Disable

### 1.2 Enable USB_DEVICE

1. Di **Middleware and Software Packs** > **USB_DEVICE**
2. Set Class For FS IP: **Communication Device Class (Virtual Port Com)**
3. Parameter Settings (CDC):
   - USBD_MAX_NUM_INTERFACES: 2
   - USBD_MAX_NUM_CONFIGURATION: 1
   - USBD_MAX_STR_DESC_SIZ: 512
   - USBD_SUPPORT_USER_STRING_DESC: Disabled
   - USBD_SELF_POWERED: Enabled
   - USBD_DEBUG_LEVEL: 0
   - CDC_DATA_HS_MAX_PACKET_SIZE: 512
   - CDC_DATA_FS_MAX_PACKET_SIZE: 64
   - CDC_CMD_PACKET_SIZE: 8
   - APP_RX_DATA_SIZE: 2048
   - APP_TX_DATA_SIZE: 2048

### 1.3 Konfigurasi Clock

**PENTING:** USB membutuhkan clock 48MHz yang tepat!

1. Di **Clock Configuration**:
   - Input frequency (HSE): 8 MHz (untuk Discovery board)
   - PLL Source Mux: HSE
   - PLLM: /8
   - PLLN: x168
   - PLLP: /4 (untuk SYSCLK 84 MHz) atau /2 (untuk 168 MHz)
   - PLLQ: /7 (untuk USB clock 48 MHz)

2. Pastikan:
   - SYSCLK: 84 MHz atau 168 MHz
   - USB Clock: **48 MHz** (harus tepat!)

### 1.4 Enable NVIC Interrupt

1. Di **System Core** > **NVIC**
2. Enable: **USB On The Go FS global interrupt**

### 1.5 Generate Code

1. Klik **Project** > **Generate Code** atau tekan Alt+K
2. STM32CubeIDE akan membuat folder:
   - `USB_DEVICE/` (App, Target)
   - `Middlewares/ST/STM32_USB_Device_Library/`

## Langkah 2: Buat Debug Module

### 2.1 Buat file `Core/Inc/debug.h`

```c
#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include <stdint.h>
#include <stdarg.h>

// Initialize debug module
void Debug_Init(void);

// Print string
void Debug_Print(const char* msg);

// Printf-style formatted output
void Debug_Printf(const char* format, ...);

// Print newline
void Debug_Newline(void);

// Print hex dump
void Debug_HexDump(const char* label, const uint8_t* data, uint16_t len);

// Print TX data structure (customize for your data)
void Debug_PrintTxData(void* data);

#endif /* INC_DEBUG_H_ */
```

### 2.2 Buat file `Core/Src/debug.c`

```c
#include "debug.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

static char debug_buffer[256];

void Debug_Init(void)
{
    // USB CDC sudah di-init di MX_USB_DEVICE_Init()
}

void Debug_Print(const char* msg)
{
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void Debug_Printf(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    int len = vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);

    if (len > 0) {
        CDC_Transmit_FS((uint8_t*)debug_buffer, len);
    }
}

void Debug_Newline(void)
{
    Debug_Print("\r\n");
}

void Debug_HexDump(const char* label, const uint8_t* data, uint16_t len)
{
    Debug_Printf("%s: ", label);
    for (uint16_t i = 0; i < len; i++) {
        Debug_Printf("%02X ", data[i]);
    }
    Debug_Newline();
}

// Sesuaikan dengan struktur TX_Data Anda
void Debug_PrintTxData(void* data)
{
    // Contoh implementasi - sesuaikan dengan struktur data Anda
    uint8_t* bytes = (uint8_t*)data;
    Debug_Printf("TX Data: ");
    for (int i = 0; i < 32; i++) {  // Sesuaikan ukuran
        Debug_Printf("%02X ", bytes[i]);
    }
    Debug_Newline();
}
```

## Langkah 3: Modifikasi main.c

### 3.1 Tambahkan includes

```c
/* USER CODE BEGIN Includes */
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "debug.h"
/* USER CODE END Includes */
```

### 3.2 Inisialisasi USB

Di dalam fungsi `main()`, setelah semua `MX_xxx_Init()`:

```c
/* USER CODE BEGIN 2 */
// Init USB CDC
MX_USB_DEVICE_Init();

// Tunggu USB enumeration
HAL_Delay(1000);

// Print startup message
Debug_Printf("\r\n=== Transmitter Demolition Robot ===\r\n");
Debug_Printf("USB CDC Debug Ready!\r\n");
/* USER CODE END 2 */
```

### 3.3 Debug dalam main loop

```c
/* USER CODE BEGIN WHILE */
uint32_t debug_counter = 0;
const uint32_t DEBUG_INTERVAL = 100;  // Setiap 100 iterasi

while (1)
{
    // ... kode existing ...

    // Debug output periodik
    if (++debug_counter >= DEBUG_INTERVAL) {
        debug_counter = 0;
        Debug_Printf("Loop running, ADC: %d\r\n", adc_value);
        // Atau gunakan Debug_PrintTxData(&tx_data);
    }

    HAL_Delay(10);
    /* USER CODE END WHILE */
}
```

## Langkah 4: Pastikan HAL PCD Enabled

Di file `Core/Inc/stm32f4xx_hal_conf.h`, pastikan baris berikut TIDAK di-comment:

```c
#define HAL_PCD_MODULE_ENABLED
```

Jika masih di-comment, hapus `/* */` nya.

## Langkah 5: Build dan Flash

1. Build project: **Project** > **Build Project** (Ctrl+B)
2. Flash ke board: **Run** > **Debug** atau **Run** > **Run**

## Langkah 6: Gunakan Serial Monitor

### Windows
1. Buka Device Manager, cari "Ports (COM & LPT)"
2. Catat nomor COM port (misal COM3)
3. Gunakan:
   - PuTTY
   - Tera Term
   - Arduino Serial Monitor
   - STM32CubeIDE Terminal (Window > Show View > Terminal)

### Linux
1. Device biasanya muncul sebagai `/dev/ttyACM0`
2. Gunakan:
   ```bash
   # Dengan screen
   screen /dev/ttyACM0 115200

   # Dengan minicom
   minicom -D /dev/ttyACM0

   # Dengan cat (read only)
   cat /dev/ttyACM0
   ```

### Settings Serial
- Baud rate: 115200 (atau sesuai kebutuhan)
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow control: None

## Troubleshooting

### USB tidak terdeteksi
1. Pastikan USB clock tepat 48 MHz
2. Pastikan menggunakan port **USB User** (bukan ST-Link)
3. Coba reset board setelah flashing

### No output di serial monitor
1. Pastikan `HAL_PCD_MODULE_ENABLED` sudah di-enable
2. Pastikan `MX_USB_DEVICE_Init()` dipanggil di main
3. Tambahkan delay setelah init (1 detik)

### Build error "stm32f4xx_hal_pcd.h not found"
1. Buka .ioc file
2. Re-generate code (Alt+K)
3. Pastikan folder `Drivers/STM32F4xx_HAL_Driver/Src/` berisi:
   - stm32f4xx_hal_pcd.c
   - stm32f4xx_hal_pcd_ex.c
   - stm32f4xx_ll_usb.c

### Build error "PCD_HandleTypeDef unknown"
1. Enable `HAL_PCD_MODULE_ENABLED` di stm32f4xx_hal_conf.h

## Contoh Output

```
=== Transmitter Demolition Robot ===
USB CDC Debug Ready!
Loop running, ADC: 2048
Loop running, ADC: 2051
TX Data: 01 02 03 04 05 06 07 08 ...
NRF24 Status: 0x0E
```

## Pin Configuration (Discovery Board)

| Pin  | Function     |
|------|--------------|
| PA11 | USB_OTG_FS_DM |
| PA12 | USB_OTG_FS_DP |
| PA9  | USB_OTG_FS_VBUS (optional) |

## Catatan

- USB CDC tidak memerlukan baud rate setting di sisi MCU (virtual serial port)
- Buffer TX default 2048 bytes, sesuaikan jika perlu di ioc file
- Jangan print terlalu sering dalam loop, bisa menyebabkan buffer overflow
