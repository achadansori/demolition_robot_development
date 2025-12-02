/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb.c
  * @brief          : USB CDC printing implementation
  ******************************************************************************
  * @attention
  *
  * Module khusus untuk menampilkan data via USB CDC (Serial Monitor)
  * Semua fungsi print/printf hanya ada di module ini (OOP principle)
  *
  * Functions:
  * - USB_Print: Print string biasa
  * - USB_PrintData: Print data terstruktur dalam format readable
  * - USB_PrintBinary: Print data dalam format binary (0b10101010)
  * - USB_PrintHex: Print data dalam format hexadecimal
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usb.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define USB_TX_BUFFER_SIZE 1024
#define BAR_LENGTH 20

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static char usb_buffer[USB_TX_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
static void USB_PrintProgressBar(uint8_t value, char* buffer, int buffer_size);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Create progress bar for analog values
  * @param  value: Value 0-255
  * @param  buffer: Output buffer
  * @param  buffer_size: Size of buffer
  * @retval None
  */
static void USB_PrintProgressBar(uint8_t value, char* buffer, int buffer_size)
{
    int filled = (value * BAR_LENGTH) / 255;
    int offset = 0;

    offset += snprintf(buffer + offset, buffer_size - offset, "[");
    for (int i = 0; i < BAR_LENGTH; i++) {
        if (i < filled) {
            offset += snprintf(buffer + offset, buffer_size - offset, "=");
        } else if (i == filled) {
            offset += snprintf(buffer + offset, buffer_size - offset, ">");
        } else {
            offset += snprintf(buffer + offset, buffer_size - offset, " ");
        }
    }
    offset += snprintf(buffer + offset, buffer_size - offset, "]");
}

/**
  * @brief  Initialize USB module
  * @retval None
  */
void USB_Init(void)
{
    // USB Device sudah diinisialisasi di MX_USB_DEVICE_Init()
    // Delay untuk stabilisasi USB
    HAL_Delay(100);
}

/**
  * @brief  Print string via USB CDC
  * @param  str: String to print
  * @retval None
  */
void USB_Print(const char* str)
{
    uint16_t len = strlen(str);
    CDC_Transmit_FS((uint8_t*)str, len);
    HAL_Delay(10); // Small delay for USB transmission
}

/**
  * @brief  Print transmitter data in readable format
  * @param  data: Pointer to Transmitter_Data_t structure
  * @retval None
  */
void USB_PrintData(Transmitter_Data_t* data)
{
    int len = 0;
    int offset = 0;
    char bar_lx[30], bar_ly[30], bar_rx[30], bar_ry[30], bar_r8[30], bar_r1[30];

    // Prepare all progress bars first
    USB_PrintProgressBar(data->joystick.left_x, bar_lx, sizeof(bar_lx));
    USB_PrintProgressBar(data->joystick.left_y, bar_ly, sizeof(bar_ly));
    USB_PrintProgressBar(data->joystick.right_x, bar_rx, sizeof(bar_rx));
    USB_PrintProgressBar(data->joystick.right_y, bar_ry, sizeof(bar_ry));
    USB_PrintProgressBar(data->joystick.r8, bar_r8, sizeof(bar_r8));
    USB_PrintProgressBar(data->joystick.r1, bar_r1, sizeof(bar_r1));

    // Build complete display in buffer (single transmission)
    offset = 0;

    // Clear screen and header
    offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset,
        "\033[2J\033[H"
        "╔════════════════════════════════════════════════════════╗\r\n"
        "║      DEMOLITION ROBOT TRANSMITTER - LIVE DATA          ║\r\n"
        "╚════════════════════════════════════════════════════════╝\r\n\r\n");

    // Joystick Left
    offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset,
        "┌─ JOYSTICK LEFT ────────────────────────────────────────┐\r\n"
        "│  X-Axis: %3d  %s │\r\n"
        "│  Y-Axis: %3d  %s │\r\n"
        "│  Btn1: %s   Btn2: %s                                │\r\n"
        "└────────────────────────────────────────────────────────┘\r\n\r\n",
        data->joystick.left_x, bar_lx,
        data->joystick.left_y, bar_ly,
        data->switches.joy_left_btn1 ? "[ON]" : "[--]",
        data->switches.joy_left_btn2 ? "[ON]" : "[--]");

    // Joystick Right
    offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset,
        "┌─ JOYSTICK RIGHT ───────────────────────────────────────┐\r\n"
        "│  X-Axis: %3d  %s │\r\n"
        "│  Y-Axis: %3d  %s │\r\n"
        "│  Btn1: %s   Btn2: %s                                │\r\n"
        "└────────────────────────────────────────────────────────┘\r\n\r\n",
        data->joystick.right_x, bar_rx,
        data->joystick.right_y, bar_ry,
        data->switches.joy_right_btn1 ? "[ON]" : "[--]",
        data->switches.joy_right_btn2 ? "[ON]" : "[--]");

    // Potentiometers
    offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset,
        "┌─ POTENTIOMETERS ───────────────────────────────────────┐\r\n"
        "│  R8:  %3d     %s │\r\n"
        "│  R1:  %3d     %s │\r\n"
        "└────────────────────────────────────────────────────────┘\r\n\r\n",
        data->joystick.r8, bar_r8,
        data->joystick.r1, bar_r1);

    // Switches
    offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset,
        "┌─ SWITCHES ─────────────────────────────────────────────┐\r\n"
        "│  S0: %s  S1: %s,%s  S2: %s,%s  S4: %s,%s  S5: %s,%s  │\r\n"
        "└────────────────────────────────────────────────────────┘\r\n\r\n",
        data->switches.s0 ? "[1]" : "[0]",
        data->switches.s1_1 ? "[1]" : "[0]", data->switches.s1_2 ? "[1]" : "[0]",
        data->switches.s2_1 ? "[1]" : "[0]", data->switches.s2_2 ? "[1]" : "[0]",
        data->switches.s4_1 ? "[1]" : "[0]", data->switches.s4_2 ? "[1]" : "[0]",
        data->switches.s5_1 ? "[1]" : "[0]", data->switches.s5_2 ? "[1]" : "[0]");

    // Single transmission of complete display
    CDC_Transmit_FS((uint8_t*)usb_buffer, offset);
    HAL_Delay(50);
}

/**
  * @brief  Print data in binary format
  * @param  data: Pointer to data buffer
  * @param  size: Size of data in bytes
  * @retval None
  */
void USB_PrintBinary(uint8_t* data, uint16_t size)
{
    int len = 0;
    int offset = 0;

    len = snprintf(usb_buffer, USB_TX_BUFFER_SIZE,
        "\r\n┌─ BINARY BREAKDOWN ─────────────────────────────────────┐\r\n");
    CDC_Transmit_FS((uint8_t*)usb_buffer, len);
    HAL_Delay(5);

    // Print each byte in binary
    for (uint16_t i = 0; i < size; i++)
    {
        offset = 0;
        offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset,
            "│  [%d] 0b", i);

        // Print each bit
        for (int8_t bit = 7; bit >= 0; bit--)
        {
            offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset,
                "%d", (data[i] >> bit) & 0x01);
            if (bit == 4) {
                offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset, " ");
            }
        }

        offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset,
            "  (0x%02X = %3d)               │\r\n", data[i], data[i]);

        CDC_Transmit_FS((uint8_t*)usb_buffer, offset);
        HAL_Delay(5);
    }

    len = snprintf(usb_buffer, USB_TX_BUFFER_SIZE,
        "└────────────────────────────────────────────────────────┘\r\n");
    CDC_Transmit_FS((uint8_t*)usb_buffer, len);
    HAL_Delay(5);
}

/**
  * @brief  Print data in hexadecimal format
  * @param  data: Pointer to data buffer
  * @param  size: Size of data in bytes
  * @retval None
  */
void USB_PrintHex(uint8_t* data, uint16_t size)
{
    int len = 0;
    int offset = 0;

    len = snprintf(usb_buffer, USB_TX_BUFFER_SIZE,
        "┌─ BINARY DATA PACKET (%d bytes for LoRa) ───────────────┐\r\n"
        "│  HEX: ", size);
    CDC_Transmit_FS((uint8_t*)usb_buffer, len);
    HAL_Delay(5);

    // Print hex string
    offset = 0;
    for (uint16_t i = 0; i < size; i++)
    {
        offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset,
            "%02X ", data[i]);
    }

    // Pad to align
    while (offset < 35) {
        offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset, " ");
    }
    offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset, "│\r\n");

    CDC_Transmit_FS((uint8_t*)usb_buffer, offset);
    HAL_Delay(5);

    len = snprintf(usb_buffer, USB_TX_BUFFER_SIZE,
        "└────────────────────────────────────────────────────────┘\r\n");
    CDC_Transmit_FS((uint8_t*)usb_buffer, len);
    HAL_Delay(5);
}
