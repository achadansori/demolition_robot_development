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

    // Single line format - easy to read, no screen corruption
    len = snprintf(usb_buffer, USB_TX_BUFFER_SIZE,
        "JL:%03d,%03d,%d,%d JR:%03d,%03d,%d,%d POT:R8=%03d,R1=%03d SW:S0=%d,S1=%d%d,S2=%d%d,S4=%d%d,S5=%d%d\r\n",
        data->joystick.left_x,
        data->joystick.left_y,
        data->switches.joy_left_btn1,
        data->switches.joy_left_btn2,
        data->joystick.right_x,
        data->joystick.right_y,
        data->switches.joy_right_btn1,
        data->switches.joy_right_btn2,
        data->joystick.r8,
        data->joystick.r1,
        data->switches.s0,
        data->switches.s1_1, data->switches.s1_2,
        data->switches.s2_1, data->switches.s2_2,
        data->switches.s4_1, data->switches.s4_2,
        data->switches.s5_1, data->switches.s5_2);

    CDC_Transmit_FS((uint8_t*)usb_buffer, len);
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

    // Simple hex format on single line
    offset = snprintf(usb_buffer, USB_TX_BUFFER_SIZE, "HEX:");
    for (uint16_t i = 0; i < size; i++)
    {
        offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset,
            "%02X", data[i]);
    }
    offset += snprintf(usb_buffer + offset, USB_TX_BUFFER_SIZE - offset, "\r\n");

    CDC_Transmit_FS((uint8_t*)usb_buffer, offset);
}
