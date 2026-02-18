/**
  ******************************************************************************
  * @file           : joystick.c
  * @brief          : Joystick and ADC reading implementation (DMA Mode)
  *                   STM32F407 Discovery Transmitter
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca semua input analog via DMA:
  * - 2 Joystick (masing-masing 2 axis)
  * - 2 Potentiometer (R1, R8)
  *
  * DMA Buffer Order (sesuai Rank di CubeMX):
  * [0] = joy_left_y  (PC1 - IN11)
  * [1] = joy_left_x  (PC3 - IN13)
  * [2] = joy_right_y (PA5 - IN5)
  * [3] = joy_right_x (PA7 - IN7)
  * [4] = r1          (PA0 - IN0)
  * [5] = r8          (PA2 - IN2)
  *
  * Data dikonversi dari 12-bit (0-4095) ke 8-bit (0-255) untuk efisiensi
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "joystick.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define ADC_MAX_VALUE 4095  // 12-bit ADC maximum value

/* DMA Buffer - Filled automatically by DMA */
uint16_t adc_buffer[ADC_CHANNELS] = {0};

/* Calibration center values (raw 8-bit at neutral position) */
int8_t joy_cal_offset[4] = {0, 0, 0, 0};  // Offset from 127 per axis [LX, LY, RY, RX]

/* Private variables ---------------------------------------------------------*/
static uint8_t dma_started = 0;

/**
  * @brief  Apply asymmetric calibration to joystick axis
  *         Maps raw value so that calibrated center = 127
  *         Does not reduce max range (0 and 255 still reachable)
  * @param  raw8: Raw 8-bit ADC value (0-255)
  * @param  offset: Calibration offset (raw_center - 127)
  * @retval Calibrated 8-bit value (0-255), center = 127
  */
static uint8_t apply_calibration(uint8_t raw8, int8_t offset)
{
    if (offset == 0) return raw8;

    uint8_t center = (uint8_t)(127 + offset);  // Hardware neutral in 8-bit

    if (raw8 == center) return 127;

    if (raw8 < center)
    {
        // Map [0, center] → [0, 127]
        if (center == 0) return 0;
        return (uint8_t)((uint16_t)raw8 * 127 / center);
    }
    else
    {
        // Map [center, 255] → [127, 255]
        uint8_t range = 255 - center;
        if (range == 0) return 255;
        return (uint8_t)(127 + (uint16_t)(raw8 - center) * 128 / range);
    }
}

/**
  * @brief  Initialize joystick module and start DMA
  * @retval None
  */
void Joystick_Init(void)
{
    // Start ADC DMA
    Joystick_StartDMA();
}

/**
  * @brief  Start ADC DMA conversion
  * @retval None
  */
void Joystick_StartDMA(void)
{
    if (!dma_started)
    {
        // Start ADC with DMA (Circular mode - runs continuously)
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS);
        dma_started = 1;
    }
}

/**
  * @brief  Read all joystick and potentiometer data from DMA buffer
  * @param  data: Pointer to Joystick_Data_t structure
  * @retval None
  */
void Joystick_Read(Joystick_Data_t* data)
{
    // Make sure DMA is running
    if (!dma_started)
    {
        Joystick_StartDMA();
    }

    // Read from DMA buffer and convert 12-bit to 8-bit
    // DMA Buffer Order:
    // [0] = joy_left_y  (PC1 - IN11)
    // [1] = joy_left_x  (PC3 - IN13)
    // [2] = joy_right_y (PA5 - IN5)
    // [3] = joy_right_x (PA7 - IN7)
    // [4] = r1          (PA0 - IN0)
    // [5] = r8          (PA2 - IN2)

    // Convert 12-bit to 8-bit and apply calibration offset
    data->left_x  = apply_calibration((uint8_t)(adc_buffer[0] >> 4), joy_cal_offset[0]);
    data->left_y  = apply_calibration((uint8_t)(adc_buffer[1] >> 4), joy_cal_offset[1]);
    data->right_y = apply_calibration((uint8_t)(adc_buffer[2] >> 4), joy_cal_offset[2]);
    data->right_x = apply_calibration((uint8_t)(adc_buffer[3] >> 4), joy_cal_offset[3]);
    data->r1      = (uint8_t)(adc_buffer[4] >> 4);
    data->r8      = (uint8_t)(adc_buffer[5] >> 4);
}

/**
  * @brief  Calibrate joystick by reading current position as neutral (center)
  *         Call this when joystick is at rest position
  *         After calibration, neutral = 127 (0%), full range preserved
  * @retval None
  */
void Joystick_Calibrate(void)
{
    // Read current raw 8-bit values at neutral position
    uint8_t raw[4];
    raw[0] = (uint8_t)(adc_buffer[0] >> 4);  // left_x
    raw[1] = (uint8_t)(adc_buffer[1] >> 4);  // left_y
    raw[2] = (uint8_t)(adc_buffer[2] >> 4);  // right_y
    raw[3] = (uint8_t)(adc_buffer[3] >> 4);  // right_x

    // Calculate offset: how far hardware neutral is from 127
    for (int i = 0; i < 4; i++)
    {
        joy_cal_offset[i] = (int8_t)((int16_t)raw[i] - 127);
    }
}
