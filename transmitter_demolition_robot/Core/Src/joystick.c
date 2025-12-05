/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : joystick.c
  * @brief          : Joystick and ADC reading implementation
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca semua input analog:
  * - 2 Joystick (masing-masing 2 axis)
  * - 2 Potentiometer (R8, R1)
  *
  * Data dikonversi dari 12-bit (0-4095) ke 8-bit (0-255) untuk efisiensi
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "joystick.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ADC_MAX_VALUE 4095  // 12-bit ADC maximum value
#define ADC_TIMEOUT   100   // ADC conversion timeout in milliseconds

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize joystick module
  * @retval None
  */
void Joystick_Init(void)
{
    // ADC sudah diinisialisasi di MX_ADC1_Init()
    // STM32F4 tidak memerlukan kalibrasi ADC (hanya STM32F3/L4)
}

/**
  * @brief  Read specific ADC channel
  * @param  channel: ADC channel number
  * @retval 16-bit ADC value (0-4095)
  */
uint16_t Joystick_ReadChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t adc_value = 0;

    // Configure channel
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK)
    {
        // Start conversion
        HAL_ADC_Start(&hadc1);

        // Wait for conversion to complete
        if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT) == HAL_OK)
        {
            // Read value
            adc_value = HAL_ADC_GetValue(&hadc1);
        }

        // Stop ADC
        HAL_ADC_Stop(&hadc1);
    }

    return adc_value;
}

/**
  * @brief  Read all joystick and potentiometer data
  * @param  data: Pointer to Joystick_Data_t structure
  * @retval None
  */
void Joystick_Read(Joystick_Data_t* data)
{
    uint16_t adc_value;

    // Read Left Joystick X (PA2 - ADC_CHANNEL_2)
    adc_value = Joystick_ReadChannel(ADC_CHANNEL_2);
    data->left_x = (uint8_t)(adc_value >> 4); // Convert 12-bit to 8-bit (divide by 16)

    // Read Left Joystick Y (PA3 - ADC_CHANNEL_3)
    adc_value = Joystick_ReadChannel(ADC_CHANNEL_3);
    data->left_y = (uint8_t)(adc_value >> 4);

    // Read Right Joystick Y (PA6 - ADC_CHANNEL_6) - SWAPPED WITH X
    adc_value = Joystick_ReadChannel(ADC_CHANNEL_6);
    data->right_y = (uint8_t)(adc_value >> 4);

    // Read Right Joystick X (PA7 - ADC_CHANNEL_7) - SWAPPED WITH Y
    adc_value = Joystick_ReadChannel(ADC_CHANNEL_7);
    data->right_x = (uint8_t)(adc_value >> 4);

    // Read R1 Potentiometer (PA0 - ADC_CHANNEL_0) - SWAPPED WITH R8
    adc_value = Joystick_ReadChannel(ADC_CHANNEL_0);
    data->r1 = (uint8_t)(adc_value >> 4);

    // Read R8 Potentiometer (PA1 - ADC_CHANNEL_1) - SWAPPED WITH R1
    adc_value = Joystick_ReadChannel(ADC_CHANNEL_1);
    data->r8 = (uint8_t)(adc_value >> 4);
}
