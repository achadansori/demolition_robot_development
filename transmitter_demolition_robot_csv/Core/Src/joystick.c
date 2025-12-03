/**
  ******************************************************************************
  * @file           : joystick.c
  * @brief          : Joystick module implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "joystick.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef *hadc_joystick;

/* Private function prototypes -----------------------------------------------*/
static uint16_t Joystick_ReadAxis(uint32_t channel);

/**
  * @brief  Initialize joystick module
  * @param  hadc: pointer to ADC handle
  * @retval None
  */
void Joystick_Init(ADC_HandleTypeDef *hadc)
{
    hadc_joystick = hadc;
}

/**
  * @brief  Read all joystick axes and buttons
  * @param  data: pointer to Joystick_Data_t structure
  * @retval None
  */
void Joystick_Read(Joystick_Data_t *data)
{
    if (data == NULL || hadc_joystick == NULL)
    {
        return;
    }

    // Read Left Joystick Analog
    data->left_x = Joystick_ReadAxis(JOY_LEFT_X_CHANNEL);   // PA3
    data->left_y = Joystick_ReadAxis(JOY_LEFT_Y_CHANNEL);   // PA2

    // Read Left Joystick Buttons (Pull-down)
    data->left_btn1 = HAL_GPIO_ReadPin(JOY_LEFT_BTN1_GPIO_Port, JOY_LEFT_BTN1_Pin);  // PA5
    data->left_btn2 = HAL_GPIO_ReadPin(JOY_LEFT_BTN2_GPIO_Port, JOY_LEFT_BTN2_Pin);  // PA4

    // Read Right Joystick Analog
    data->right_x = Joystick_ReadAxis(JOY_RIGHT_X_CHANNEL); // PA7
    data->right_y = Joystick_ReadAxis(JOY_RIGHT_Y_CHANNEL); // PA6

    // Read Right Joystick Buttons (Pull-down)
    data->right_btn1 = HAL_GPIO_ReadPin(JOY_RIGHT_BTN1_GPIO_Port, JOY_RIGHT_BTN1_Pin);  // PB1
    data->right_btn2 = HAL_GPIO_ReadPin(JOY_RIGHT_BTN2_GPIO_Port, JOY_RIGHT_BTN2_Pin);  // PB0
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Read single axis from ADC
  * @param  channel: ADC channel to read
  * @retval ADC value (0-4095)
  */
static uint16_t Joystick_ReadAxis(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t adc_value = 0;

    // Configure ADC channel
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;  // Faster (was 84 cycles)

    if (HAL_ADC_ConfigChannel(hadc_joystick, &sConfig) != HAL_OK)
    {
        return 0;
    }

    // Start ADC conversion
    if (HAL_ADC_Start(hadc_joystick) == HAL_OK)
    {
        // Wait for conversion to complete
        if (HAL_ADC_PollForConversion(hadc_joystick, 100) == HAL_OK)
        {
            adc_value = HAL_ADC_GetValue(hadc_joystick);
        }
        HAL_ADC_Stop(hadc_joystick);
    }

    return adc_value;
}
