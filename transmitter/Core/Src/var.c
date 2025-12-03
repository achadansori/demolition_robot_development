/**
  ******************************************************************************
  * @file           : var.c
  * @brief          : Variable Resistor/Potentiometer module implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "var.h"

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef *hadc_var;

/* Private function prototypes -----------------------------------------------*/
static uint16_t Var_ReadChannel(uint32_t channel);

/**
  * @brief  Initialize variable resistor module
  * @param  hadc: pointer to ADC handle
  * @retval None
  */
void Var_Init(ADC_HandleTypeDef *hadc)
{
    hadc_var = hadc;
}

/**
  * @brief  Read all variable resistor values
  * @param  data: pointer to Var_Data_t structure
  * @retval None
  */
void Var_Read(Var_Data_t *data)
{
    if (data == NULL || hadc_var == NULL)
    {
        return;
    }

    // Read R1 (PA1)
    data->r1 = Var_ReadChannel(VAR_R1_CHANNEL);

    // Read R8 (PA0)
    data->r8 = Var_ReadChannel(VAR_R8_CHANNEL);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Read single channel from ADC
  * @param  channel: ADC channel to read
  * @retval ADC value (0-4095)
  */
static uint16_t Var_ReadChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t adc_value = 0;

    // Configure ADC channel
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;  // Faster (was 84 cycles)

    if (HAL_ADC_ConfigChannel(hadc_var, &sConfig) != HAL_OK)
    {
        return 0;
    }

    // Start ADC conversion
    if (HAL_ADC_Start(hadc_var) == HAL_OK)
    {
        // Wait for conversion to complete
        if (HAL_ADC_PollForConversion(hadc_var, 100) == HAL_OK)
        {
            adc_value = HAL_ADC_GetValue(hadc_var);
        }
        HAL_ADC_Stop(hadc_var);
    }

    return adc_value;
}
