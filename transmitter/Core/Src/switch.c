/**
  ******************************************************************************
  * @file           : switch.c
  * @brief          : Switch module implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "switch.h"
#include "main.h"

/**
  * @brief  Initialize switch module
  * @note   GPIO initialization is handled by MX_GPIO_Init()
  * @retval None
  */
void Switch_Init(void)
{
    // GPIO initialization is already handled by CubeMX generated code
    // This function is kept for future expansion if needed
}

/**
  * @brief  Read all switch states
  * @param  data: pointer to Switch_Data_t structure
  * @retval None
  */
void Switch_Read(Switch_Data_t *data)
{
    if (data == NULL)
    {
        return;
    }

    // Read all switch states (Pull-down configuration: 1=pressed, 0=released)
    data->s0 = HAL_GPIO_ReadPin(S0_GPIO_Port, S0_Pin);          // PB12
    data->s1_1 = HAL_GPIO_ReadPin(S1_1_GPIO_Port, S1_1_Pin);    // PB5
    data->s1_2 = HAL_GPIO_ReadPin(S1_2_GPIO_Port, S1_2_Pin);    // PB4
    data->s2_1 = HAL_GPIO_ReadPin(S2_1_GPIO_Port, S2_1_Pin);    // PB3
    data->s2_2 = HAL_GPIO_ReadPin(S2_2_GPIO_Port, S2_2_Pin);    // PA15
    data->s4_1 = HAL_GPIO_ReadPin(S4_1_GPIO_Port, S4_1_Pin);    // PB14
    data->s4_2 = HAL_GPIO_ReadPin(S4_2_GPIO_Port, S4_2_Pin);    // PB13
    data->s5_1 = HAL_GPIO_ReadPin(S5_1_GPIO_Port, S5_1_Pin);    // PA8
    data->s5_2 = HAL_GPIO_ReadPin(S5_2_GPIO_Port, S5_2_Pin);    // PB15
}
