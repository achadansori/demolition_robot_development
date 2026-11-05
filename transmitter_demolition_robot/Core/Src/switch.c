/**
  ******************************************************************************
  * @file           : switch.c
  * @brief          : Switch and button reading implementation
  *                   STM32F407 Discovery Transmitter
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca semua input digital:
  * - 4 Joystick buttons (2 per joystick)
  * - 9 Additional switches (S0, S1_1, S1_2, S2_1, S2_2, S4_1, S4_2, S5_1, S5_2)
  *
  * Data dikemas dalam 2 bytes menggunakan bit packing
  *
  * Pin Configuration (STM32F407 Discovery):
  * - JOY_LEFT_BTN1:  PA3
  * - JOY_LEFT_BTN2:  PA1
  * - JOY_RIGHT_BTN1: PB1
  * - JOY_RIGHT_BTN2: PC5
  * - S0:             PB0
  * - S1_1:           PE3
  * - S1_2:           PE5
  * - S2_1:           PE1
  * - S2_2:           PB8
  * - S4_1:           PD6
  * - S4_2:           PB3
  * - S5_1:           PB5
  * - S5_2:           PB7
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "switch.h"
#include "main.h"

/**
  * @brief  Initialize switch module
  * @retval None
  */
void Switch_Init(void)
{
    // GPIO sudah diinisialisasi di MX_GPIO_Init()
    // Tidak ada inisialisasi tambahan yang diperlukan
}

/**
  * @brief  Read specific GPIO pin state (ACTIVE HIGH with Pull-down)
  * @param  port: GPIO port (GPIOA, GPIOB, etc)
  * @param  pin: GPIO pin number
  * @retval Pin state (true = pressed/ON, false = released/OFF)
  *         With Pull-down: pressed = HIGH = return 1, released = LOW = return 0
  */
bool Switch_ReadPin(GPIO_TypeDef* port, uint16_t pin)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);
    // Direct logic for Pull-down configuration (active HIGH)
    // When pressed: pin is HIGH (connected to VCC) → return 1 (true)
    // When released: pin is LOW (pulled down) → return 0 (false)
    return (state == GPIO_PIN_SET);
}

/**
  * @brief  Read all switch and button states
  * @param  data: Pointer to Switch_Data_t structure
  * @retval None
  */
void Switch_Read(Switch_Data_t* data)
{
    // Read Joystick Buttons
    data->joy_left_btn1  = Switch_ReadPin(JOY_LEFT_BTN1_GPIO_Port, JOY_LEFT_BTN1_Pin);
    data->joy_left_btn2  = Switch_ReadPin(JOY_LEFT_BTN2_GPIO_Port, JOY_LEFT_BTN2_Pin);
    data->joy_right_btn1 = Switch_ReadPin(JOY_RIGHT_BTN1_GPIO_Port, JOY_RIGHT_BTN1_Pin);
    data->joy_right_btn2 = Switch_ReadPin(JOY_RIGHT_BTN2_GPIO_Port, JOY_RIGHT_BTN2_Pin);

    // Read Additional Switches
    data->s0   = Switch_ReadPin(S0_GPIO_Port, S0_Pin);
    data->s1_1 = Switch_ReadPin(S1_1_GPIO_Port, S1_1_Pin);
    data->s1_2 = Switch_ReadPin(S1_2_GPIO_Port, S1_2_Pin);
    data->s2_1 = Switch_ReadPin(S2_1_GPIO_Port, S2_1_Pin);
    data->s2_2 = Switch_ReadPin(S2_2_GPIO_Port, S2_2_Pin);
    data->s4_1 = Switch_ReadPin(S4_1_GPIO_Port, S4_1_Pin);
    data->s4_2 = Switch_ReadPin(S4_2_GPIO_Port, S4_2_Pin);
    data->s5_1 = Switch_ReadPin(S5_1_GPIO_Port, S5_1_Pin);
    data->s5_2 = Switch_ReadPin(S5_2_GPIO_Port, S5_2_Pin);

    // Reserved bits set to 0
    data->reserved = 0;
}
