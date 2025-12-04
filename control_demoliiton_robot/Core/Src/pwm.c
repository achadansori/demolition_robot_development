/**
  ******************************************************************************
  * @file           : pwm.c
  * @brief          : PWM Control Implementation for 20 Channels
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pwm.h"
#include "tim.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define PWM_FREQUENCY       10000   // 10kHz for TIP122 transistor switching (visible on oscilloscope)
#define PWM_PERIOD          100     // 100us period (10kHz)
#define PWM_MIN_PULSE       0       // 0% duty = 0V
#define PWM_MAX_PULSE       100     // 100% duty = 3.3V average

/* Private variables ---------------------------------------------------------*/
static uint8_t pwm_duty[PWM_CHANNEL_COUNT] = {0};

/* Private function prototypes -----------------------------------------------*/
static void PWM_ConfigureGPIO(void);
static void PWM_ConfigureTimers(void);
static void PWM_SetChannelPulse(PWM_Channel_t channel, uint16_t pulse_width);

/**
  * @brief  Initialize all PWM channels
  * @retval None
  */
void PWM_Init(void)
{
    // Configure GPIO pins
    PWM_ConfigureGPIO();

    // Configure Timers
    PWM_ConfigureTimers();

    // Initialize all channels to 0%
    PWM_StopAll();
}

/**
  * @brief  Configure GPIO pins for PWM output
  * @retval None
  */
static void PWM_ConfigureGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    // Configure TIM1 pins (PE9, PE11, PE13, PE14) - AF1
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // Configure TIM2 pins (PA0, PA1, PA2, PA3) - AF1
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure TIM3 pins (PB4, PB5, PB0, PB1) - AF2
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure TIM4 pins (PD12, PD13, PD14, PD15) - AF2
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // Configure TIM8 pins (PC6, PC7, PC8, PC9) - AF3
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief  Configure Timer peripherals for PWM
  * @retval None
  */
static void PWM_ConfigureTimers(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    // Enable timer clocks
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    // Common timer configuration (10kHz PWM for TIP122 switching)
    // Assuming system clock is 84MHz for APB2 timers (TIM1, TIM8)
    // and 42MHz for APB1 timers (TIM2, TIM3, TIM4)

    // Configure TIM1 (84MHz / 84 = 1MHz, 1MHz / 10kHz = 100)
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 83;  // 84MHz / 84 = 1MHz
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_PERIOD - 1;  // 100 - 1 = 99
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(&htim1);
    HAL_TIM_PWM_Init(&htim1);

    // Configure TIM2, TIM3, TIM4 (42MHz / 42 = 1MHz, 1MHz / 10kHz = 100)
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 41;  // 42MHz / 42 = 1MHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = PWM_PERIOD - 1;  // 100 - 1 = 99
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);
    HAL_TIM_PWM_Init(&htim2);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 41;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = PWM_PERIOD - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim3);
    HAL_TIM_PWM_Init(&htim3);

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 41;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = PWM_PERIOD - 1;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim4);
    HAL_TIM_PWM_Init(&htim4);

    // Configure TIM8 (84MHz / 84 = 1MHz, 1MHz / 10kHz = 100)
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 83;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = PWM_PERIOD - 1;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(&htim8);
    HAL_TIM_PWM_Init(&htim8);

    // Configure PWM channels (common settings)
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    // Configure all channels
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);

    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);

    // Start all PWM channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

/**
  * @brief  Set PWM duty cycle (0-100%)
  * @param  channel: PWM channel
  * @param  duty_percent: Duty cycle in percentage (0-100)
  * @retval None
  *
  * @note   For TIP122 transistor switching:
  *         - 0% duty   = 0V average (TIP122 OFF, no current to base)
  *         - 50% duty  = ~1.65V average (TIP122 half power)
  *         - 100% duty = 3.3V average (TIP122 fully ON)
  *         - PWM @ 10kHz will be clearly visible on oscilloscope
  */
void PWM_SetDutyCycle(PWM_Channel_t channel, uint8_t duty_percent)
{
    if (channel >= PWM_CHANNEL_COUNT) return;
    if (duty_percent > 100) duty_percent = 100;

    // Store duty cycle
    pwm_duty[channel] = duty_percent;

    // Calculate pulse width directly from duty cycle
    // duty_percent 0-100 maps directly to CCR value 0-100
    uint16_t pulse_width = duty_percent;

    // Set channel pulse
    PWM_SetChannelPulse(channel, pulse_width);
}

/**
  * @brief  Get current PWM duty cycle
  * @param  channel: PWM channel
  * @retval Duty cycle percentage (0-100)
  */
uint8_t PWM_GetDutyCycle(PWM_Channel_t channel)
{
    if (channel >= PWM_CHANNEL_COUNT) return 0;
    return pwm_duty[channel];
}

/**
  * @brief  Stop PWM on specific channel (set to 0%)
  * @param  channel: PWM channel
  * @retval None
  */
void PWM_Stop(PWM_Channel_t channel)
{
    PWM_SetDutyCycle(channel, 0);
}

/**
  * @brief  Stop all PWM channels
  * @retval None
  */
void PWM_StopAll(void)
{
    for (uint8_t i = 0; i < PWM_CHANNEL_COUNT; i++)
    {
        PWM_Stop((PWM_Channel_t)i);
    }
}

/**
  * @brief  Set pulse width for specific channel
  * @param  channel: PWM channel
  * @param  pulse_width: Pulse width in microseconds
  * @retval None
  */
static void PWM_SetChannelPulse(PWM_Channel_t channel, uint16_t pulse_width)
{
    switch (channel)
    {
        // TIM8 channels
        case PWM_1_CYLINDER_1_OUT:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse_width);
            break;
        case PWM_2_CYLINDER_1_IN:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pulse_width);
            break;
        case PWM_3_CYLINDER_2_OUT:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pulse_width);
            break;
        case PWM_10_TOOL_2:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pulse_width);
            break;

        // TIM2 channels
        case PWM_5_CYLINDER_3_OUT:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_width);
            break;
        case PWM_7_CYLINDER_4_OUT:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_width);
            break;
        case PWM_8_CYLINDER_4_IN:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width);
            break;
        case PWM_15_OUTRIGGER_BACK_UP:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse_width);
            break;

        // TIM3 channels
        case PWM_4_CYLINDER_2_IN:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_width);
            break;
        case PWM_6_CYLINDER_3_IN:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_width);
            break;
        case PWM_9_TOOL_1:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_width);
            break;
        case PWM_14_OUTRIGGER_FRONT_DOWN:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_width);
            break;

        // TIM4 channels
        case PWM_11_SLEW_CW:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse_width);
            break;
        case PWM_12_SLEW_CCW:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_width);
            break;
        case PWM_13_OUTRIGGER_FRONT_UP:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_width);
            break;
        case PWM_19_TRACK_LEFT_FORWARD:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse_width);
            break;

        // TIM1 channels
        case PWM_16_OUTRIGGER_BACK_DOWN:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width);
            break;
        case PWM_17_TRACK_RIGHT_FORWARD:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_width);
            break;
        case PWM_18_TRACK_RIGHT_BACKWARD:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_width);
            break;
        case PWM_20_TRACK_LEFT_BACKWARD:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_width);
            break;

        default:
            break;
    }
}
