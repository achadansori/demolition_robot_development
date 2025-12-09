/**
  ******************************************************************************
  * @file           : pwm.c
  * @brief          : PWM Control Implementation for 21 Channels (Register-based)
  *                   Uses direct register access like working example
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pwm.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
// PWM Frequency: 200 Hz (5ms period) - EASY to measure on oscilloscope at 5ms/div
// Timer clock = 168 MHz (TIM1, TIM8) or 84 MHz (TIM2, TIM3, TIM4)
#define PWM_FREQUENCY       200     // 200Hz - 5ms period (perfect for oscilloscope measurement!)
#define PWM_PRESCALER_APB2  167     // For TIM1, TIM8: 168MHz / (167+1) = 1MHz
#define PWM_PRESCALER_APB1  83      // For TIM2, TIM3, TIM4: 84MHz / (83+1) = 1MHz
#define PWM_PERIOD          4999    // 1MHz / (4999+1) = 200 Hz (5ms period)

/* Private variables ---------------------------------------------------------*/
static uint8_t pwm_duty[PWM_CHANNEL_COUNT] = {0};

/* Private function prototypes -----------------------------------------------*/
static void PWM_ConfigureGPIO(void);
static void PWM_TIM1_Init(void);
static void PWM_TIM2_Init(void);
static void PWM_TIM3_Init(void);
static void PWM_TIM4_Init(void);
static void PWM_TIM8_Init(void);
static void PWM_TIM9_Init(void);

/**
  * @brief  Initialize all PWM channels
  * @retval None
  */
void PWM_Init(void)
{
    // Configure GPIO pins first
    PWM_ConfigureGPIO();

    // Configure each timer with register access
    PWM_TIM1_Init();
    PWM_TIM2_Init();
    PWM_TIM3_Init();
    PWM_TIM4_Init();
    PWM_TIM8_Init();
    PWM_TIM9_Init();

    // Initialize all channels to 0%
    PWM_StopAll();
}

/**
  * @brief  Configure GPIO pins for PWM output (using register access)
  * @retval None
  */
static void PWM_ConfigureGPIO(void)
{
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOEEN;

    // Configure TIM1 pins (PE9, PE11, PE13, PE14) - AF1
    GPIOE->MODER &= ~((3<<(9*2)) | (3<<(11*2)) | (3<<(13*2)) | (3<<(14*2)));
    GPIOE->MODER |= (2<<(9*2)) | (2<<(11*2)) | (2<<(13*2)) | (2<<(14*2));  // Alternate function
    GPIOE->AFR[1] &= ~((0xF<<((9-8)*4)) | (0xF<<((11-8)*4)) | (0xF<<((13-8)*4)) | (0xF<<((14-8)*4)));
    GPIOE->AFR[1] |= (1<<((9-8)*4)) | (1<<((11-8)*4)) | (1<<((13-8)*4)) | (1<<((14-8)*4));  // AF1
    GPIOE->OSPEEDR |= (3<<(9*2)) | (3<<(11*2)) | (3<<(13*2)) | (3<<(14*2));  // High speed

    // Configure TIM2 pins (PA0, PA1, PA2, PA3) - AF1
    GPIOA->MODER &= ~((3<<(0*2)) | (3<<(1*2)) | (3<<(2*2)) | (3<<(3*2)));
    GPIOA->MODER |= (2<<(0*2)) | (2<<(1*2)) | (2<<(2*2)) | (2<<(3*2));
    GPIOA->AFR[0] &= ~((0xF<<(0*4)) | (0xF<<(1*4)) | (0xF<<(2*4)) | (0xF<<(3*4)));
    GPIOA->AFR[0] |= (1<<(0*4)) | (1<<(1*4)) | (1<<(2*4)) | (1<<(3*4));  // AF1
    GPIOA->OSPEEDR |= (3<<(0*2)) | (3<<(1*2)) | (3<<(2*2)) | (3<<(3*2));

    // Configure TIM3 pins (PB0, PB1, PB4, PB5) - AF2
    GPIOB->MODER &= ~((3<<(0*2)) | (3<<(1*2)) | (3<<(4*2)) | (3<<(5*2)));
    GPIOB->MODER |= (2<<(0*2)) | (2<<(1*2)) | (2<<(4*2)) | (2<<(5*2));
    GPIOB->AFR[0] &= ~((0xF<<(0*4)) | (0xF<<(1*4)) | (0xF<<(4*4)) | (0xF<<(5*4)));
    GPIOB->AFR[0] |= (2<<(0*4)) | (2<<(1*4)) | (2<<(4*4)) | (2<<(5*4));  // AF2
    GPIOB->OSPEEDR |= (3<<(0*2)) | (3<<(1*2)) | (3<<(4*2)) | (3<<(5*2));

    // Configure TIM4 pins (PD12, PD13, PD14, PD15) - AF2
    GPIOD->MODER &= ~((3<<(12*2)) | (3<<(13*2)) | (3<<(14*2)) | (3<<(15*2)));
    GPIOD->MODER |= (2<<(12*2)) | (2<<(13*2)) | (2<<(14*2)) | (2<<(15*2));
    GPIOD->AFR[1] &= ~((0xF<<((12-8)*4)) | (0xF<<((13-8)*4)) | (0xF<<((14-8)*4)) | (0xF<<((15-8)*4)));
    GPIOD->AFR[1] |= (2<<((12-8)*4)) | (2<<((13-8)*4)) | (2<<((14-8)*4)) | (2<<((15-8)*4));  // AF2
    GPIOD->OSPEEDR |= (3<<(12*2)) | (3<<(13*2)) | (3<<(14*2)) | (3<<(15*2));

    // Configure TIM8 pins (PC6, PC7, PC8, PC9) - AF3
    GPIOC->MODER &= ~((3<<(6*2)) | (3<<(7*2)) | (3<<(8*2)) | (3<<(9*2)));
    GPIOC->MODER |= (2<<(6*2)) | (2<<(7*2)) | (2<<(8*2)) | (2<<(9*2));
    GPIOC->AFR[0] &= ~((0xF<<(6*4)) | (0xF<<(7*4)));
    GPIOC->AFR[0] |= (3<<(6*4)) | (3<<(7*4));  // AF3
    GPIOC->AFR[1] &= ~((0xF<<((8-8)*4)) | (0xF<<((9-8)*4)));
    GPIOC->AFR[1] |= (3<<((8-8)*4)) | (3<<((9-8)*4));  // AF3
    GPIOC->OSPEEDR |= (3<<(6*2)) | (3<<(7*2)) | (3<<(8*2)) | (3<<(9*2));

    // Configure TIM9 pin (PE6) - AF3
    GPIOE->MODER &= ~(3<<(6*2));
    GPIOE->MODER |= (2<<(6*2));  // Alternate function
    GPIOE->AFR[0] &= ~(0xF<<(6*4));
    GPIOE->AFR[0] |= (3<<(6*4));  // AF3
    GPIOE->OSPEEDR |= (3<<(6*2));  // High speed
}

/**
  * @brief  Initialize TIM1 (Advanced timer) for PWM - APB2 168MHz
  * @retval None
  */
static void PWM_TIM1_Init(void)
{
    // Enable TIM1 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Configure timer
    TIM1->PSC = PWM_PRESCALER_APB2;  // 168MHz / 168 = 1MHz
    TIM1->ARR = PWM_PERIOD;          // 1MHz / 5000 = 200Hz (5ms period)

    // Configure all 4 channels as PWM mode 1
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);
    TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos);
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    // Set initial duty cycle to 0
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;

    // Enable channels
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Enable auto-reload preload
    TIM1->CR1 |= TIM_CR1_ARPE;

    // IMPORTANT: Enable main output for advanced timers (TIM1, TIM8)
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Start timer
    TIM1->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Initialize TIM2 (General purpose) for PWM - APB1 84MHz
  * @retval None
  */
static void PWM_TIM2_Init(void)
{
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure timer
    TIM2->PSC = PWM_PRESCALER_APB1;  // 84MHz / 84 = 1MHz
    TIM2->ARR = PWM_PERIOD;          // 1MHz / 5000 = 200Hz (5ms period)

    // Configure all 4 channels as PWM mode 1
    TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);
    TIM2->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos);
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    // Set initial duty cycle to 0
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    // Enable channels
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Enable auto-reload preload
    TIM2->CR1 |= TIM_CR1_ARPE;

    // Start timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Initialize TIM3 (General purpose) for PWM - APB1 84MHz
  * @retval None
  */
static void PWM_TIM3_Init(void)
{
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure timer
    TIM3->PSC = PWM_PRESCALER_APB1;  // 84MHz / 84 = 1MHz
    TIM3->ARR = PWM_PERIOD;          // 1MHz / 5000 = 200Hz (5ms period)

    // Configure all 4 channels as PWM mode 1
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);
    TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos);
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    // Set initial duty cycle to 0
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;

    // Enable channels
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Enable auto-reload preload
    TIM3->CR1 |= TIM_CR1_ARPE;

    // Start timer
    TIM3->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Initialize TIM4 (General purpose) for PWM - APB1 84MHz
  * @retval None
  */
static void PWM_TIM4_Init(void)
{
    // Enable TIM4 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Configure timer
    TIM4->PSC = PWM_PRESCALER_APB1;  // 84MHz / 84 = 1MHz
    TIM4->ARR = PWM_PERIOD;          // 1MHz / 5000 = 200Hz (5ms period)

    // Configure all 4 channels as PWM mode 1
    TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM4->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
    TIM4->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    TIM4->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);
    TIM4->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos);
    TIM4->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    // Set initial duty cycle to 0
    TIM4->CCR1 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;

    // Enable channels
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Enable auto-reload preload
    TIM4->CR1 |= TIM_CR1_ARPE;

    // Start timer
    TIM4->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Initialize TIM8 (Advanced timer) for PWM - APB2 168MHz
  * @retval None
  */
static void PWM_TIM8_Init(void)
{
    // Enable TIM8 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    // Configure timer
    TIM8->PSC = PWM_PRESCALER_APB2;  // 168MHz / 168 = 1MHz
    TIM8->ARR = PWM_PERIOD;          // 1MHz / 5000 = 200Hz (5ms period)

    // Configure channels 1, 2, 3, 4 as PWM mode 1
    TIM8->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM8->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    TIM8->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);
    TIM8->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos);
    TIM8->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    // Set initial duty cycle to 0
    TIM8->CCR1 = 0;
    TIM8->CCR2 = 0;
    TIM8->CCR3 = 0;
    TIM8->CCR4 = 0;

    // Enable channels
    TIM8->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Enable auto-reload preload
    TIM8->CR1 |= TIM_CR1_ARPE;

    // IMPORTANT: Enable main output for advanced timers (TIM1, TIM8)
    TIM8->BDTR |= TIM_BDTR_MOE;

    // Start timer
    TIM8->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Initialize TIM9 (General-purpose timer) for PWM - APB2 168MHz
  * @retval None
  */
static void PWM_TIM9_Init(void)
{
    // Enable TIM9 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

    // Configure timer
    TIM9->PSC = PWM_PRESCALER_APB2;  // 168MHz / 168 = 1MHz
    TIM9->ARR = PWM_PERIOD;          // 1MHz / 5000 = 200Hz (5ms period)

    // Configure channel 2 as PWM mode 1 (PE6 = TIM9_CH2)
    TIM9->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM9->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos);  // PWM mode 1
    TIM9->CCMR1 |= TIM_CCMR1_OC2PE;           // Preload enable

    // Set initial duty cycle to 0
    TIM9->CCR2 = 0;

    // Enable channel 2
    TIM9->CCER |= TIM_CCER_CC2E;

    // Enable auto-reload preload
    TIM9->CR1 |= TIM_CR1_ARPE;

    // NOTE: TIM9 is general-purpose timer (not advanced), so no BDTR_MOE needed

    // Start timer
    TIM9->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Set PWM duty cycle for a channel
  * @param  channel: PWM channel (0-20)
  * @param  duty_percent: Duty cycle percentage (0-100)
  * @retval None
  */
void PWM_SetDutyCycle(PWM_Channel_t channel, uint8_t duty_percent)
{
    if (channel >= PWM_CHANNEL_COUNT) return;
    if (duty_percent > 100) duty_percent = 100;

    // Store duty cycle
    pwm_duty[channel] = duty_percent;

    // Calculate CCR value
    uint32_t ccr_value = ((PWM_PERIOD + 1) * duty_percent) / 100;

    // Set CCR based on channel mapping
    switch(channel)
    {
        // TIM8 channels
        case PWM_1_CYLINDER_1_OUT:  TIM8->CCR1 = ccr_value; break;  // PC6
        case PWM_2_CYLINDER_1_IN:   TIM8->CCR2 = ccr_value; break;  // PC7
        case PWM_3_CYLINDER_2_OUT:  TIM8->CCR4 = ccr_value; break;  // PC9
        case PWM_10_TOOL_2:         TIM8->CCR3 = ccr_value; break;  // PC8

        // TIM3 channels
        case PWM_4_CYLINDER_2_IN:   TIM3->CCR2 = ccr_value; break;  // PB5
        case PWM_6_CYLINDER_3_IN:   TIM3->CCR3 = ccr_value; break;  // PB0
        case PWM_9_TOOL_1:          TIM3->CCR4 = ccr_value; break;  // PB1
        case PWM_14_OUTRIGGER_LEFT_DOWN: TIM3->CCR1 = ccr_value; break;  // PB4

        // TIM2 channels
        case PWM_5_CYLINDER_3_OUT:  TIM2->CCR2 = ccr_value; break;  // PA1
        case PWM_7_CYLINDER_4_OUT:  TIM2->CCR3 = ccr_value; break;  // PA2
        case PWM_8_CYLINDER_4_IN:   TIM2->CCR1 = ccr_value; break;  // PA0
        case PWM_15_OUTRIGGER_RIGHT_UP: TIM2->CCR4 = ccr_value; break;  // PA3

        // TIM4 channels
        case PWM_11_SLEW_CW:        TIM4->CCR4 = ccr_value; break;  // PD15
        case PWM_12_SLEW_CCW:       TIM4->CCR3 = ccr_value; break;  // PD14
        case PWM_13_OUTRIGGER_LEFT_UP: TIM4->CCR1 = ccr_value; break;  // PD12
        case PWM_19_TRACK_LEFT_FORWARD: TIM4->CCR2 = ccr_value; break;  // PD13

        // TIM1 channels
        case PWM_16_OUTRIGGER_RIGHT_DOWN: TIM1->CCR1 = ccr_value; break;  // PE9
        case PWM_17_TRACK_RIGHT_FORWARD:  TIM1->CCR2 = ccr_value; break;  // PE11
        case PWM_18_TRACK_RIGHT_BACKWARD: TIM1->CCR3 = ccr_value; break;  // PE13
        case PWM_20_TRACK_LEFT_BACKWARD:  TIM1->CCR4 = ccr_value; break;  // PE14

        // TIM9 channels
        case PWM_21_MOTOR_STARTER:  TIM9->CCR2 = ccr_value; break;  // PE6

        default: break;
    }
}

/**
  * @brief  Get current duty cycle for a channel
  * @param  channel: PWM channel (0-20)
  * @retval Duty cycle percentage (0-100)
  */
uint8_t PWM_GetDutyCycle(PWM_Channel_t channel)
{
    if (channel >= PWM_CHANNEL_COUNT) return 0;
    return pwm_duty[channel];
}

/**
  * @brief  Stop PWM on a single channel
  * @param  channel: PWM channel to stop
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
        PWM_SetDutyCycle((PWM_Channel_t)i, 0);
    }

    // Clear duty array
    memset(pwm_duty, 0, sizeof(pwm_duty));
}
