/**
  ******************************************************************************
  * @file           : pwm.h
  * @brief          : PWM Control for 20 Channels (Demolition Robot)
  *                   Uses TIM1, TIM2, TIM3, TIM4, TIM8
  ******************************************************************************
  */

#ifndef __PWM_H
#define __PWM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* PWM Channel Definitions ---------------------------------------------------*/
typedef enum {
    PWM_1_CYLINDER_1_OUT = 0,      // TIM8_CH1 (PC6)
    PWM_2_CYLINDER_1_IN,           // TIM8_CH2 (PC7)
    PWM_3_CYLINDER_2_OUT,          // TIM8_CH4 (PC9)
    PWM_4_CYLINDER_2_IN,           // TIM3_CH2 (PB5)
    PWM_5_CYLINDER_3_OUT,          // TIM2_CH2 (PA1)
    PWM_6_CYLINDER_3_IN,           // TIM3_CH3 (PB0)
    PWM_7_CYLINDER_4_OUT,          // TIM2_CH3 (PA2)
    PWM_8_CYLINDER_4_IN,           // TIM2_CH1 (PA0)
    PWM_9_TOOL_1,                  // TIM3_CH4 (PB1)
    PWM_10_TOOL_2,                 // TIM8_CH3 (PC8)
    PWM_11_SLEW_CW,                // TIM4_CH4 (PD15)
    PWM_12_SLEW_CCW,               // TIM4_CH3 (PD14)
    PWM_13_OUTRIGGER_FRONT_UP,     // TIM4_CH1 (PD12)
    PWM_14_OUTRIGGER_FRONT_DOWN,   // TIM3_CH1 (PB4)
    PWM_15_OUTRIGGER_BACK_UP,      // TIM2_CH4 (PA3)
    PWM_16_OUTRIGGER_BACK_DOWN,    // TIM1_CH1 (PE9)
    PWM_17_TRACK_RIGHT_FORWARD,    // TIM1_CH2 (PE11)
    PWM_18_TRACK_RIGHT_BACKWARD,   // TIM1_CH3 (PE13)
    PWM_19_TRACK_LEFT_FORWARD,     // TIM4_CH2 (PD13)
    PWM_20_TRACK_LEFT_BACKWARD,    // TIM1_CH4 (PE14)
    PWM_CHANNEL_COUNT = 20
} PWM_Channel_t;

/* Public function prototypes ------------------------------------------------*/
void PWM_Init(void);
void PWM_SetDutyCycle(PWM_Channel_t channel, uint8_t duty_percent);
uint8_t PWM_GetDutyCycle(PWM_Channel_t channel);
void PWM_Stop(PWM_Channel_t channel);
void PWM_StopAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __PWM_H */
