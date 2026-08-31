/**
  ******************************************************************************
  * @file           : pwm.h
  * @brief          : PWM Control for 21 Channels (Demolition Robot)
  *                   Uses TIM1, TIM2, TIM3, TIM4, TIM8, TIM9
  *
  * @note           PWM Configuration for Hydraulic Proportional Valves:
  *                 - Frequency: CONFIGURABLE in pwm.c (default: 100Hz / 10ms period)
  *                 - Duty Cycle: 0-100% maps to 0-3.3V average
  *                 - Register-based implementation for direct hardware control
  *                 - Each channel drives TIP122 base through resistor
  *
  * @note           Frequency Configuration (in pwm.c):
  *                 - Change PWM_FREQUENCY_HZ define to adjust solenoid response
  *                 - Lower frequency (50-100Hz) prevents sticking at low duty cycles
  *                 - Higher frequency (500-1000Hz) gives faster, more precise control
  *                 - Current: 100Hz - good balance for preventing solenoid sticking
  *                 - Valid range: 50Hz-2kHz (proportional valve operating range)
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
    PWM_1_CYLINDER_1_OUT = 0,      // TIM2_CH2 (PA1) - Cylinder 1 OUT (joy_right_btn2 + joystick)
    PWM_2_CYLINDER_1_IN,           // TIM2_CH4 (PA3) - Cylinder 1 IN (joy_right_btn2 + joystick)
    PWM_3_CYLINDER_2_OUT,          // TIM3_CH4 (PB1)
    PWM_4_CYLINDER_2_IN,           // TIM1_CH1 (PE9)
    PWM_5_CYLINDER_3_OUT,          // TIM1_CH2 (PE11)
    PWM_6_CYLINDER_3_IN,           // TIM1_CH3 (PE13)
    PWM_7_CYLINDER_4_OUT,          // TIM4_CH2 (PD13)
    PWM_8_CYLINDER_4_IN,           // TIM4_CH4 (PD15)
    PWM_9_TOOL_1,                  // GPIO     (PD12) - digital ON/OFF, not PWM
    PWM_10_TOOL_2,                 // GPIO     (PD14) - digital ON/OFF, not PWM
    PWM_11_SLEW_CW,                // TIM1_CH4 (PE14)
    PWM_12_SLEW_CCW,               // TIM3_CH3 (PB0)
    PWM_13_OUTRIGGER_LEFT_UP,      // TIM2_CH3 (PA2)
    PWM_14_OUTRIGGER_LEFT_DOWN,    // TIM2_CH1 (PA0)
    PWM_15_OUTRIGGER_RIGHT_UP,     // TIM3_CH1 (PB4)
    PWM_16_OUTRIGGER_RIGHT_DOWN,   // TIM8_CH3 (PC8)
    PWM_17_TRACK_RIGHT_FORWARD,    // TIM8_CH1 (PC6)
    PWM_18_TRACK_RIGHT_BACKWARD,   // TIM8_CH2 (PC7)
    PWM_19_TRACK_LEFT_FORWARD,     // TIM8_CH4 (PC9)
    PWM_20_TRACK_LEFT_BACKWARD,    // TIM3_CH2 (PB5)
    PWM_CHANNEL_COUNT = 20
} PWM_Channel_t;

/* Public function prototypes ------------------------------------------------*/
void PWM_Init(void);
void PWM_SetDutyCycle(PWM_Channel_t channel, uint8_t duty_percent);
uint8_t PWM_GetDutyCycle(PWM_Channel_t channel);
void PWM_Stop(PWM_Channel_t channel);
void PWM_StopAll(void);

/* GPIO Control for Tools (digital trigger - ON/OFF) -------------------------*/
void GPIO_SetTool1(uint8_t state);  // 0 = OFF, 1 = ON (PD12)
void GPIO_SetTool2(uint8_t state);  // 0 = OFF, 1 = ON (PD14)

#ifdef __cplusplus
}
#endif

#endif /* __PWM_H */
