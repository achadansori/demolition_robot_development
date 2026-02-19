/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern volatile uint8_t s0_emergency_flag;  // Set by EXTI0 interrupt when S0=0
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define S1_2_Pin GPIO_PIN_5
#define S1_2_GPIO_Port GPIOE
#define S1_1_Pin GPIO_PIN_3
#define S1_1_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define S0_Pin GPIO_PIN_0
#define S0_GPIO_Port GPIOB
#define JOY_LEFT_Y_Pin GPIO_PIN_1
#define JOY_LEFT_Y_GPIO_Port GPIOC
#define JOY_LEFT_X_Pin GPIO_PIN_3
#define JOY_LEFT_X_GPIO_Port GPIOC
#define JOY_LEFT_BTN2_Pin GPIO_PIN_1
#define JOY_LEFT_BTN2_GPIO_Port GPIOA
#define JOY_LEFT_BTN1_Pin GPIO_PIN_3
#define JOY_LEFT_BTN1_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define JOY_RIGHT_Y_Pin GPIO_PIN_5
#define JOY_RIGHT_Y_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define JOY_RIGHT_X_Pin GPIO_PIN_7
#define JOY_RIGHT_X_GPIO_Port GPIOA
#define JOY_RIGHT_BTN2_Pin GPIO_PIN_5
#define JOY_RIGHT_BTN2_GPIO_Port GPIOC
#define JOY_RIGHT_BTN1_Pin GPIO_PIN_1
#define JOY_RIGHT_BTN1_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define LORA_M0_Pin GPIO_PIN_12
#define LORA_M0_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define NRF_IRQ_Pin GPIO_PIN_8
#define NRF_IRQ_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SCK_Pin GPIO_PIN_10
#define I2S3_SCK_GPIO_Port GPIOC
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_3
#define LED_B_GPIO_Port GPIOD
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_5
#define LED_G_GPIO_Port GPIOD
#define S4_2_Pin GPIO_PIN_3
#define S4_2_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_7
#define LED_R_GPIO_Port GPIOD
#define S4_1_Pin GPIO_PIN_6
#define S4_1_GPIO_Port GPIOD
#define S5_2_Pin GPIO_PIN_7
#define S5_2_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define S5_1_Pin GPIO_PIN_5
#define S5_1_GPIO_Port GPIOB
#define S2_2_Pin GPIO_PIN_8
#define S2_2_GPIO_Port GPIOB
#define LORA_M1_Pin GPIO_PIN_0
#define LORA_M1_GPIO_Port GPIOE
#define S2_1_Pin GPIO_PIN_1
#define S2_1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
// NRF24L01+ pins (SPI2)
#define NRF_CSN_Pin GPIO_PIN_6
#define NRF_CSN_GPIO_Port GPIOC
#define NRF_CE_Pin GPIO_PIN_7
#define NRF_CE_GPIO_Port GPIOC
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
