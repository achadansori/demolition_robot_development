/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define R8_Pin GPIO_PIN_0
#define R8_GPIO_Port GPIOA
#define R1_Pin GPIO_PIN_1
#define R1_GPIO_Port GPIOA
#define JOY_LEFT_X_Pin GPIO_PIN_2
#define JOY_LEFT_X_GPIO_Port GPIOA
#define JOY_LEFT_Y_Pin GPIO_PIN_3
#define JOY_LEFT_Y_GPIO_Port GPIOA
#define JOY_LEFT_BTN1_Pin GPIO_PIN_4
#define JOY_LEFT_BTN1_GPIO_Port GPIOA
#define JOY_LEFT_BTN2_Pin GPIO_PIN_5
#define JOY_LEFT_BTN2_GPIO_Port GPIOA
#define JOY_RIGHT_X_Pin GPIO_PIN_6
#define JOY_RIGHT_X_GPIO_Port GPIOA
#define JOY_RIGHT_Y_Pin GPIO_PIN_7
#define JOY_RIGHT_Y_GPIO_Port GPIOA
#define JOY_RIGHT_BTN1_Pin GPIO_PIN_0
#define JOY_RIGHT_BTN1_GPIO_Port GPIOB
#define JOY_RIGHT_BTN2_Pin GPIO_PIN_1
#define JOY_RIGHT_BTN2_GPIO_Port GPIOB
#define S5_1_Pin GPIO_PIN_10
#define S5_1_GPIO_Port GPIOB
#define S0_Pin GPIO_PIN_12
#define S0_GPIO_Port GPIOB
#define S5_2_Pin GPIO_PIN_8
#define S5_2_GPIO_Port GPIOA
#define S4_1_Pin GPIO_PIN_9
#define S4_1_GPIO_Port GPIOA
#define S4_2_Pin GPIO_PIN_10
#define S4_2_GPIO_Port GPIOA
#define S2_1_Pin GPIO_PIN_15
#define S2_1_GPIO_Port GPIOA
#define S2_2_Pin GPIO_PIN_3
#define S2_2_GPIO_Port GPIOB
#define S1_1_Pin GPIO_PIN_4
#define S1_1_GPIO_Port GPIOB
#define S1_2_Pin GPIO_PIN_5
#define S1_2_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_8
#define NRF_CE_GPIO_Port GPIOB
#define NRF_CSN_Pin GPIO_PIN_9
#define NRF_CSN_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_13
#define NRF_IRQ_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
// NRF24L01 Module pins (Hardware SPI2)
// SPI2_SCK:  PB13
// SPI2_MISO: PB14
// SPI2_MOSI: PB15
#define NRF24_CE_Pin    NRF_CE_Pin
#define NRF24_CE_Port   NRF_CE_GPIO_Port
#define NRF24_CSN_Pin   NRF_CSN_Pin
#define NRF24_CSN_Port  NRF_CSN_GPIO_Port
#define NRF24_IRQ_Pin   NRF_IRQ_Pin
#define NRF24_IRQ_Port  NRF_IRQ_GPIO_Port
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
