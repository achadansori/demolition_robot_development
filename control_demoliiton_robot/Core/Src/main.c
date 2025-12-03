/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LoRa Receiver Main Program (STM32F401CCU6)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
LoRa_ReceivedData_t lora_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  UART receive complete callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        LoRa_Receiver_IRQHandler();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // Initialize M0 and M1 pins for LoRa configuration
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;  // PE4=M0, PE5=M1
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Initialize LoRa receiver
  LoRa_Receiver_Init(&huart1, GPIOE, GPIO_PIN_4, GPIOE, GPIO_PIN_5);

  // Wait for USB to be ready (reduced from 2000ms)
  HAL_Delay(1000);

  // Send startup message to USB
  char *startup_msg = "\r\n=================================\r\n";
  CDC_Transmit_FS((uint8_t*)startup_msg, strlen(startup_msg));
  HAL_Delay(20);

  char *title_msg = "   LoRa Receiver Started\r\n";
  CDC_Transmit_FS((uint8_t*)title_msg, strlen(title_msg));
  HAL_Delay(20);

  char *port_msg = "   STM32F401CCU6 - UART1\r\n";
  CDC_Transmit_FS((uint8_t*)port_msg, strlen(port_msg));
  HAL_Delay(20);

  char *pin_msg = "   RX: PA10 | TX: PA9\r\n";
  CDC_Transmit_FS((uint8_t*)pin_msg, strlen(pin_msg));
  HAL_Delay(20);

  char *end_msg = "=================================\r\n\r\n";
  CDC_Transmit_FS((uint8_t*)end_msg, strlen(end_msg));
  HAL_Delay(50);

  // Configure LoRa module
  char *config_msg = "Configuring LoRa...\r\n";
  CDC_Transmit_FS((uint8_t*)config_msg, strlen(config_msg));

  if (LoRa_Receiver_Configure())
  {
      char *success_msg = "LoRa configured successfully!\r\n\r\n";
      CDC_Transmit_FS((uint8_t*)success_msg, strlen(success_msg));
  }
  else
  {
      char *fail_msg = "LoRa configuration failed!\r\n\r\n";
      CDC_Transmit_FS((uint8_t*)fail_msg, strlen(fail_msg));
  }

  HAL_Delay(50);

  // Send CSV header (19 fields now)
  char *csv_header = "joy_left_x,joy_left_y,joy_left_btn1,joy_left_btn2,joy_right_x,joy_right_y,joy_right_btn1,joy_right_btn2,s0,s1_1,s1_2,s2_1,s2_2,s4_1,s4_2,s5_1,s5_2,r1,r8\r\n";
  CDC_Transmit_FS((uint8_t*)csv_header, strlen(csv_header));

  HAL_Delay(50);

  char *waiting_msg = "Waiting for LoRa data...\r\n\r\n";
  CDC_Transmit_FS((uint8_t*)waiting_msg, strlen(waiting_msg));
  HAL_Delay(50);

  // Start listening for LoRa data
  LoRa_Receiver_StartListening();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Check if new data is available
    if (LoRa_Receiver_IsDataAvailable())
    {
      // Get received data
      if (LoRa_Receiver_GetData(&lora_data))
      {
        // Format CSV string (19 fields)
        char csv_buffer[200];
        int len = snprintf(csv_buffer, sizeof(csv_buffer),
                           "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                           lora_data.joy_left_x,
                           lora_data.joy_left_y,
                           lora_data.joy_left_btn1,
                           lora_data.joy_left_btn2,
                           lora_data.joy_right_x,
                           lora_data.joy_right_y,
                           lora_data.joy_right_btn1,
                           lora_data.joy_right_btn2,
                           lora_data.s0,
                           lora_data.s1_1,
                           lora_data.s1_2,
                           lora_data.s2_1,
                           lora_data.s2_2,
                           lora_data.s4_1,
                           lora_data.s4_2,
                           lora_data.s5_1,
                           lora_data.s5_2,
                           lora_data.r1,
                           lora_data.r8);

        // Forward to USB CDC (non-blocking, will drop if busy)
        CDC_Transmit_FS((uint8_t*)csv_buffer, len);
      }
    }
    // No delay here - check immediately for next data
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
