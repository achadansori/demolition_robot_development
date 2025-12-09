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
#include "control.h"
#include "pwm.h"
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

  // ========================================================================
  // SAFETY FEATURE: Communication Timeout Watchdog
  // ========================================================================
  // If no data received from transmitter for COMM_TIMEOUT_MS,
  // smoothly transition all PWM outputs to 0 to prevent runaway robot!
  #define COMM_TIMEOUT_MS         500     // 500ms timeout (10 missed packets @ 50ms rate)
  #define SAFETY_TRANSITION_STEPS 20      // 20 steps for smooth transition to 0

  uint32_t last_data_received_time = 0;   // Timestamp of last valid LoRa packet
  uint8_t safety_mode_active = 0;         // 1 = timeout detected, transitioning to safe state
  uint8_t safety_transition_step = 0;     // Current step in smooth transition (0-20)
  uint8_t pwm_backup[PWM_CHANNEL_COUNT];  // Backup of PWM values before safety transition

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

  // Send startup info
  char *format_msg = "Format: JL:x,y,b1,b2 JR:x,y,b1,b2 POT:R8=x,R1=x SW:S0=x,S1=xx,S2=xx,S4=xx,S5=xx\r\n\r\n";
  CDC_Transmit_FS((uint8_t*)format_msg, strlen(format_msg));

  HAL_Delay(50);

  char *waiting_msg = "Waiting for LoRa data...\r\n\r\n";
  CDC_Transmit_FS((uint8_t*)waiting_msg, strlen(waiting_msg));
  HAL_Delay(50);

  // Start listening for LoRa data
  LoRa_Receiver_StartListening();

  // Initialize control system (PWM outputs)
  Control_Init();

  char *control_msg = "Control system initialized - Ready!\r\n\r\n";
  CDC_Transmit_FS((uint8_t*)control_msg, strlen(control_msg));
  HAL_Delay(50);

  // Initialize safety timeout - give transmitter time to start (1 second grace period)
  last_data_received_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // ========================================================================
    // COMMUNICATION TIMEOUT SAFETY WATCHDOG
    // ========================================================================
    uint32_t current_time = HAL_GetTick();
    uint32_t time_since_last_data = current_time - last_data_received_time;

    // Check if new data is available
    if (LoRa_Receiver_IsDataAvailable())
    {
      // Get received data
      if (LoRa_Receiver_GetData(&lora_data))
      {
        // Update timestamp - we received valid data!
        last_data_received_time = current_time;

        // If recovering from safety mode, restore normal operation
        if (safety_mode_active)
        {
          safety_mode_active = 0;
          safety_transition_step = 0;
          // PWM will be restored by Control_Update() below

          // Send recovery message to USB
          char *recovery_msg = "\r\n*** COMM RESTORED! Resuming normal operation ***\r\n";
          CDC_Transmit_FS((uint8_t*)recovery_msg, strlen(recovery_msg));
        }

        // Update control outputs based on received data (normal operation)
        Control_Update(&lora_data);

        // Print to USB less frequently to avoid blocking
        // USB CDC_Transmit is SLOW and can cause delay if called too often
        static uint8_t usb_counter = 0;
        if (++usb_counter >= 10)  // Print USB every 10 packets (500ms) to minimize blocking
        {
          usb_counter = 0;

          // Format human-readable string (matching transmitter format)
          char output_buffer[200];
          int len = snprintf(output_buffer, sizeof(output_buffer),
                             "JL:%03d,%03d,%d,%d JR:%03d,%03d,%d,%d POT:R8=%d,R1=%d SW:S0=%d,S1=%d%d,S2=%d%d,S4=%d%d,S5=%d%d\r\n",
                             lora_data.joy_left_x,
                             lora_data.joy_left_y,
                             lora_data.joy_left_btn1,
                             lora_data.joy_left_btn2,
                             lora_data.joy_right_x,
                             lora_data.joy_right_y,
                             lora_data.joy_right_btn1,
                             lora_data.joy_right_btn2,
                             lora_data.r8,
                             lora_data.r1,
                             lora_data.s0,
                             lora_data.s1_1,
                             lora_data.s1_2,
                             lora_data.s2_1,
                             lora_data.s2_2,
                             lora_data.s4_1,
                             lora_data.s4_2,
                             lora_data.s5_1,
                             lora_data.s5_2);

          // Forward to USB CDC (print every 10th packet to minimize blocking delay)
          CDC_Transmit_FS((uint8_t*)output_buffer, len);
        }
        // Data is still received every 50ms via LoRa realtime - just not printed every time
      }
    }

    // ========================================================================
    // SAFETY TIMEOUT: Smooth transition to 0 if no data received
    // ========================================================================
    else if (time_since_last_data > COMM_TIMEOUT_MS)
    {
      // TIMEOUT DETECTED! No data received for more than 500ms
      // Smoothly transition all PWM outputs to 0 to prevent runaway robot

      if (!safety_mode_active)
      {
        // First time entering safety mode - backup current PWM values
        safety_mode_active = 1;
        safety_transition_step = 0;

        for (uint8_t i = 0; i < PWM_CHANNEL_COUNT; i++)
        {
          pwm_backup[i] = PWM_GetDutyCycle((PWM_Channel_t)i);
        }

        // Send warning message to USB
        char *warning_msg = "\r\n*** COMM TIMEOUT! Transitioning PWM to 0... ***\r\n";
        CDC_Transmit_FS((uint8_t*)warning_msg, strlen(warning_msg));
      }

      // Smooth transition: Gradually reduce all PWM to 0 over 20 steps
      // Each step runs every 10ms (in main loop), total transition = 200ms
      static uint32_t last_transition_step = 0;
      if (HAL_GetTick() - last_transition_step >= 10)  // 10ms per step
      {
        last_transition_step = HAL_GetTick();

        if (safety_transition_step < SAFETY_TRANSITION_STEPS)
        {
          safety_transition_step++;

          // Calculate transition factor (1.0 → 0.0)
          float factor = 1.0f - ((float)safety_transition_step / (float)SAFETY_TRANSITION_STEPS);

          // Apply smooth transition to all PWM channels
          for (uint8_t i = 0; i < PWM_CHANNEL_COUNT; i++)
          {
            uint8_t new_duty = (uint8_t)(pwm_backup[i] * factor);
            PWM_SetDutyCycle((PWM_Channel_t)i, new_duty);
          }
        }
        else
        {
          // Transition complete - force all PWM to exact 0
          PWM_StopAll();
        }
      }
    }

    // ========================================================================
    // Send PWM data via USB CDC for monitoring
    // Send every 100ms (10Hz) - independent of LoRa packet rate
    // ========================================================================
    static uint32_t last_pwm_send = 0;
    if (HAL_GetTick() - last_pwm_send >= 100)
    {
      last_pwm_send = HAL_GetTick();

      // Create binary packet: Header(2) + PWM Data(20) + Checksum(1) = 23 bytes
      uint8_t pwm_packet[23];
      pwm_packet[0] = 0xAA;  // Header byte 1
      pwm_packet[1] = 0x55;  // Header byte 2

      // Get all 20 PWM duty cycle values (0-100%)
      for (uint8_t i = 0; i < 20; i++)
      {
        pwm_packet[2 + i] = PWM_GetDutyCycle((PWM_Channel_t)i);
      }

      // Calculate XOR checksum of PWM data bytes
      uint8_t checksum = 0;
      for (uint8_t i = 2; i < 22; i++)
      {
        checksum ^= pwm_packet[i];
      }
      pwm_packet[22] = checksum;

      // Send packet via USB CDC
      CDC_Transmit_FS(pwm_packet, 23);
    }

    // No delay here - process LoRa data immediately without blocking
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
