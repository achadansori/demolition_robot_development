/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Custom modules (OOP style)
#include "var.h"
#include "joystick.h"
#include "switch.h"
#include "usb.h"
#include "lora.h"
#include "oled.h"
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
// NOTE: Add this in i2c.c after enabling I2C1 in CubeMX:
// I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c1;  // I2C1 for OLED (PB6=SCL, PB7=SDA)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // SLEEP mode variables
  uint8_t sleep_mode_active = 1;  // Start in SLEEP mode for safety!
  uint8_t sleep_transition_steps = 0;
  uint8_t safety_check_passed = 0;
  uint8_t last_s2_1_state = 0;

  #define SLEEP_TRANSITION_SPEED 10  // 10 steps = 100ms total transition (10ms per step, very responsive!)

  // Safety tolerances for exiting SLEEP mode
  #define JOYSTICK_CENTER 127
  #define JOYSTICK_TOLERANCE 5   // ±5 points tolerance
  #define RESISTOR_TOLERANCE 10  // ±10 points tolerance for R1/R8

  // Initialize M0 and M1 GPIO pins for LoRa (PB8=M0, PB9=M1)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;  // PB8, PB9
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Initialize custom modules
  Var_Init();      // Initialize data structure dan sub-modules (joystick, switch)
  USB_Init();      // Initialize USB CDC

  // Initialize LoRa E220 module with M0=PB8, M1=PB9
  LoRa_Init(&huart1, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);

  // Welcome message
  USB_Print("\r\n\r\n");
  USB_Print("========================================\r\n");
  USB_Print("   DEMOLITION ROBOT TRANSMITTER\r\n");
  USB_Print("========================================\r\n");
  USB_Print("Configuring LoRa E220...\r\n");

  // Configure LoRa module
  if (LoRa_Configure())
  {
      USB_Print("LoRa configured successfully!\r\n");
  }
  else
  {
      USB_Print("LoRa configuration failed!\r\n");
  }

  USB_Print("LoRa E220 initialized - Ready to transmit\r\n");
  USB_Print("========================================\r\n");
  HAL_Delay(50);

  // Initialize OLED Display (128x64 I2C on PB6/PB7)
  // NOTE: Make sure I2C1 is enabled in CubeMX first!
  // PB6 = I2C1_SCL, PB7 = I2C1_SDA
  OLED_Init(&hi2c1);
  USB_Print("OLED Display initialized\r\n");

  // Show splash screen
  OLED_ShowSplashScreen();
  USB_Print("Splash screen shown\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Update semua data sensor
    Var_Update();

    // ========================================================================
    // SLEEP MODE - Emergency Safety Feature with Safety Interlock
    // ========================================================================

    if (tx_data.switches.s0 == 0)
    {
        // ====================================================================
        // S0 = 0 (Emergency button pressed) - FORCE SLEEP MODE
        // ====================================================================
        if (!sleep_mode_active)
        {
            sleep_mode_active = 1;
            sleep_transition_steps = 0;
            safety_check_passed = 0;
        }

        // Fast smooth transition to default values (10 steps = 100ms)
        if (sleep_transition_steps < SLEEP_TRANSITION_SPEED)
        {
            // Calculate transition factor (0.0 to 1.0)
            float factor = (float)(sleep_transition_steps + 1) / (float)SLEEP_TRANSITION_SPEED;

            // Smooth transition for joysticks to center (127)
            tx_data.joystick.left_x  = tx_data.joystick.left_x  + (int16_t)((127 - tx_data.joystick.left_x) * factor);
            tx_data.joystick.left_y  = tx_data.joystick.left_y  + (int16_t)((127 - tx_data.joystick.left_y) * factor);
            tx_data.joystick.right_x = tx_data.joystick.right_x + (int16_t)((127 - tx_data.joystick.right_x) * factor);
            tx_data.joystick.right_y = tx_data.joystick.right_y + (int16_t)((127 - tx_data.joystick.right_y) * factor);

            // Smooth transition for R1 and R8 to 0
            tx_data.joystick.r1 = tx_data.joystick.r1 - (uint8_t)(tx_data.joystick.r1 * factor);
            tx_data.joystick.r8 = tx_data.joystick.r8 - (uint8_t)(tx_data.joystick.r8 * factor);

            sleep_transition_steps++;
        }
        else
        {
            // Transition complete - force exact default values
            tx_data.joystick.left_x  = 127;
            tx_data.joystick.left_y  = 127;
            tx_data.joystick.right_x = 127;
            tx_data.joystick.right_y = 127;
            tx_data.joystick.r1 = 0;
            tx_data.joystick.r8 = 0;
        }

        // All switches to 0 (except S0 which is read from hardware)
        tx_data.switches.joy_left_btn1  = 0;
        tx_data.switches.joy_left_btn2  = 0;
        tx_data.switches.joy_right_btn1 = 0;
        tx_data.switches.joy_right_btn2 = 0;
        tx_data.switches.s1_1 = 0;
        tx_data.switches.s1_2 = 0;
        tx_data.switches.s2_1 = 0;
        tx_data.switches.s2_2 = 0;
        tx_data.switches.s4_1 = 0;
        tx_data.switches.s4_2 = 0;
        tx_data.switches.s5_1 = 0;
        tx_data.switches.s5_2 = 0;
    }
    else if (sleep_mode_active)
    {
        // ====================================================================
        // S0 = 1, but still in SLEEP mode - Check Safety Interlock to Exit
        // ====================================================================

        // Step 1: Check if all controls are in SAFE position
        uint8_t joystick_safe = 0;
        uint8_t resistor_safe = 0;
        uint8_t switches_safe = 0;

        // Check joysticks are centered (127 ± 5)
        if ((tx_data.joystick.left_x  >= JOYSTICK_CENTER - JOYSTICK_TOLERANCE) &&
            (tx_data.joystick.left_x  <= JOYSTICK_CENTER + JOYSTICK_TOLERANCE) &&
            (tx_data.joystick.left_y  >= JOYSTICK_CENTER - JOYSTICK_TOLERANCE) &&
            (tx_data.joystick.left_y  <= JOYSTICK_CENTER + JOYSTICK_TOLERANCE) &&
            (tx_data.joystick.right_x >= JOYSTICK_CENTER - JOYSTICK_TOLERANCE) &&
            (tx_data.joystick.right_x <= JOYSTICK_CENTER + JOYSTICK_TOLERANCE) &&
            (tx_data.joystick.right_y >= JOYSTICK_CENTER - JOYSTICK_TOLERANCE) &&
            (tx_data.joystick.right_y <= JOYSTICK_CENTER + JOYSTICK_TOLERANCE))
        {
            joystick_safe = 1;
        }

        // Check R1 and R8 are at 0 (0 ± 10)
        if ((tx_data.joystick.r1 <= RESISTOR_TOLERANCE) &&
            (tx_data.joystick.r8 <= RESISTOR_TOLERANCE))
        {
            resistor_safe = 1;
        }

        // Check all switches are 0 (except S0 which we already checked)
        if ((tx_data.switches.joy_left_btn1  == 0) &&
            (tx_data.switches.joy_left_btn2  == 0) &&
            (tx_data.switches.joy_right_btn1 == 0) &&
            (tx_data.switches.joy_right_btn2 == 0) &&
            (tx_data.switches.s1_1 == 0) &&
            (tx_data.switches.s1_2 == 0) &&
            (tx_data.switches.s2_1 == 0) &&  // S2_1 must be 0 initially
            (tx_data.switches.s2_2 == 0) &&
            (tx_data.switches.s4_1 == 0) &&
            (tx_data.switches.s4_2 == 0) &&
            (tx_data.switches.s5_1 == 0) &&
            (tx_data.switches.s5_2 == 0))
        {
            switches_safe = 1;
        }

        // Step 2: If all safety checks pass, wait for S2_1 press to exit
        if (joystick_safe && resistor_safe && switches_safe)
        {
            safety_check_passed = 1;

            // Detect S2_1 rising edge (press event)
            if (tx_data.switches.s2_1 == 1 && last_s2_1_state == 0)
            {
                // S2_1 pressed while all controls safe - EXIT SLEEP MODE
                sleep_mode_active = 0;
                sleep_transition_steps = 0;
                safety_check_passed = 0;
            }

            last_s2_1_state = tx_data.switches.s2_1;
        }
        else
        {
            // Safety check failed - reset state
            safety_check_passed = 0;
            last_s2_1_state = 0;

            // Override transmitted data to safe values during SLEEP
            tx_data.joystick.left_x  = 127;
            tx_data.joystick.left_y  = 127;
            tx_data.joystick.right_x = 127;
            tx_data.joystick.right_y = 127;
            tx_data.joystick.r1 = 0;
            tx_data.joystick.r8 = 0;

            // All switches to 0
            tx_data.switches.joy_left_btn1  = 0;
            tx_data.switches.joy_left_btn2  = 0;
            tx_data.switches.joy_right_btn1 = 0;
            tx_data.switches.joy_right_btn2 = 0;
            tx_data.switches.s1_1 = 0;
            tx_data.switches.s1_2 = 0;
            tx_data.switches.s2_1 = 0;
            tx_data.switches.s2_2 = 0;
            tx_data.switches.s4_1 = 0;
            tx_data.switches.s4_2 = 0;
            tx_data.switches.s5_1 = 0;
            tx_data.switches.s5_2 = 0;
        }
    }
    else
    {
        // ====================================================================
        // Normal operation - S0 = 1 and not in SLEEP mode
        // ====================================================================
        // Use actual sensor readings (already in tx_data from Var_Update)
        safety_check_passed = 0;
        last_s2_1_state = tx_data.switches.s2_1;
    }

    // Transmit via LoRa using BINARY format (FAST! No parsing needed)
    // Binary is much faster than CSV - only 8 bytes, direct copy
    if (LoRa_IsReady())
    {
        LoRa_SendBinary(Var_GetBinaryData(), Var_GetDataSize());
    }

    // Print data ke USB untuk debugging (less frequently)
    static uint8_t usb_counter = 0;
    if (++usb_counter >= 5)  // Print USB every 5 cycles (500ms)
    {
        usb_counter = 0;
        USB_PrintData(&tx_data);
    }

    // Update OLED display with mode info and percentages (every 10 cycles = 1 second)
    // Update immediately when entering/exiting SLEEP mode or when safety status changes
    static uint8_t oled_counter = 0;
    static uint8_t last_sleep_state = 0;
    static uint8_t last_safety_state = 0;

    if (sleep_mode_active != last_sleep_state ||
        safety_check_passed != last_safety_state ||
        ++oled_counter >= 10)
    {
        oled_counter = 0;
        last_sleep_state = sleep_mode_active;
        last_safety_state = safety_check_passed;
        OLED_ShowModeScreen(tx_data.switches.s5_1, tx_data.switches.s5_2, (uint8_t*)&tx_data.joystick, sleep_mode_active, safety_check_passed);
        OLED_Update();
    }

    // Delay 100ms for stable transmission (10Hz update rate)
    // Slower rate = more time for LoRa to process, reduces delay
    HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
