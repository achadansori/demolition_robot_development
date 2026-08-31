/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Custom modules
#include "var.h"
#include "joystick.h"
#include "switch.h"
#include "nrf24.h"
#include "oled.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "debug.h"
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
extern I2C_HandleTypeDef hi2c3;  // I2C3 for OLED (PA8=SCL, PC9=SDA)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Debug_Print(const char* msg);
void Debug_Printf(const char* format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t s0_emergency_flag = 0;  // Set by EXTI0 ISR when S0 = 0
volatile uint8_t clock_source_is_hsi = 0;  // 1 if HSE failed and HSI fallback is active
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // Re-configure switch pins with internal pull-down (survives CubeMX regeneration)
  MX_GPIO_ConfigureSwitchPullDown();

  // Read initial S0 state (EXTI only triggers on edges, need initial state)
  if (HAL_GPIO_ReadPin(S0_GPIO_Port, S0_Pin) == GPIO_PIN_RESET)
  {
      s0_emergency_flag = 1;
  }

  // Wait for USB CDC to be ready
  HAL_Delay(2000);

  // Debug: Send startup message
  Debug_Printf("\r\n=== Transmitter Demolition Robot (NRF24 + Discovery) ===\r\n");
  HAL_Delay(10);
  if (clock_source_is_hsi)
  {
      Debug_Printf("System Clock: 84 MHz (HSI fallback - HSE failed)\r\n\r\n");
  }
  else
  {
      Debug_Printf("System Clock: 84 MHz (HSE), USB CDC Ready\r\n\r\n");
  }
  HAL_Delay(10);

  // Debug counter for periodic status output
  uint16_t debug_counter = 0;
  #define DEBUG_INTERVAL 200  // Print debug every 200 cycles (~1s at fast loop)

  // TX diagnostics`
  uint32_t tx_ok_count = 0;
  uint32_t tx_fail_count = 0;
  uint32_t tx_not_ready = 0;


  // SLEEP mode variables
  uint8_t sleep_mode_active = 1;  // Start in SLEEP mode for safety!
  uint8_t sleep_transition_steps = 0;
  uint8_t safety_check_passed = 0;
  uint8_t last_s1_1_state = 0;
  uint8_t s1_1_hold_counter = 0;
  uint8_t s1_2_hold_counter = 0;
  uint8_t calibration_done = 0;
  uint8_t s2_1_hold_counter = 0;
  uint8_t last_s0_state = 1;  // Track S0 state to detect S0 transitions (0→1)

  #define SLEEP_TRANSITION_SPEED 10  // 10 steps transition
  #define S1_1_HOLD_REQUIRED 20      // ~20 cycles = ~0.1 second hold required (exit SLEEP)
  #define S1_2_HOLD_REQUIRED 20      // ~20 cycles = ~0.1 second hold required (calibration)
  #define S2_1_HOLD_REQUIRED 20      // ~20 cycles = ~0.1 second hold required (start MOTOR)

  // Motor starter variables
  uint8_t motor_active = 0;        // Motor starter state (0=OFF, 1=ON)

  // Safety tolerances for exiting SLEEP mode
  #define JOYSTICK_CENTER 127
  #define JOYSTICK_TOLERANCE 25   // ±5 points tolerance

  // Initialize NRF24 GPIO pins (PC6=CSN, PC7=CE) - Manual init since not in IOC
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = NRF_CSN_Pin | NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Set initial states
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);    // CSN HIGH (inactive)
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);    // CE LOW (inactive)

  // Initialize custom modules
  Var_Init();      // Initialize data structure dan sub-modules (joystick, switch)

  // Initialize NRF24L01+ module with CE=PC7, CSN=PC6
  NRF24_Init(&hspi2, NRF_CE_GPIO_Port, NRF_CE_Pin, NRF_CSN_GPIO_Port, NRF_CSN_Pin);

  // Configure NRF24 module
  if (NRF24_Configure())
  {
      // NRF24 configured successfully - LED green ON briefly
      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
      Debug_Printf("NRF24: OK\r\n");
      HAL_Delay(200);
      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
  }
  else
  {
      // NRF24 configuration failed - LED red ON
      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
      Debug_Printf("NRF24: FAIL (check wiring)\r\n");
  }
  HAL_Delay(10);

  HAL_Delay(50);

  // ========================================================================
  // WAIT FOR S0 = 1 BEFORE SYSTEM START (Emergency safety check)
  // ========================================================================
  Debug_Printf("Waiting for S0=1...\r\n");
  HAL_Delay(10);
  while (1)
  {
      Var_Update();  // Read switch states

      // Print raw S0 value for debugging
      Debug_Printf("RAW S0=%d\r\n", tx_data.switches.s0);
      HAL_Delay(10);

      if (tx_data.switches.s0 == 1)
      {
          // S0 is in NORMAL position - safe to start system!
          Debug_Printf("S0=1, System starting...\r\n");
          HAL_Delay(10);
          break;  // Exit wait loop
      }

      // S0 still in EMERGENCY position - keep waiting, blink red LED
      HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
      HAL_Delay(500);
  }
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

  // Initialize OLED Display (128x64 I2C3 on PA8/PC9)
  OLED_Init(&hi2c3);

  // Show splash screen
  OLED_ShowSplashScreen();

  Debug_Printf("Entering main loop...\r\n");
  HAL_Delay(10);

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
    // DEBUG: Print RAW hardware values BEFORE any override (every DEBUG_INTERVAL)
    // ========================================================================
    if (++debug_counter >= DEBUG_INTERVAL)
    {
        debug_counter = 0;
        Debug_PrintTxData(&tx_data);  // Print RAW values here, before override
        Debug_Printf("  NRF: ST=0x%02X | OK=%lu FAIL=%lu NRDY=%lu | LQ=%u%%\r\n",
            NRF24_GetStatus(), tx_ok_count, tx_fail_count, tx_not_ready, NRF24_GetLinkQuality());
    }

    // ========================================================================
    // S0 EMERGENCY SWITCH - Controls emergency relay on robot
    // ========================================================================
    // Also update tx_data.switches.s0 from interrupt flag
    tx_data.switches.s0 = s0_emergency_flag ? 0 : 1;

    if (s0_emergency_flag)
    {
        // S0 = 0 - EMERGENCY MODE
        // First emergency after boot: just clear (avoid any text-render error
        // before OLED is fully exercised). Subsequent emergencies show text.
        // Re-draw periodically (every ~20 loops) so an I2C glitch self-recovers.
        static uint8_t emergency_redraw_counter = 0;
        static uint8_t emergency_first_event = 1;
        if (last_s0_state != 0 || ++emergency_redraw_counter >= 20)
        {
            emergency_redraw_counter = 0;
            OLED_Clear();
            if (!emergency_first_event)
            {
                OLED_SetCursor(8, 16);
                OLED_WriteString("EMERGENCY", FONT_SIZE_NORMAL);
                OLED_SetCursor(32, 36);
                OLED_WriteString("STOP", FONT_SIZE_LARGE);
            }
            OLED_Update();
            emergency_first_event = 0;
        }

        // Override all controls to SAFE values for emergency
        tx_data.joystick.left_x  = 127;
        tx_data.joystick.left_y  = 127;
        tx_data.joystick.right_x = 127;
        tx_data.joystick.right_y = 127;

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
        tx_data.switches.motor_active = 0;

        last_s0_state = 0;
        sleep_mode_active = 1;
        sleep_transition_steps = 0;
        safety_check_passed = 0;
        s1_1_hold_counter = 0;
        s1_2_hold_counter = 0;
        calibration_done = 0;
        s2_1_hold_counter = 0;
        motor_active = 0;

        // Red LED ON for emergency
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    }
    else
    {
        // S0 = 1 - NORMAL MODE
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

        if (last_s0_state == 0)
        {
            OLED_ShowSplashScreen();
            // No HAL_Delay here - blocking would freeze emergency response
            last_s0_state = 1;
            sleep_mode_active = 1;
            sleep_transition_steps = 0;
            safety_check_passed = 0;
            s1_1_hold_counter = 0;
            s1_2_hold_counter = 0;
            calibration_done = 0;
            s2_1_hold_counter = 0;
            motor_active = 0;
        }
    }

    // ========================================================================
    // SLEEP MODE - Safety Feature with Safety Interlock
    // ========================================================================
    if (sleep_mode_active)
    {
        uint8_t joystick_safe = 0;
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

        // Check all switches are 0 (except S0, S1_1 for exit SLEEP, S1_2 for calibration)
        if ((tx_data.switches.joy_left_btn1  == 0) &&
            (tx_data.switches.joy_left_btn2  == 0) &&
            (tx_data.switches.joy_right_btn1 == 0) &&
            (tx_data.switches.joy_right_btn2 == 0) &&
            (tx_data.switches.s2_1 == 0) &&
            (tx_data.switches.s2_2 == 0) &&
            (tx_data.switches.s4_1 == 0) &&
            (tx_data.switches.s4_2 == 0) &&
            (tx_data.switches.s5_1 == 0) &&
            (tx_data.switches.s5_2 == 0))
        {
            switches_safe = 1;
        }

        if (joystick_safe && switches_safe)
        {
            safety_check_passed = 1;

            // S1_2 = Joystick Calibration (in SLEEP mode only, hold to calibrate, once only)
            if (!calibration_done && tx_data.switches.s1_2 == 1)
            {
                s1_2_hold_counter++;
                if (s1_2_hold_counter >= S1_2_HOLD_REQUIRED)
                {
                    Joystick_Calibrate();
                    calibration_done = 1;
                    s1_2_hold_counter = 0;
                }
            }
            else if (!calibration_done)
            {
                s1_2_hold_counter = 0;
            }

            if (tx_data.switches.s1_1 == 1)
            {
                s1_1_hold_counter++;
                if (s1_1_hold_counter >= S1_1_HOLD_REQUIRED)
                {
                    sleep_mode_active = 0;
                    sleep_transition_steps = 0;
                    s1_1_hold_counter = 0;
                }
            }
            else
            {
                s1_1_hold_counter = 0;
            }
            last_s1_1_state = tx_data.switches.s1_1;
        }
        else
        {
            safety_check_passed = 0;
            last_s1_1_state = 0;
            s1_1_hold_counter = 0;
            s1_2_hold_counter = 0;

            // Override transmitted data to safe values during SLEEP
            tx_data.joystick.left_x  = 127;
            tx_data.joystick.left_y  = 127;
            tx_data.joystick.right_x = 127;
            tx_data.joystick.right_y = 127;

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
        safety_check_passed = 0;
        last_s1_1_state = tx_data.switches.s1_1;
    }

    // ========================================================================
    // MOTOR STARTER CONTROL - S2_1 HOLD - SELF-HOLDING
    // ========================================================================
    if (!sleep_mode_active)
    {
        if (tx_data.switches.s2_1 == 1)
        {
            s2_1_hold_counter++;
            if (s2_1_hold_counter >= S2_1_HOLD_REQUIRED)
            {
                motor_active = 1;
            }
        }
        else
        {
            s2_1_hold_counter = 0;
        }
    }
    else
    {
        motor_active = 0;
        s2_1_hold_counter = 0;
    }

    // Update motor state in tx_data before transmission
    tx_data.switches.motor_active = motor_active;

    // Unlock state: 1 = sudah keluar SLEEP (S1_1 ditahan), 0 = masih terkunci
    // Control board pakai ini untuk mengizinkan PWM solenoid tanpa start motor
    tx_data.switches.unlocked = !sleep_mode_active;

    // Transmit via NRF24 using BINARY format (8 bytes)
    if (NRF24_IsReady())
    {
        if (NRF24_SendBinary(Var_GetBinaryData(), Var_GetDataSize()))
        {
            tx_ok_count++;
            // Transmission success - brief green LED blink
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }
        else
        {
            tx_fail_count++;
            // Transmission failed - blue LED
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        }
    }
    else
    {
        tx_not_ready++;
    }

    // Update OLED display (every 10 cycles = ~500ms)
    if (tx_data.switches.s0 == 1)
    {
        static uint8_t oled_counter = 0;
        static uint8_t last_sleep_state = 0;
        static uint8_t last_safety_state = 0;
        static uint8_t last_hold_counter = 0;
        static uint8_t last_motor_state = 0;
        static uint8_t last_cal_state = 0;

        uint8_t current_hold_progress = sleep_mode_active ? s1_1_hold_counter : s2_1_hold_counter;

        if (sleep_mode_active != last_sleep_state ||
            safety_check_passed != last_safety_state ||
            current_hold_progress != last_hold_counter ||
            motor_active != last_motor_state ||
            s1_2_hold_counter != last_cal_state ||
            ++oled_counter >= 10)
        {
            oled_counter = 0;
            last_sleep_state = sleep_mode_active;
            last_safety_state = safety_check_passed;
            last_hold_counter = current_hold_progress;
            last_motor_state = motor_active;
            last_cal_state = s1_2_hold_counter;
            uint8_t cal_progress = calibration_done ? 255 : s1_2_hold_counter;  // 255 = done
            uint8_t link_quality = NRF24_GetLinkQuality();  // 0-100 from robot Auto-ACK
            OLED_ShowModeScreen(tx_data.switches.s5_1, tx_data.switches.s5_2, (uint8_t*)&tx_data.joystick, sleep_mode_active, safety_check_passed, current_hold_progress, motor_active, cal_progress, link_quality);
            OLED_Update();
        }
    }

    // Turn off LEDs after brief indication
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

    // No delay — transmit as fast as possible
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
  HAL_StatusTypeDef status = HAL_ERROR;
  const uint8_t HSE_MAX_ATTEMPTS = 5;

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Try HSE first (8 MHz crystal → PLL → 84 MHz SYSCLK, 48 MHz USB).
   *  HSE can fail to start when VDD ramps up slowly (e.g. external 5V supply).
   *  Retry up to HSE_MAX_ATTEMPTS times, cycling HSE off/on between attempts
   *  to let the crystal oscillator restart cleanly.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  for (uint8_t attempt = 0; attempt < HSE_MAX_ATTEMPTS; attempt++)
  {
    status = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if (status == HAL_OK)
    {
      break;
    }
    __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
    HAL_Delay(50);  // SysTick runs on HSI here, so HAL_Delay still works
  }

  /** Fallback to HSI (16 MHz) if HSE never started.
   *  PLL re-tuned so SYSCLK still = 84 MHz and USB still = 48 MHz:
   *    VCO_in = HSI/PLLM = 16/16 = 1 MHz (same as HSE/8)
   *  Note: HSI tolerance (~1%) does not meet USB-CDC spec (±0.25%);
   *  USB may be unreliable in this fallback mode, but SPI/NRF24/OLED work fine.
   */
  if (status != HAL_OK)
  {
    clock_source_is_hsi = 1;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#include <stdarg.h>

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
