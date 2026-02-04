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
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Custom modules (OOP style)
#include "var.h"
#include "joystick.h"
#include "switch.h"
#include "usb.h"
#include "nrf24.h"
#include "oled.h"
#include "battery.h"
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
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // SLEEP mode variables
  uint8_t sleep_mode_active = 1;  // Start in SLEEP mode for safety!
  uint8_t sleep_transition_steps = 0;
  uint8_t safety_check_passed = 0;
  uint8_t last_s2_1_state = 0;
  uint8_t s2_1_hold_counter = 0;
  uint8_t s1_1_hold_counter = 0;
  uint8_t last_s0_state = 1;  // Track S0 state to detect S0 transitions (0→1)

  #define SLEEP_TRANSITION_SPEED 10  // 10 steps = 100ms total transition (10ms per step, very responsive!)
  #define S2_1_HOLD_REQUIRED 10      // 10 cycles x 100ms = 1 second hold required
  #define S1_1_HOLD_REQUIRED 10      // 10 cycles x 100ms = 1 second hold required

  // Motor starter variables
  uint8_t motor_active = 0;        // Motor starter state (0=OFF, 1=ON)

  // Safety tolerances for exiting SLEEP mode
  #define JOYSTICK_CENTER 127
  #define JOYSTICK_TOLERANCE 5   // ±5 points tolerance

  // Initialize custom modules
  Var_Init();      // Initialize data structure dan sub-modules (joystick, switch)
  USB_Init();      // Initialize USB CDC

  // Initialize NRF24L01+ module with CE=PB8, CSN=PB9 (Hardware SPI2)
  NRF24_Init(&hspi2, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);

  // Welcome message
  USB_Print("\r\n\r\n");
  USB_Print("========================================\r\n");
  USB_Print("  DEMOLITION ROBOT TRANSMITTER (NRF24)\r\n");
  USB_Print("========================================\r\n");
  USB_Print("Configuring NRF24L01+...\r\n");

  // Configure NRF24 module
  if (NRF24_Configure())
  {
      USB_Print("NRF24L01+ configured successfully!\r\n");
      if (NRF24_IsConnected())
      {
          USB_Print("NRF24L01+ module detected!\r\n");
      }
      else
      {
          USB_Print("WARNING: NRF24L01+ not detected!\r\n");
      }
  }
  else
  {
      USB_Print("NRF24L01+ configuration failed!\r\n");
  }

  USB_Print("NRF24L01+ initialized - Ready to transmit\r\n");
  USB_Print("========================================\r\n");
  HAL_Delay(50);

  // ========================================================================
  // WAIT FOR S0 = 1 BEFORE SYSTEM START (Emergency safety check)
  // ========================================================================
  // System will NOT start until S0 switch is in NORMAL position (S0 = 1)
  // This prevents starting system in EMERGENCY mode (S0 = 0)
  // Ensures safe startup with emergency relay (PB8) in non-active state
  USB_Print("Waiting for S0 switch in NORMAL position (not emergency)...\r\n");

  while (1)
  {
      Var_Update();  // Read switch states

      if (tx_data.switches.s0 == 1)
      {
          // S0 is in NORMAL position - safe to start system!
          USB_Print("S0 in NORMAL mode - Starting system...\r\n");
          break;  // Exit wait loop
      }

      // S0 still in EMERGENCY position - keep waiting
      HAL_Delay(100);
  }

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
    // S0 EMERGENCY SWITCH - Controls emergency relay (PB8) on robot
    // ========================================================================
    // S0 = 0: EMERGENCY MODE (transmit s0=0 → PB8 LOW → emergency relay aktif)
    // S0 = 1: NORMAL MODE (transmit s0=1 → PB8 HIGH → emergency relay non-aktif)

    if (tx_data.switches.s0 == 0)
    {
        // ====================================================================
        // S0 = 0 - EMERGENCY MODE
        // ====================================================================
        // Transmit s0 = 0 to trigger emergency relay (PB8 LOW) on robot

        // Turn off OLED to save battery
        OLED_Clear();
        OLED_Update();

        // Override all controls to SAFE values for emergency
        tx_data.joystick.left_x  = 127;
        tx_data.joystick.left_y  = 127;
        tx_data.joystick.right_x = 127;
        tx_data.joystick.right_y = 127;

        // All switches to 0 (except s0 which remains 0 for emergency signal)
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

        // Remember S0 was OFF
        last_s0_state = 0;

        // Reset all system variables (emergency state)
        sleep_mode_active = 1;
        sleep_transition_steps = 0;
        safety_check_passed = 0;
        s2_1_hold_counter = 0;
        s1_1_hold_counter = 0;
        motor_active = 0;

        // IMPORTANT: Continue to transmit s0 = 0 (DO NOT skip transmission!)
    }
    else
    {
        // ====================================================================
        // S0 = 1 - NORMAL MODE
        // ====================================================================
        // Check if S0 just transitioned from 0→1 (recovery from emergency)
        if (last_s0_state == 0)
        {
            // S0 switched from emergency to normal! Show splash screen
            USB_Print("S0 switched ON - Recovery from emergency\r\n");
            OLED_ShowSplashScreen();
            HAL_Delay(1000);  // Display splash for 1 second
            last_s0_state = 1;

            // Reset all system variables (fresh start after emergency)
            sleep_mode_active = 1;
            sleep_transition_steps = 0;
            safety_check_passed = 0;
            s2_1_hold_counter = 0;
            s1_1_hold_counter = 0;
            motor_active = 0;

            USB_Print("System restarted - Entering SLEEP mode\r\n");
        }
    }

    // ========================================================================
    // SLEEP MODE - Safety Feature with Safety Interlock
    // ========================================================================

    if (sleep_mode_active)
    {
        // ====================================================================
        // S0 = 1, but still in SLEEP mode - Check Safety Interlock to Exit
        // ====================================================================

        // Step 1: Check if all controls are in SAFE position
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

        // Check all switches are 0 (except S0 and S2_1 which are excluded)
        // S0 is emergency button (already checked above)
        // S2_1 is excluded so user can press S2_1 to exit SLEEP mode
        // S2_2 must still be OFF for safety
        if ((tx_data.switches.joy_left_btn1  == 0) &&
            (tx_data.switches.joy_left_btn2  == 0) &&
            (tx_data.switches.joy_right_btn1 == 0) &&
            (tx_data.switches.joy_right_btn2 == 0) &&
            (tx_data.switches.s1_1 == 0) &&
            (tx_data.switches.s1_2 == 0) &&
            // S2_1 NOT checked - used to exit SLEEP mode
            (tx_data.switches.s2_2 == 0) &&  // S2_2 must be OFF
            (tx_data.switches.s4_1 == 0) &&
            (tx_data.switches.s4_2 == 0) &&
            (tx_data.switches.s5_1 == 0) &&
            (tx_data.switches.s5_2 == 0))
        {
            switches_safe = 1;
        }

        // Step 2: If all safety checks pass, wait for S2_1 HOLD to exit SLEEP mode
        if (joystick_safe && switches_safe)
        {
            safety_check_passed = 1;

            // S2_1 must be held for S2_1_HOLD_REQUIRED cycles (2 seconds)
            if (tx_data.switches.s2_1 == 1)
            {
                // S2_1 is pressed - increment hold counter
                s2_1_hold_counter++;

                // Check if held long enough
                if (s2_1_hold_counter >= S2_1_HOLD_REQUIRED)
                {
                    // S2_1 held for 2 seconds - EXIT SLEEP MODE!
                    sleep_mode_active = 0;
                    sleep_transition_steps = 0;
                    s2_1_hold_counter = 0;  // Reset counter
                }
            }
            else
            {
                // S2_1 released before hold time completed - reset counter
                s2_1_hold_counter = 0;
            }

            last_s2_1_state = tx_data.switches.s2_1;
        }
        else
        {
            // Safety check failed - reset state
            safety_check_passed = 0;
            last_s2_1_state = 0;
            s2_1_hold_counter = 0;

            // Override transmitted data to safe values during SLEEP
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

    // ========================================================================
    // MOTOR STARTER CONTROL - S1_1 HOLD (1 second) - SELF-HOLDING
    // ========================================================================
    // Flow: SLEEP → Safety OK → Hold S2_1 (1s) → Exit SLEEP → Hold S1_1 (1s) → Motor ON (LATCHED)
    // Motor remains ON even after S1_1 is released (self-holding relay behavior)
    // Motor only turns OFF when entering SLEEP mode

    if (!sleep_mode_active)
    {
        // Not in SLEEP mode - motor control allowed
        if (tx_data.switches.s1_1 == 1)
        {
            // S1_1 is pressed - increment hold counter
            s1_1_hold_counter++;

            // Check if held long enough
            if (s1_1_hold_counter >= S1_1_HOLD_REQUIRED)
            {
                // S1_1 held for 1 second - ACTIVATE MOTOR! (Latched/Self-holding)
                motor_active = 1;
            }
        }
        else
        {
            // S1_1 released - Motor stays ON (self-holding), just reset counter
            s1_1_hold_counter = 0;  // Reset counter for next time
            // Note: motor_active remains unchanged (stays ON if it was ON)
        }
    }
    else
    {
        // In SLEEP mode - force motor OFF (only way to stop motor)
        motor_active = 0;
        s1_1_hold_counter = 0;
    }

    // Update motor state in tx_data before transmission
    tx_data.switches.motor_active = motor_active;

    // Transmit via NRF24L01+ using BINARY format (FAST! No parsing needed)
    // Binary format - only 8 bytes, compatible with receiver
    // NRF24 structure is compatible with our Transmitter_Data_t
    NRF24_Data_t nrf_data;
    nrf_data.joy_left_x = tx_data.joystick.left_x;
    nrf_data.joy_left_y = tx_data.joystick.left_y;
    nrf_data.joy_right_x = tx_data.joystick.right_x;
    nrf_data.joy_right_y = tx_data.joystick.right_y;
    nrf_data.r8 = tx_data.joystick.r8;
    nrf_data.r1 = tx_data.joystick.battery_percent;  // Using battery as R1
    nrf_data.joy_left_btn1 = tx_data.switches.joy_left_btn1;
    nrf_data.joy_left_btn2 = tx_data.switches.joy_left_btn2;
    nrf_data.joy_right_btn1 = tx_data.switches.joy_right_btn1;
    nrf_data.joy_right_btn2 = tx_data.switches.joy_right_btn2;
    nrf_data.s0 = tx_data.switches.s0;
    nrf_data.s1_1 = tx_data.switches.s1_1;
    nrf_data.s1_2 = tx_data.switches.s1_2;
    nrf_data.s2_1 = tx_data.switches.s2_1;
    nrf_data.s2_2 = tx_data.switches.s2_2;
    nrf_data.s4_1 = tx_data.switches.s4_1;
    nrf_data.s4_2 = tx_data.switches.s4_2;
    nrf_data.s5_1 = tx_data.switches.s5_1;
    nrf_data.s5_2 = tx_data.switches.s5_2;
    nrf_data.motor_active = tx_data.switches.motor_active;

    // Transmit packet
    NRF24_TransmitData(&nrf_data);

    // Print data ke USB untuk debugging (less frequently)
    static uint8_t usb_counter = 0;
    if (++usb_counter >= 5)  // Print USB every 5 cycles (500ms)
    {
        usb_counter = 0;
        USB_PrintData(&tx_data);
    }

    // Update OLED display with mode info and percentages (every 10 cycles = 1 second)
    // Update immediately when entering/exiting SLEEP mode, safety status changes, or hold progress changes
    // IMPORTANT: Only update OLED when S0 = 1 (NORMAL mode), not in EMERGENCY mode (S0 = 0)
    if (tx_data.switches.s0 == 1)
    {
        static uint8_t oled_counter = 0;
        static uint8_t last_sleep_state = 0;
        static uint8_t last_safety_state = 0;
        static uint8_t last_hold_counter = 0;
        static uint8_t last_motor_state = 0;

        uint8_t current_hold_progress = sleep_mode_active ? s2_1_hold_counter : s1_1_hold_counter;

        if (sleep_mode_active != last_sleep_state ||
            safety_check_passed != last_safety_state ||
            current_hold_progress != last_hold_counter ||
            motor_active != last_motor_state ||
            ++oled_counter >= 10)
        {
            oled_counter = 0;
            last_sleep_state = sleep_mode_active;
            last_safety_state = safety_check_passed;
            last_hold_counter = current_hold_progress;
            last_motor_state = motor_active;
            OLED_ShowModeScreen(tx_data.switches.s5_1, tx_data.switches.s5_2, (uint8_t*)&tx_data.joystick, sleep_mode_active, safety_check_passed, current_hold_progress, motor_active);
            OLED_Update();
        }
    }


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
