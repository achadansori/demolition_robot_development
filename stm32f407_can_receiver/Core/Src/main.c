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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "OD.h"
#include "ctrl_link.h"

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
volatile uint8_t clock_source_is_hsi = 0;  /* 1 if HSE failed and HSI fallback is active */
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim14;
CANopenNodeSTM32 canOpenNodeSTM32;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void MX_CAN1_Init(void);
static void MX_TIM14_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Marks the time of the last RPDO1 reception (set from the OD write extension). */
static volatile uint32_t ctrl_last_rx_tick = 0;
static OD_extension_t ctrl_OD2000_ext;

/* Called by the stack whenever RPDO1 writes the control data (OD 0x2000). */
static ODR_t ctrl_OD2000_write(OD_stream_t* stream, const void* buf,
                               OD_size_t count, OD_size_t* countWritten)
{
  ctrl_last_rx_tick = HAL_GetTick();
  return OD_writeOriginal(stream, buf, count, countWritten);
}

/* 1 ms timer interrupt -> CANopenNode tmrThread (RPDO/TPDO/SYNC processing). */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (canopenNodeSTM32 != NULL && htim == canopenNodeSTM32->timerHandle)
  {
    canopen_app_interrupt();
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
  /* USER CODE BEGIN 2 */
  /* ---- CAN + 1 ms timer + CANopen (control = Node-ID 2, receives RPDO1) ---- */
  MX_CAN1_Init();
  MX_TIM14_Init();

  canOpenNodeSTM32.CANHandle = &hcan1;
  canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
  canOpenNodeSTM32.timerHandle = &htim14;
  canOpenNodeSTM32.desiredNodeID = CTRL_LINK_CONTROL_NODE_ID;
  canOpenNodeSTM32.baudrate = CTRL_LINK_BITRATE_KBPS;
  canopen_app_init(&canOpenNodeSTM32);

  /* hook the OD control-data object so we know when fresh RPDO data arrives */
  ctrl_OD2000_ext.object = NULL;
  ctrl_OD2000_ext.read = OD_readOriginal;
  ctrl_OD2000_ext.write = ctrl_OD2000_write;
  OD_extension_init(OD_ENTRY_H2000, &ctrl_OD2000_ext);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    canopen_app_process();

    /* ---- Read the latest RPDO1 process data (OD 0x2000) ----
     * The 8 received bytes are available in OD_RAM.x2000_controlData[] and decoded
     * with the ctrl_link helpers; the real control.c/pwm.c logic plugs in here. */
    uint8_t d[8];
    if (canopenNodeSTM32 != NULL && canopenNodeSTM32->canOpenStack != NULL)
    {
      CO_LOCK_OD(canopenNodeSTM32->canOpenStack->CANmodule);
      for (int i = 0; i < 8; i++) { d[i] = OD_RAM.x2000_controlData[i]; }
      CO_UNLOCK_OD(canopenNodeSTM32->canOpenStack->CANmodule);
    }
    else
    {
      for (int i = 0; i < 8; i++) { d[i] = 0; }
    }
    (void)d;

    uint32_t now = HAL_GetTick();
    /* CAN link is "alive" while a fresh RPDO1 arrived recently. Frames come ~every
     * 50 ms (bridge TPDO1 event timer), so a 200 ms window gives a snappy connect/
     * disconnect indication without flickering on a single missed frame. */
    uint8_t link_alive = ((now - ctrl_last_rx_tick) < 200u);

    /* Self-heal: if the link was up and has now been lost for >2 s, do a full
     * CANopen communication reset to clear any stuck CAN state so it reconnects by
     * itself (backup to the TX-abort handling in the CAN driver). */
    {
      static uint8_t  was_up  = 0;
      static uint32_t lost_ms = 0;
      if (link_alive) { was_up = 1; lost_ms = 0; }
      else if (was_up)
      {
        uint32_t tnow = HAL_GetTick();
        if (lost_ms == 0u) { lost_ms = tnow; }
        else if ((tnow - lost_ms) >= 2000u) { lost_ms = tnow; canopen_app_resetCommunication(); }
      }
    }
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
  /* Try HSE first (8 MHz from ST-LINK MCO on the Discovery) -> PLL -> 168 MHz.
   * At cold power-on (board powered from the ST-LINK USB) the MCO may not be running yet
   * when the F407 reaches here, so HSE can fail and the board would hang in Error_Handler.
   * Retry HSE a few times, cycling it off/on between attempts. */
  HAL_StatusTypeDef status = HAL_ERROR;
  const uint8_t HSE_MAX_ATTEMPTS = 5;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  for (uint8_t attempt = 0; attempt < HSE_MAX_ATTEMPTS; attempt++)
  {
    status = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if (status == HAL_OK)
    {
      break;
    }
    __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
    HAL_Delay(50);  /* SysTick still runs on HSI here, so HAL_Delay works */
  }

  /* Fallback to HSI (16 MHz) if HSE never started. PLL re-tuned so SYSCLK stays 168 MHz
   * (VCO_in = HSI/16 = 1 MHz, same as HSE/8) so the CAN bit timing is unchanged. */
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
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* The only GPIOs used are CAN1 PD0/PD1, configured in HAL_CAN_MspInit(). */

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* CAN1: 500 kbps @ APB1 = 42 MHz  (Tq = 1/(42M/6); bit = 1+11+2 = 14 Tq -> 500 kbps).
 * For a SINGLE-BOARD self-test without CAN transceivers, change CAN_MODE_NORMAL
 * to CAN_MODE_LOOPBACK below. */
static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = ENABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM14: 1 ms time base for CANopenNode (APB1 timer clock = 84 MHz). */
static void MX_TIM14_Init(void)
{
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 84 - 1;          /* 84 MHz / 84 = 1 MHz */
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000 - 1;           /* 1 MHz / 1000 = 1 kHz = 1 ms */
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
}

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
