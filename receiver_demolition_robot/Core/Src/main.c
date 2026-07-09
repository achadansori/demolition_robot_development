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
#include "nrf24.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* NRF24L01+ control pins (same wiring as the control board: SPI1 on PA5/6/7,
 * CE=PE4, CSN=PE5; PE3 is the Discovery LIS302DL CS that must stay HIGH to
 * keep it off the shared SPI1 bus). */
#define NRF_CE_GPIO_Port    GPIOE
#define NRF_CE_Pin          GPIO_PIN_4
#define NRF_CSN_GPIO_Port   GPIOE
#define NRF_CSN_Pin         GPIO_PIN_5
#define LIS_CS_GPIO_Port    GPIOE
#define LIS_CS_Pin          GPIO_PIN_3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint8_t clock_source_is_hsi = 0;  /* 1 if HSE failed and HSI fallback is active */
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim14;
SPI_HandleTypeDef hspi1;
CANopenNodeSTM32 canOpenNodeSTM32;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void MX_CAN1_Init(void);
static void MX_TIM14_Init(void);
static void MX_SPI1_Init(void);
static void MX_NRF24_GPIO_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  /* ---- CAN + 1 ms timer + CANopen (bridge = Node-ID 1, sends TPDO1) ---- */
  MX_CAN1_Init();
  MX_TIM14_Init();

  canOpenNodeSTM32.CANHandle = &hcan1;
  canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
  canOpenNodeSTM32.timerHandle = &htim14;
  canOpenNodeSTM32.desiredNodeID = CTRL_LINK_BRIDGE_NODE_ID;
  canOpenNodeSTM32.baudrate = CTRL_LINK_BITRATE_KBPS;
  canopen_app_init(&canOpenNodeSTM32);

  /* ---- NRF24 receive side (radio link from the transmitter) ---- */
  MX_SPI1_Init();
  MX_NRF24_GPIO_Init();

  NRF24_Init(&hspi1, NRF_CE_GPIO_Port, NRF_CE_Pin, NRF_CSN_GPIO_Port, NRF_CSN_Pin);

  /* At COLD power-on the NRF24 module's supply can still be ramping when the
   * MCU gets here, so a single Configure() fails silently and the radio never
   * comes up (symptom: receiver needs its reset button pressed once before
   * the link works - an MCU-only reset retries while the module is already
   * powered). Configure() verifies its CONFIG readback, so retry until it
   * reports success. Boot-time only; no effect on the running loop. */
  for (uint8_t attempt = 0; attempt < 10; attempt++)
  {
    if (NRF24_Configure())
    {
      break;
    }
    HAL_Delay(100);
  }
  NRF24_StartListening();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    /* ---- Forward the NRF24 radio packet onto the CAN bus (OD 0x2000) ----
     * The 8-byte radio payload has the exact byte/bit layout of ctrl_link.h,
     * so it is copied straight into the TPDO1-mapped process data and the
     * control board (RPDO1 consumer) decodes it just like an NRF24 packet.
     * TPDO1 auto-transmits every 50 ms via its event timer.
     *
     * If the radio link drops for >500 ms we publish an all-zero packet
     * (S0=0 -> emergency, motor_active=0) so the robot fails safe instead of
     * latching the last command. */
    static uint32_t last_rf_ms = 0;
    uint8_t d[8];
    uint8_t have_packet = 0;

    if (NRF24_IsDataAvailable() && NRF24_GetRawPayload(d))
    {
      have_packet = 1;
      last_rf_ms = HAL_GetTick();
    }
    else if ((HAL_GetTick() - last_rf_ms) >= 500u)
    {
      for (int i = 0; i < 8; i++) { d[i] = 0; }   /* radio lost -> safe packet */
      have_packet = 1;
    }

    if (have_packet && canopenNodeSTM32 != NULL && canopenNodeSTM32->canOpenStack != NULL)
    {
      CO_LOCK_OD(canopenNodeSTM32->canOpenStack->CANmodule);
      for (int i = 0; i < 8; i++) { OD_RAM.x2000_controlData[i] = d[i]; }
      CO_UNLOCK_OD(canopenNodeSTM32->canOpenStack->CANmodule);
    }

    /* === CAN connection self-heal ===
     * The bridge knows the control board (Node-ID 2) is connected by consuming its
     * CANopen heartbeat (OD 0x1016). If the link was up and has now been lost for
     * >2 s, do a full CANopen communication reset to clear any stuck CAN state so it
     * reconnects by itself (backup to the TX-abort handling in the CAN driver). */
    {
      uint8_t link_alive = 0;
      CO_t* co = (canopenNodeSTM32 != NULL) ? canopenNodeSTM32->canOpenStack : NULL;
      if (co != NULL && co->HBcons != NULL && co->HBcons->monitoredNodes != NULL)
      {
        link_alive = (co->HBcons->monitoredNodes[0].HBstate == CO_HBconsumer_ACTIVE);
      }

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

    canopen_app_process();
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

/* SPI1 master for the NRF24L01+ (PA5=SCK, PA6=MISO, PA7=MOSI).
 * APB2 = 84 MHz, /16 = 5.25 MHz SCK (NRF24 max 10 MHz). */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* SPI1 pin alternate-function setup (called by HAL_SPI_Init). */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (spiHandle->Instance == SPI1)
  {
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/* NRF24 control pins: CE (PE4), CSN (PE5) as push-pull outputs, and the
 * Discovery LIS302DL CS (PE3) forced HIGH so it stays off the SPI1 bus. */
static void MX_NRF24_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Park CS / CSN / CE before configuring as outputs */
  HAL_GPIO_WritePin(LIS_CS_GPIO_Port, LIS_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LIS_CS_Pin | NRF_CE_Pin | NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LIS_CS_GPIO_Port, LIS_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
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
