 /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Demolition Robot Control board (STM32F407VGT6)
  *                   Receives the 8-byte control packet over a CANopen bus
  *                   (RPDO1, Node-ID 2) and drives the hydraulic PWM outputs.
  *
  *                   The radio link is gone: an upstream bridge
  *                   (receiver_demolition_robot) receives the NRF24 packet and
  *                   republishes the identical 8 bytes via TPDO1. This board is
  *                   CAN-only.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2s.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "OD.h"
#include "ctrl_link.h"
#include "nrf24.h"          /* only for NRF24_ReceivedData_t (packet layout) */
#include "control.h"
#include "pwm.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* The bridge TPDO1 fires every ~50 ms; treat the link as alive while a fresh
 * RPDO1 arrived within this window (snappy connect/disconnect, no flicker). */
#define COMM_TIMEOUT_MS  200u

/* End-to-end freshness window: the transmitter increments byte 5 of the
 * packet every loop, so it changes on every 50 ms TPDO. If it stops changing
 * while RPDOs still arrive, a hung bridge (or hung TX) is replaying stale
 * data - treat the link as dead.
 * Must stay ABOVE the bridge's radio window (250 ms) + one TPDO period
 * (50 ms) + margin, or normal short radio gaps (TX OLED updates) would trip
 * it and cause spurious drops. 400 ms it is. */
#define FRESH_TIMEOUT_MS 400u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t clock_source_is_hsi = 0;  /* 1 if HSE failed and HSI fallback active */
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim14;
CANopenNodeSTM32 canOpenNodeSTM32;

NRF24_ReceivedData_t nrf24_data;           /* decoded control packet (fed to Control_Update) */
static char debug_buffer[256];

/* Marks the time of the last RPDO1 reception (set from the OD write extension). */
static volatile uint32_t ctrl_last_rx_tick = 0;
static OD_extension_t ctrl_OD2000_ext;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Debug_Print(const char* msg);
void Debug_Printf(const char* format, ...);
static void MX_CAN1_Init(void);
static void MX_TIM14_Init(void);
static void ctrl_decode(const uint8_t d[8], NRF24_ReceivedData_t* o);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Force every actuator output to its safe state using ONLY direct
  *         register writes (no HAL, no globals) so it can be called from any
  *         fault handler even with a corrupted heap/stack or disabled IRQs.
  *
  *         - all PWM compare values -> 0 (constant-low within one PWM period)
  *         - TIM1/TIM8 main output enable (MOE) cleared -> outputs cut now
  *         - PB1 (Tool 1 / breaker), PB8 (emergency relay), PE6 (motor
  *           starter) -> LOW
  */
void Failsafe_EmergencyOutputs(void)
{
    TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCR4 = 0;
    TIM2->CCR1 = 0; TIM2->CCR2 = 0; TIM2->CCR3 = 0; TIM2->CCR4 = 0;
    TIM3->CCR1 = 0; TIM3->CCR2 = 0; TIM3->CCR3 = 0; TIM3->CCR4 = 0;
    TIM4->CCR1 = 0; TIM4->CCR2 = 0; TIM4->CCR3 = 0; TIM4->CCR4 = 0;
    TIM8->CCR1 = 0; TIM8->CCR2 = 0; TIM8->CCR3 = 0; TIM8->CCR4 = 0;

    /* Advanced timers: cut the outputs immediately, not at the next update */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;

    GPIOB->BSRR = (1u << (1 + 16));   /* PB1 Tool 1 (breaker) LOW  */
    GPIOB->BSRR = (1u << (8 + 16));   /* PB8 emergency relay  LOW  */
    GPIOE->BSRR = (1u << (6 + 16));   /* PE6 motor starter    LOW  */
}

/* Independent watchdog (direct register access, no HAL module needed).
 * LSI/32 = ~1 kHz -> reload ~= timeout in ms. If the control loop ever hangs,
 * the MCU resets and re-boots into Control_Init() (all PWM at 0%) instead of
 * leaving the hydraulics driven by the last duty cycle forever. */
#define IWDG_TIMEOUT_MS 800u

static void Failsafe_IWDG_Start(void)
{
    /* Freeze the IWDG while the core is halted by a debugger, otherwise
     * every breakpoint would end in a watchdog reset. No effect in the field. */
      DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

      IWDG->KR  = 0x5555u;                      /* unlock PR/RLR access */
    IWDG->PR  = 3u;                           /* LSI / 32 -> ~1 kHz   */
    IWDG->RLR = (IWDG_TIMEOUT_MS > 4095u) ? 4095u : IWDG_TIMEOUT_MS;
    IWDG->KR  = 0xAAAAu;                      /* load reload value    */
    IWDG->KR  = 0xCCCCu;                      /* start watchdog       */
}

static inline void Failsafe_IWDG_Refresh(void)
{
    IWDG->KR = 0xAAAAu;
}

/**
  * @brief  Send debug message via USB CDC (best-effort, NON-BLOCKING).
  *         No HAL_Delay: a delay here would add latency to the control loop.
  *         If the CDC endpoint is busy the message is simply dropped.
  */
void Debug_Print(const char* msg)
{
    if (msg == NULL) return;
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

/**
  * @brief  Send formatted debug message via USB CDC (printf-style,
  *         best-effort, NON-BLOCKING - see Debug_Print).
  */
void Debug_Printf(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);
    CDC_Transmit_FS((uint8_t*)debug_buffer, strlen(debug_buffer));
}

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

/* Decode the 8-byte ctrl_link packet (same layout as the old NRF24 payload). */
static void ctrl_decode(const uint8_t d[8], NRF24_ReceivedData_t* o)
{
    o->joy_left_x  = d[0];
    o->joy_left_y  = d[1];
    o->joy_right_x = d[2];
    o->joy_right_y = d[3];
    /* Bytes 4-5 no longer carry the R8/R1 pots: the transmitter now sends
     * its battery percentage in byte 4 (byte 5 reserved). Mapping byte 4
     * into r8 would drive the breaker flow valve (PWM_10) proportional to
     * the remote's battery level — force both to 0 instead. */
    o->r8 = 0;
    o->r1 = 0;

    uint16_t sw = (uint16_t)((d[7] << 8) | d[6]);
    o->joy_left_btn1  = (sw >> 0)  & 0x01;
    o->joy_left_btn2  = (sw >> 1)  & 0x01;
    o->joy_right_btn1 = (sw >> 2)  & 0x01;
    o->joy_right_btn2 = (sw >> 3)  & 0x01;
    o->s0   = (sw >> 4)  & 0x01;
    o->s1_1 = (sw >> 5)  & 0x01;
    o->s1_2 = (sw >> 6)  & 0x01;
    o->s2_1 = (sw >> 7)  & 0x01;
    o->s2_2 = (sw >> 8)  & 0x01;
    o->s4_1 = (sw >> 9)  & 0x01;
    o->s4_2 = (sw >> 10) & 0x01;
    o->s5_1 = (sw >> 11) & 0x01;
    o->s5_2 = (sw >> 12) & 0x01;
    o->motor_active = (sw >> 13) & 0x01;
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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /* ---- CAN + 1 ms timer + CANopen (control = Node-ID 2, receives RPDO1) ---- */
  MX_CAN1_Init();
  MX_TIM14_Init();

  /* hook the OD control-data object so we know when fresh RPDO data arrives.
   * MUST be installed BEFORE canopen_app_init(): RPDO mapping snapshots the
   * OD write function (OD_getSub in PDOconfigMap) during init, so an
   * extension installed afterwards is never called -> ctrl_last_rx_tick
   * would never update and the link would always read DOWN. */
  ctrl_OD2000_ext.object = NULL;
  ctrl_OD2000_ext.read = OD_readOriginal;
  ctrl_OD2000_ext.write = ctrl_OD2000_write;
  OD_extension_init(OD_ENTRY_H2000, &ctrl_OD2000_ext);

  canOpenNodeSTM32.CANHandle = &hcan1;
  canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
  canOpenNodeSTM32.timerHandle = &htim14;
  canOpenNodeSTM32.desiredNodeID = CTRL_LINK_CONTROL_NODE_ID;
  canOpenNodeSTM32.baudrate = CTRL_LINK_BITRATE_KBPS;
  canopen_app_init(&canOpenNodeSTM32);

  /* Initialize control system (PWM outputs to safe 0%) */
  Control_Init();

  /* Wait for USB CDC, then announce */
  HAL_Delay(2000);
  Debug_Printf("\r\n=== Demolition Robot Control (CANopen Node-ID %u) ===\r\n",
               (unsigned)CTRL_LINK_CONTROL_NODE_ID);
  HAL_Delay(10);  /* let the first CDC frame drain (prints are non-blocking now) */
  Debug_Printf("CAN 500 kbps on PD0/PD1, waiting for RPDO1...\r\n\r\n");

  /* Watchdog: started after all blocking init (USB wait) is done. From here
   * on the loop must keep refreshing it or the MCU resets - which re-enters
   * Control_Init() with every PWM output at 0%. */
  Failsafe_IWDG_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_diag_time = 0;
  uint32_t rx_packet_seen = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Feed the watchdog - must stay the first thing the loop does */
    Failsafe_IWDG_Refresh();

    canopen_app_process();

    /* ---- Read the latest RPDO1 process data (OD 0x2000) ---- */
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

    uint32_t now = HAL_GetTick();

    /* Link validity = three independent checks, ALL must pass:
     * 1. link_alive : an RPDO1 arrived within COMM_TIMEOUT_MS (CAN hop OK)
     * 2. data_fresh : the TX freshness counter (byte 5) keeps changing
     *                 (catches a hung bridge/TX whose ISR replays old data)
     * 3. sig_ok     : packet signature bits present (rejects a foreign or
     *                 incompatible transmitter and the bridge's radio-loss
     *                 zero packet) */
    uint8_t link_alive = ((now - ctrl_last_rx_tick) < COMM_TIMEOUT_MS);

    static uint8_t  last_counter = 0;
    static uint32_t counter_change_tick = 0;
    if (d[CTRL_BYTE_COUNTER] != last_counter)
    {
      last_counter = d[CTRL_BYTE_COUNTER];
      counter_change_tick = now;
    }
    uint8_t data_fresh = ((now - counter_change_tick) < FRESH_TIMEOUT_MS);

    uint8_t sig_ok  = ctrl_signature_ok_raw(d);
    uint8_t link_ok = (link_alive && data_fresh && sig_ok);

    /* Re-arm interlock: after ANY link interruption the robot must not jump
     * back into motion just because the radio came back while the operator
     * still holds a deflected stick with the motor latched ON. Motion stays
     * blocked until a clean packet with motor_active=0 arrives (operator
     * cycles S0 on the remote, which forces it to SLEEP). Starts latched at
     * boot; the transmitter's boot-in-SLEEP state clears it immediately. */
    static uint8_t rearm_required = 1;
    if (!link_ok)
    {
      rearm_required = 1;
    }

    if (link_ok)
    {
      ctrl_decode(d, &nrf24_data);
      rx_packet_seen = 1;

      if (rearm_required)
      {
        if (nrf24_data.motor_active == 0)
        {
          rearm_required = 0;  /* TX is in SLEEP / motor off -> safe to resume */
        }
        else
        {
          /* Block motion but do NOT drop the S0 emergency relay: treat as
           * motor-off + neutral sticks (sleep branch in Control_Update). */
          nrf24_data.motor_active = 0;
          nrf24_data.joy_left_x  = 127;
          nrf24_data.joy_left_y  = 127;
          nrf24_data.joy_right_x = 127;
          nrf24_data.joy_right_y = 127;
        }
      }
    }
    else
    {
      /* Link lost/stale/invalid -> all-zero packet: s0=0 forces the
       * emergency-stop branch in Control_Update (PWM all 0, motor relay +
       * tool off). Fail safe. */
      memset(&nrf24_data, 0, sizeof(nrf24_data));
    }

    Control_Update(&nrf24_data);

    /* Diagnostic output every 1 second (single non-blocking CDC frame) */
    if ((now - last_diag_time) >= 1000u)
    {
      last_diag_time = now;
      if (rx_packet_seen)
      {
        Debug_Printf("LINK=%s%s | RAW:[%02X %02X %02X %02X %02X %02X %02X %02X] | LX=%3d LY=%3d RX=%3d RY=%3d | S0=%d S5=%d%d | M=%d\r\n",
            link_ok ? "UP  " : "DOWN",
            (!link_ok && link_alive) ? (sig_ok ? "(stale)" : "(sig)") :
                (rearm_required ? "(rearm)" : ""),
            d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7],
            nrf24_data.joy_left_x, nrf24_data.joy_left_y,
            nrf24_data.joy_right_x, nrf24_data.joy_right_y,
            nrf24_data.s0, nrf24_data.s5_1, nrf24_data.s5_2,
            nrf24_data.motor_active);
      }
      else
      {
        Debug_Printf("LINK=%s | RAW:[%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
            link_ok ? "UP  " : "DOWN",
            d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
      }
    }
    /* USER CODE END 3 */
  }
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
  /* Try HSE first (8 MHz) -> PLL -> 168 MHz. HSE can fail to start on a slow
   * VDD ramp; retry a few times, cycling it off/on between attempts, instead
   * of hanging in Error_Handler with the robot dead. */
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

  /* Fallback to HSI (16 MHz) if HSE never started. PLL re-tuned so SYSCLK
   * stays 168 MHz (VCO_in = HSI/16 = 1 MHz, same as HSE/8): CAN bit timing,
   * timer clocks and PWM frequency are all unchanged. */
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

/* USER CODE BEGIN 4 */
/* CAN1: 500 kbps @ APB1 = 42 MHz  (Tq = 1/(42M/6); bit = 1+11+2 = 14 Tq). */
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

/* CAN1 pin + clock + NVIC setup (called by HAL_CAN_Init). PD0=RX, PD1=TX. */
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
}

/* TIM14: 1 ms time base for CANopenNode (APB1 timer clock = 84 MHz). */
static void MX_TIM14_Init(void)
{
  __HAL_RCC_TIM14_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);

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
  /* Force every actuator to its safe state BEFORE parking: the PWM timers
   * run in hardware and would otherwise keep driving the hydraulics with the
   * last duty cycle forever. If the IWDG is already running, the MCU resets
   * out of this loop into a clean boot (Control_Init -> all PWM 0%). */
  Failsafe_EmergencyOutputs();
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
