 /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : NRF24 Receiver Main Program (STM32F407VGT6)
  *                   Demolition Robot Control via NRF24L01+
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24.h"
#include "control.h"
#include "pwm.h"
#include "can.h"
#include "ctrl_can.h"
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
NRF24_ReceivedData_t nrf24_data;
static char debug_buffer[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Debug_Print(const char* msg);
void Debug_Printf(const char* format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Send debug message via USB CDC
  * @param  msg: Null-terminated string to send
  * @retval None
  */
void Debug_Print(const char* msg)
{
    if (msg == NULL) return;
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    HAL_Delay(5);  // Allow USB to process
}

/**
  * @brief  Send formatted debug message via USB CDC (like printf)
  * @param  format: Format string (printf-style)
  * @param  ...: Variable arguments
  * @retval None
  */
void Debug_Printf(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);
    CDC_Transmit_FS((uint8_t*)debug_buffer, strlen(debug_buffer));
    HAL_Delay(1);  // Minimal USB processing time
}

/* USER CODE END 0 */

/**
  * @brief  The  application entry point.
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();  // Initialize USB CDC
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  // Deselect the onboard LIS302DL accelerometer. The NRF24 has moved to SPI2
  // (PB13/PB14/PB15), so PA5/PA6/PA7 are no longer driven - but keep PE3
  // (CS_I2C_SPI) HIGH anyway so the MEMS never wakes onto the SPI1 lines.
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET);

  // ========================================================================
  // SAFETY FEATURE: Communication Timeout Watchdog with Noise Filter
  // ========================================================================
  // If no data received from transmitter for COMM_TIMEOUT_MS,
  // smoothly transition all PWM outputs to 0 to prevent runaway robot!
  // Noise filter: Requires TIMEOUT_NOISE_FILTER consecutive failures before triggering
  #define COMM_TIMEOUT_MS         500     // 500ms timeout (10 missed packets @ 50ms rate)
  #define SAFETY_TRANSITION_STEPS 20      // 20 steps for smooth transition to 0
  // CATATAN: ini dihitung per ITERASI LOOP, bukan per paket. Main loop
  // free-running (tidak ada delay), jadi 5 hitungan tercapai dalam hitungan
  // mikrodetik setelah ambang 500 ms lewat - praktis tidak menambah peredaman.
  // Yang benar-benar menyaring noise adalah COMM_TIMEOUT_MS itu sendiri.
  #define TIMEOUT_NOISE_FILTER    5       // Require 5 consecutive timeouts before safety mode

  uint32_t last_data_received_time = 0;   // Timestamp of last valid NRF24 packet
  uint8_t safety_mode_active = 0;         // 1 = timeout detected, transitioning to safe state
  uint8_t safety_transition_step = 0;     // Current step in smooth transition (0-20)
  uint8_t pwm_backup[PWM_CHANNEL_COUNT];  // Backup of PWM values before safety transition
  uint8_t timeout_noise_counter = 0;      // Count consecutive timeouts for noise filtering

  // Debug counter for periodic output
  uint16_t debug_counter = 0;
  #define DEBUG_INTERVAL 20  // Print debug every 20 cycles

  // Packet counters for diagnostics
  uint32_t rx_packet_count = 0;
  uint32_t can_packet_count = 0;          // Packets that arrived over the wire
  uint8_t  last_src_is_can = 0;           // Transport of the most recent packet
  // MISS: bertambah tiap iterasi loop yang tidak menemukan paket. Loop ini
  // free-running, jadi nilainya mengukur kecepatan loop - BUKAN paket hilang,
  // dan tetap puluhan ribu per detik walau link sempurna. Pakai PKT/CANPKT
  // untuk menilai link; MISS hanya berguna sebagai tanda loop masih berputar.
  uint32_t no_data_count = 0;
  uint32_t last_diag_time = 0;

  // Initialize NRF24L01+ receiver (SPI2 - PE5=CSN, PB13=SCK, PB14=MISO, PB15=MOSI, PE4=CE)
  // Pin configuration based on main.h definitions
  NRF24_Init(&hspi2, NRF_CE_GPIO_Port, NRF_CE_Pin, NRF_CSN_GPIO_Port, NRF_CSN_Pin);

  // Wait for USB CDC to be ready
  HAL_Delay(2000);

  // Send startup message via USB CDC
  Debug_Printf("\r\n========================================\r\n");
  Debug_Printf("  NRF24 Receiver Started (USB CDC)\r\n");
  Debug_Printf("  STM32F407VGT6 - SPI2\r\n");
  Debug_Printf("  CE: PE4 | CSN: PE5\r\n");
  Debug_Printf("========================================\r\n\r\n");

  // RAW SPI2 TEST: Manual read of NRF24 STATUS register (0x07)
  Debug_Printf("SPI2 RAW TEST...\r\n");
  {
      // Ensure PE3 HIGH (deselect LIS302DL)
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
      // Ensure NRF24 CSN HIGH first
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
      HAL_Delay(1);

      // Read STATUS register manually: send NOP (0xFF), read status
      uint8_t cmd = 0xFF;
      uint8_t status = 0;
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);  // CSN LOW
      HAL_SPI_TransmitReceive(&hspi2, &cmd, &status, 1, 100);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);    // CSN HIGH

      Debug_Printf("  NOP -> STATUS=0x%02X (expect 0x0E)\r\n", status);

      // Read CONFIG register: send 0x00, then read 1 byte
      cmd = 0x00;
      uint8_t config_val = 0;
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi2, &cmd, &status, 1, 100);
      HAL_SPI_Receive(&hspi2, &config_val, 1, 100);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);

      Debug_Printf("  CONFIG=0x%02X (expect 0x08 default)\r\n", config_val);

      // Read RF_CH register (0x05)
      cmd = 0x05;
      uint8_t rfch_val = 0;
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi2, &cmd, &status, 1, 100);
      HAL_SPI_Receive(&hspi2, &rfch_val, 1, 100);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);

      Debug_Printf("  RF_CH=0x%02X (expect 0x02 default)\r\n", rfch_val);

      // Check PE3 and PE5 pin state
      Debug_Printf("  PE3(LIS_CS)=%d PE5(NRF_CSN)=%d\r\n",
          HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3),
          HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5));

      if (status == 0x00 && config_val == 0x00)
          Debug_Printf("  ** NRF24 NOT RESPONDING on SPI2! Check wiring **\r\n");
      else
          Debug_Printf("  ** SPI2 communication OK **\r\n");
  }
  Debug_Printf("\r\n");

  // Configure NRF24 module
  Debug_Printf("Configuring NRF24L01+...\r\n");

  if (NRF24_Configure())
  {
      Debug_Printf("NRF24L01+ configured successfully!\r\n\r\n");
  }
  else
  {
      Debug_Printf("NRF24L01+ configuration failed!\r\n\r\n");
  }

  Debug_Printf("Waiting for NRF24 data...\r\n\r\n");

  // Start listening for NRF24 data
  NRF24_StartListening();

  // Start listening on the wire too, and begin announcing ourselves. Harmless
  // with no transceiver attached: nothing arrives and the heartbeat frames are
  // simply never acknowledged.
  CtrlCAN_Init();
  Debug_Printf("CAN1 listening on PD0/PD1 @ 500 kbps (ID 0x181)\r\n\r\n");

  // Initialize control system (PWM outputs)
  Control_Init();

  Debug_Printf("Control system initialized - Ready!\r\n\r\n");

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

    // Listen on both transports. CAN wins when a frame is there: if the cable
    // is plugged in it is the link under test, and it carries the identical
    // 8-byte packet (ctrl_link.h) so the decode is shared with the radio path.
    uint8_t can_payload[8];
    uint8_t got_data = 0;

    if (CtrlCAN_GetData(can_payload))
    {
      NRF24_DecodePayload(can_payload, &nrf24_data);
      can_packet_count++;
      last_src_is_can = 1;
      got_data = 1;
    }
    else if (NRF24_IsDataAvailable() && NRF24_GetData(&nrf24_data))
    {
      rx_packet_count++;
      last_src_is_can = 0;
      got_data = 1;
    }
    else
    {
      no_data_count++;
    }

    if (got_data)
    {
      // Update timestamp - we received valid data!
      // Both transports stamp the same variable, which is the only thing the
      // timeout watchdog, the ramp-down and the motor re-arm look at - so the
      // whole safety path stays transport-agnostic with no changes.
      last_data_received_time = current_time;

      // Reset timeout noise counter - valid data received
      timeout_noise_counter = 0;

      // If recovering from safety mode, restore normal operation
      if (safety_mode_active)
      {
        safety_mode_active = 0;
        safety_transition_step = 0;
        // PWM will be restored by Control_Update() below
      }

      // Update control outputs based on received data (normal operation)
      Control_Update(&nrf24_data);
    }

    // Tell the transmitter the cable is connected. Must keep running even while
    // no control packet arrives - it is the transmitter's only switch signal.
    CtrlCAN_HeartbeatTask();

    // Diagnostic output every 1 second
    if ((current_time - last_diag_time) >= 1000)
    {
        // Read NRF24 registers for SPI check
        uint8_t reg_config = NRF24_ReadReg(0x00);
        uint8_t reg_status = NRF24_ReadReg(0x07);
        uint8_t reg_rfch   = NRF24_ReadReg(0x05);
        uint8_t reg_rfset  = NRF24_ReadReg(0x06);
        uint8_t reg_fifo   = NRF24_ReadReg(0x17);

        // SRC = which transport delivered the last packet. Comparing the PKT
        // and CANPKT deltas against the transmitter's OK/FAIL counters is what
        // tells apart "packets are lost" from "only the ACKs are lost".
        Debug_Printf("SRC=%s PKT=%lu CANPKT=%lu MISS=%lu | CFG=0x%02X ST=0x%02X CH=%d RF=0x%02X FIFO=0x%02X\r\n",
            last_src_is_can ? "CAN" : "RF",
            rx_packet_count, can_packet_count, no_data_count,
            reg_config, reg_status, reg_rfch, reg_rfset, reg_fifo);

        // Read raw NRF24 data for diagnostics
        uint8_t raw_payload[8];
        memcpy(raw_payload, (uint8_t*)&nrf24_data, 8);

        Debug_Printf("  RAW:[%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
            raw_payload[0], raw_payload[1], raw_payload[2], raw_payload[3],
            raw_payload[4], raw_payload[5], raw_payload[6], raw_payload[7]);

        Debug_Printf("  LX=%3d LY=%3d RX=%3d RY=%3d | S0=%d S1=%d%d S2=%d%d S4=%d%d S5=%d%d | U=%d M=%d\r\n",
            nrf24_data.joy_left_x, nrf24_data.joy_left_y,
            nrf24_data.joy_right_x, nrf24_data.joy_right_y,
            nrf24_data.s0,
            nrf24_data.s1_1, nrf24_data.s1_2,
            nrf24_data.s2_1, nrf24_data.s2_2,
            nrf24_data.s4_1, nrf24_data.s4_2,
            nrf24_data.s5_1, nrf24_data.s5_2,
            nrf24_data.unlocked,
            nrf24_data.motor_active
        );

        // Radio hanya di-init sekali di boot dan reset MCU tidak mereset chip
        // nRF24. Kalau link mati (supply belum stabil saat cold boot, brownout
        // relay, glitch SPI) satu-satunya pemulihan dulu adalah tombol reset.
        // Konfigurasi ulang di sini = tombol reset otomatis, 1x per detik.
        // Ambang 3 detik, bukan COMM_TIMEOUT_MS: fading RF biasa jangan
        // sampai memicu power-cycle radio (~15ms buta) tiap detik.
        if (time_since_last_data > 3000)
        {
            NRF24_Configure();
            NRF24_StartListening();
        }

        last_diag_time = current_time;
        no_data_count = 0;
    }

    // ========================================================================
    // SAFETY TIMEOUT: Smooth transition to 0 if no data received (with noise filter)
    // ========================================================================
    if (time_since_last_data > COMM_TIMEOUT_MS)
    {
      // TIMEOUT DETECTED! No data received for more than 500ms
      // Use noise filter to prevent false triggers from single packet loss

      // Increment timeout counter
      if (timeout_noise_counter < TIMEOUT_NOISE_FILTER)
      {
        timeout_noise_counter++;
      }

      // Only enter safety mode if we've had TIMEOUT_NOISE_FILTER consecutive timeouts
      if (timeout_noise_counter >= TIMEOUT_NOISE_FILTER && !safety_mode_active)
      {
        // First time entering safety mode - backup current PWM values
        safety_mode_active = 1;
        safety_transition_step = 0;

        // Rekam kondisi radio TEPAT saat putus, sebelum self-heal 3 detik
        // sempat memperbaikinya. Diagnostik 1 detik sering ketinggalan momen
        // ini. Nilainya yang memisahkan tiga penyebab yang gejalanya sama:
        //   CFG=0x0F CH=76      -> konfigurasi utuh, sinyal RF-nya yang hilang
        //   CFG=0x08 CH=2       -> nilai default pabrik = radio ke-reset,
        //                          artinya supply drop / brownout
        //   CFG atau ST 0x00/0xFF -> SPI tidak menjawab = wiring PB13/14/15
        uint8_t t_cfg = NRF24_ReadReg(0x00);
        uint8_t t_st  = NRF24_ReadReg(0x07);
        uint8_t t_ch  = NRF24_ReadReg(0x05);

        Debug_Printf("TIMEOUT! CFG=0x%02X ST=0x%02X CH=%d (0x0F/76=RF, 0x08/2=reset, 00|FF=SPI)\r\n",
                     t_cfg, t_st, t_ch);

        for (uint8_t i = 0; i < PWM_CHANNEL_COUNT; i++)
        {
          pwm_backup[i] = PWM_GetDutyCycle((PWM_Channel_t)i);
        }

        // Reset nrf24_data to safe defaults (joystick center, all switches OFF)
        memset(&nrf24_data, 0, sizeof(nrf24_data));
        nrf24_data.joy_left_x  = 127;
        nrf24_data.joy_left_y  = 127;
        nrf24_data.joy_right_x = 127;
        nrf24_data.joy_right_y = 127;

        // ENTER SLEEP MODE: Set PE6 to LOW
        GPIOE->BSRR = (1<<(6+16));  // BR6 = reset PE6 to LOW

        // Relay emergency ikut dilepas. Selama safety mode Control_Update()
        // tidak dipanggil sama sekali (ramp-down memanggil PWM_SetDutyCycle()
        // langsung), jadi PB8 tidak akan tersentuh sampai link balik - tanpa
        // baris ini sistem tetap "armed" padahal link sudah mati. Jalur CAN di
        // control_demolition_robot menurunkan PB8 lewat cabang s0==0; ini
        // menyamakan perilakunya. Saat link balik Control_Update() menaikkannya
        // lagi kalau s0 masih 1, jadi tidak ada lockout.
        GPIOB->BSRR = (1<<(8+16));  // BR8 = reset PB8 to LOW

        // Sinyal putus: motor tidak boleh hidup sendiri saat link balik.
        // Operator wajib start ulang dari remote (S2_1 OFF lalu ON).
        Control_RequireMotorRestart();
      }
    }

    // ========================================================================
    // SAFETY TRANSITION: Continue smooth transition if in safety mode
    // ========================================================================
    if (safety_mode_active)
    {

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
