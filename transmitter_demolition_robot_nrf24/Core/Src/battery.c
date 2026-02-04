/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : battery.c
  * @brief          : Battery voltage monitoring implementation
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca tegangan baterai 7.2V (6 cells NiMH/NiCd)
  * Pin R1 (PA1 - ADC_CHANNEL_1) terhubung ke voltage divider 10kΩ + 20kΩ
  *
  * Voltage divider calculation:
  * Vout = Vin × (10k / (10k + 20k)) = Vin / 3
  *
  * Example:
  * - Battery 7.2V → Vout = 2.4V → ADC = 2982 (12-bit)
  * - Battery 8.4V → Vout = 2.8V → ADC = 3479
  * - Battery 6.0V → Vout = 2.0V → ADC = 2482
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "battery.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ADC_MAX_VALUE       4095    // 12-bit ADC maximum value
#define ADC_VREF_MV         3300    // ADC reference voltage in millivolts (3.3V)
#define ADC_TIMEOUT         100     // ADC conversion timeout in milliseconds
#define ADC_SAMPLES         32      // Number of samples to average (increased for stability)
#define FILTER_SIZE         16      // Moving average filter size (increased for stability)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t filter_buffer[FILTER_SIZE] = {0};
static uint8_t filter_index = 0;
static uint8_t filter_filled = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize battery monitoring module
  * @retval None
  */
void Battery_Init(void)
{
    // ADC already initialized in MX_ADC1_Init()
    // No additional initialization needed
}

/**
  * @brief  Read battery ADC value from R1 pin (PA1 - ADC_CHANNEL_1)
  * @note   Uses averaging of multiple samples for stable reading
  * @retval 12-bit ADC value (0-4095)
  */
uint16_t Battery_ReadADC(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t adc_sum = 0;
    uint16_t adc_average = 0;

    // Configure ADC channel for R1 (PA1 - ADC_CHANNEL_1)
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;  // Longer sampling for high impedance (30kΩ)

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK)
    {
        // Read multiple samples and average them to reduce noise
        for (uint8_t i = 0; i < ADC_SAMPLES; i++)
        {
            // Start conversion
            HAL_ADC_Start(&hadc1);

            // Wait for conversion to complete
            if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT) == HAL_OK)
            {
                // Accumulate ADC value
                adc_sum += HAL_ADC_GetValue(&hadc1);
            }

            // Stop ADC
            HAL_ADC_Stop(&hadc1);
        }

        // Calculate average
        adc_average = (uint16_t)(adc_sum / ADC_SAMPLES);

        // Apply moving average filter for even smoother reading
        filter_buffer[filter_index] = adc_average;
        filter_index = (filter_index + 1) % FILTER_SIZE;

        if (!filter_filled && filter_index == 0)
        {
            filter_filled = 1;  // Buffer is now full
        }

        // Calculate moving average
        uint32_t filter_sum = 0;
        uint8_t count = filter_filled ? FILTER_SIZE : filter_index;

        for (uint8_t i = 0; i < count; i++)
        {
            filter_sum += filter_buffer[i];
        }

        adc_average = (uint16_t)(filter_sum / count);
    }

    return adc_average;
}

/**
  * @brief  Get battery voltage in millivolts
  * @retval Battery voltage in mV (e.g., 7200 for 7.2V)
  */
uint16_t Battery_GetVoltage_mV(void)
{
    uint16_t adc_value = Battery_ReadADC();

    // Calculate voltage at ADC pin (Vout)
    // Vout (mV) = (adc_value × Vref) / ADC_MAX
    uint32_t vout_mv = ((uint32_t)adc_value * ADC_VREF_MV) / ADC_MAX_VALUE;

    // Calculate actual battery voltage (Vin = Vout × 3)
    uint16_t battery_mv = (uint16_t)(vout_mv * VOLTAGE_DIVIDER_RATIO);

    return battery_mv;
}

/**
  * @brief  Get battery percentage (0-100%)
  * @note   Based on voltage range: 6.0V (0%) to 8.4V (100%)
  * @retval Battery percentage (0-100)
  */
uint8_t Battery_GetPercentage(void)
{
    uint16_t voltage_mv = Battery_GetVoltage_mV();

    // Clamp voltage to valid range
    if (voltage_mv >= BATTERY_FULL_MV)
    {
        return 100;  // Fully charged
    }
    else if (voltage_mv <= BATTERY_CRITICAL_MV)
    {
        return 0;    // Battery critical
    }

    // Calculate percentage using linear interpolation
    // Range: 6.0V (0%) to 8.4V (100%)
    // Percentage = ((voltage - 6000) / (8400 - 6000)) × 100
    uint16_t voltage_range = BATTERY_FULL_MV - BATTERY_CRITICAL_MV;  // 2400mV
    uint16_t voltage_above_min = voltage_mv - BATTERY_CRITICAL_MV;

    uint8_t percentage = (uint8_t)((voltage_above_min * 100) / voltage_range);

    return percentage;
}

/**
  * @brief  Check if battery is low (below 6.6V)
  * @retval 1 if battery is low, 0 otherwise
  */
uint8_t Battery_IsLow(void)
{
    return (Battery_GetVoltage_mV() < BATTERY_LOW_MV) ? 1 : 0;
}

/**
  * @brief  Check if battery is critical (below 6.0V)
  * @retval 1 if battery is critical, 0 otherwise
  */
uint8_t Battery_IsCritical(void)
{
    return (Battery_GetVoltage_mV() < BATTERY_CRITICAL_MV) ? 1 : 0;
}
