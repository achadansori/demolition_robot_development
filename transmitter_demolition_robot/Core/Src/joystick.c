/**
  ******************************************************************************
  * @file           : joystick.c
  * @brief          : Joystick and ADC reading implementation (DMA Mode)
  *                   STM32F407 Discovery Transmitter
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca semua input analog via DMA:
  * - 2 Joystick (masing-masing 2 axis)
  * - 2 Potentiometer (R1, R8)
  *
  * DMA Buffer Order (sesuai Rank di CubeMX):
  * [0] = joy_left_y  (PC1 - IN11)
  * [1] = joy_left_x  (PC3 - IN13)
  * [2] = joy_right_y (PA5 - IN5)
  * [3] = joy_right_x (PA7 - IN7)
  * [4] = battery     (PA0 - IN0) - via voltage divider
  *
  * PA2 dihapus dari scan untuk menghilangkan cross-talk floating-pin noise.
  * Data dikonversi dari 12-bit (0-4095) ke 8-bit (0-255) untuk efisiensi.
  * Battery dikonversi langsung ke persen (0-100) di sini.
  * Joystick axes diproteksi dengan median-of-3 glitch filter.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "joystick.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define ADC_MAX_VALUE 4095  // 12-bit ADC maximum value

/* Joystick wiring-fault (plausibility) detection ------------------------------
 * A broken wiper wire or a short reads the EXACT ADC rail (0 or 4095) with no
 * noise. A real pot at full mechanical deflection has series resistance and
 * ADC noise, so it does not sit pinned within 0.2% of the rail for long.
 * If an axis stays hard at a rail for JOY_FAULT_SET_MS it is declared faulted
 * and forced to neutral (127) so a broken wire cannot command full speed.
 * Any single sample off the rail resets the fault timer (legit full stick with
 * normal ADC noise will keep resetting it). Set JOY_FAULT_DETECT_ENABLED 0 to
 * disable if your pots really reach the exact rails in normal use. */
#define JOY_FAULT_DETECT_ENABLED  1
#define JOY_FAULT_RAIL_LOW        8U     /* raw12 <= this -> at GND rail  */
#define JOY_FAULT_RAIL_HIGH       4087U  /* raw12 >= this -> at VDD rail  */
#define JOY_FAULT_SET_MS          400U   /* rail held this long -> fault  */
#define JOY_FAULT_CLEAR_MS        200U   /* plausible this long -> clear  */

/* Battery measurement configuration (PA0 via voltage divider)
 * Default: 2S Li-ion / LiPo (6.0V - 8.4V), 3:1 divider (R_top=20k, R_bot=10k)
 * Adjust these to match your actual battery + resistor values.
 *
 *   V_pa0 = V_battery * R_BOTTOM / (R_TOP + R_BOTTOM)
 *   V_battery = ADC * VREF_MV / 4095 * (R_TOP + R_BOTTOM) / R_BOTTOM
 */
#define BATTERY_VREF_MV          3300U   // MCU ADC reference (3.3V)
#define BATTERY_DIVIDER_R_TOP    20000U  // Top resistor (battery+ to PA2)  [ohm]
#define BATTERY_DIVIDER_R_BOTTOM 10000U  // Bottom resistor (PA2 to GND)    [ohm]
#define BATTERY_MIN_MV           6000U   // 0% battery threshold  (mV)
#define BATTERY_MAX_MV           8400U   // 100% battery threshold (mV)

/* Battery smoothing:
 *   EMA alpha = 1/64 -> ~64 samples time-constant (heavy filter against ADC noise)
 *   Hysteresis = 2% -> only update displayed percent if change >= 2 points
 */
#define BATTERY_EMA_SHIFT        6U      // alpha = 1/(2^6) = 1/64
#define BATTERY_HYSTERESIS_PCT   2U

/* DMA Buffer - Filled automatically by DMA */
uint16_t adc_buffer[ADC_CHANNELS] = {0};

/* Calibration center values (raw 8-bit at neutral position) */
int8_t joy_cal_offset[4] = {0, 0, 0, 0};  // Offset from 127 per axis [LX, LY, RY, RX]

/* Private variables ---------------------------------------------------------*/
static uint8_t dma_started = 0;

/**
  * @brief  Apply asymmetric calibration to joystick axis
  *         Maps raw value so that calibrated center = 127
  *         Does not reduce max range (0 and 255 still reachable)
  * @param  raw8: Raw 8-bit ADC value (0-255)
  * @param  offset: Calibration offset (raw_center - 127)
  * @retval Calibrated 8-bit value (0-255), center = 127
  */
/**
  * @brief  Convert raw ADC reading on PA0 to battery percentage (0-100%)
  *         Applies EMA filter + hysteresis to suppress ADC noise jitter.
  * @param  adc_raw: 12-bit ADC value from voltage divider on PA0
  * @retval Battery percentage clamped to 0-100, smoothed
  */
static uint8_t calculate_battery_percent(uint16_t adc_raw)
{
    // EMA filter (preserved across calls): acc = acc - (acc>>k) + sample
    static uint32_t ema_acc = 0;
    static uint8_t  ema_init = 0;
    static uint8_t  last_pct = 0;

    if (!ema_init)
    {
        ema_acc  = (uint32_t)adc_raw << BATTERY_EMA_SHIFT;
        ema_init = 1;
    }
    else
    {
        ema_acc = ema_acc - (ema_acc >> BATTERY_EMA_SHIFT) + adc_raw;
    }
    uint16_t filtered = (uint16_t)(ema_acc >> BATTERY_EMA_SHIFT);

    // Convert filtered ADC to battery voltage (mV)
    uint32_t v_pin_mv = (uint32_t)filtered * BATTERY_VREF_MV / ADC_MAX_VALUE;
    uint32_t v_battery_mv = v_pin_mv * (BATTERY_DIVIDER_R_TOP + BATTERY_DIVIDER_R_BOTTOM)
                                     / BATTERY_DIVIDER_R_BOTTOM;

    uint8_t pct;
    if      (v_battery_mv <= BATTERY_MIN_MV) pct = 0;
    else if (v_battery_mv >= BATTERY_MAX_MV) pct = 100;
    else pct = (uint8_t)((v_battery_mv - BATTERY_MIN_MV) * 100U
                         / (BATTERY_MAX_MV - BATTERY_MIN_MV));

    // Hysteresis: only update displayed value if change is significant
    int16_t diff = (int16_t)pct - (int16_t)last_pct;
    if (diff >= (int16_t)BATTERY_HYSTERESIS_PCT || diff <= -(int16_t)BATTERY_HYSTERESIS_PCT
        || pct == 0 || pct == 100)
    {
        last_pct = pct;
    }
    return last_pct;
}

/**
  * @brief  Median-of-3 filter to reject single-sample ADC spikes.
  *         Keeps last 2 samples per axis, returns median of {prev2, prev1, curr}.
  * @param  axis: Axis index (0=left_x, 1=left_y, 2=right_y, 3=right_x)
  * @param  curr: Current raw 8-bit sample
  * @retval Median-filtered 8-bit value
  */
static uint8_t median3_filter(uint8_t axis, uint8_t curr)
{
    static uint8_t prev1[4] = {127, 127, 127, 127};
    static uint8_t prev2[4] = {127, 127, 127, 127};

    uint8_t a = prev2[axis];
    uint8_t b = prev1[axis];
    uint8_t c = curr;

    // Shift history
    prev2[axis] = prev1[axis];
    prev1[axis] = curr;

    // Sort {a, b, c} ascending and return middle (median)
    uint8_t t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    return b;
}

/**
  * @brief  Detect a wiring fault on one joystick axis (value pinned at an ADC
  *         rail). Returns 1 while the axis is considered faulted.
  * @param  axis:  Axis index 0-3
  * @param  raw12: Raw 12-bit ADC sample for this axis
  * @retval 1 = axis faulted (force neutral), 0 = axis healthy
  */
static uint8_t joystick_axis_faulted(uint8_t axis, uint16_t raw12)
{
#if JOY_FAULT_DETECT_ENABLED
    static uint32_t rail_since[4]  = {0, 0, 0, 0};  /* 0 = not at rail   */
    static uint32_t clear_since[4] = {0, 0, 0, 0};  /* 0 = not recovering */
    static uint8_t  faulted[4]     = {0, 0, 0, 0};

    uint32_t now = HAL_GetTick();
    uint8_t at_rail = (raw12 <= JOY_FAULT_RAIL_LOW) || (raw12 >= JOY_FAULT_RAIL_HIGH);

    if (at_rail)
    {
        clear_since[axis] = 0;
        if (rail_since[axis] == 0)
        {
            rail_since[axis] = now | 1u;  /* |1 so the timestamp is never 0 */
        }
        else if (!faulted[axis] && (now - rail_since[axis]) >= JOY_FAULT_SET_MS)
        {
            faulted[axis] = 1;
        }
    }
    else
    {
        rail_since[axis] = 0;
        if (faulted[axis])
        {
            if (clear_since[axis] == 0)
            {
                clear_since[axis] = now | 1u;
            }
            else if ((now - clear_since[axis]) >= JOY_FAULT_CLEAR_MS)
            {
                faulted[axis] = 0;
                clear_since[axis] = 0;
            }
        }
    }

    return faulted[axis];
#else
    (void)axis; (void)raw12;
    return 0;
#endif
}

static uint8_t apply_calibration(uint8_t raw8, int8_t offset)
{
    if (offset == 0) return raw8;

    uint8_t center = (uint8_t)(127 + offset);  // Hardware neutral in 8-bit

    if (raw8 == center) return 127;

    if (raw8 < center)
    {
        // Map [0, center] → [0, 127]
        if (center == 0) return 0;
        return (uint8_t)((uint16_t)raw8 * 127 / center);
    }
    else
    {
        // Map [center, 255] → [127, 255]
        uint8_t range = 255 - center;
        if (range == 0) return 255;
        return (uint8_t)(127 + (uint16_t)(raw8 - center) * 128 / range);
    }
}

/**
  * @brief  Initialize joystick module and start DMA
  * @retval None
  */
void Joystick_Init(void)
{
    // Start ADC DMA
    Joystick_StartDMA();
}

/**
  * @brief  Start ADC DMA conversion
  * @retval None
  */
void Joystick_StartDMA(void)
{
    if (!dma_started)
    {
        // Start ADC with DMA (Circular mode - runs continuously)
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS);
        dma_started = 1;
    }
}

/**
  * @brief  Read all joystick and potentiometer data from DMA buffer
  * @param  data: Pointer to Joystick_Data_t structure
  * @retval None
  */
void Joystick_Read(Joystick_Data_t* data)
{
    // Make sure DMA is running
    if (!dma_started)
    {
        Joystick_StartDMA();
    }

    // Read from DMA buffer and convert 12-bit to 8-bit
    // DMA Buffer Order:
    // [0] = joy_left_y  (PC1 - IN11)
    // [1] = joy_left_x  (PC3 - IN13)
    // [2] = joy_right_y (PA5 - IN5)
    // [3] = joy_right_x (PA7 - IN7)
    // [4] = battery     (PA0 - IN0) via voltage divider
    // (PA2 not in scan anymore - removed to kill cross-talk noise)

    // Snapshot the DMA buffer once (12-bit) for fault detection + conversion
    uint16_t raw12[4];
    raw12[0] = adc_buffer[0];
    raw12[1] = adc_buffer[1];
    raw12[2] = adc_buffer[2];
    raw12[3] = adc_buffer[3];

    // Convert 12-bit to 8-bit, apply median-of-3 glitch filter, then calibration
    uint8_t raw_lx = (uint8_t)(raw12[0] >> 4);
    uint8_t raw_ly = (uint8_t)(raw12[1] >> 4);
    uint8_t raw_ry = (uint8_t)(raw12[2] >> 4);
    uint8_t raw_rx = (uint8_t)(raw12[3] >> 4);

    data->left_x          = apply_calibration(median3_filter(0, raw_lx), joy_cal_offset[0]);
    data->left_y          = apply_calibration(median3_filter(1, raw_ly), joy_cal_offset[1]);
    data->right_y         = apply_calibration(median3_filter(2, raw_ry), joy_cal_offset[2]);
    data->right_x         = apply_calibration(median3_filter(3, raw_rx), joy_cal_offset[3]);
    data->battery_percent = calculate_battery_percent(adc_buffer[4]);  // PA0

    // Wiring-fault protection: an axis pinned at an ADC rail (broken wiper /
    // short) is forced to neutral so it cannot command full-speed motion.
    if (joystick_axis_faulted(0, raw12[0])) data->left_x  = 127;
    if (joystick_axis_faulted(1, raw12[1])) data->left_y  = 127;
    if (joystick_axis_faulted(2, raw12[2])) data->right_y = 127;
    if (joystick_axis_faulted(3, raw12[3])) data->right_x = 127;
}

/**
  * @brief  Calibrate joystick by reading current position as neutral (center)
  *         Call this when joystick is at rest position
  *         After calibration, neutral = 127 (0%), full range preserved
  * @retval None
  */
void Joystick_Calibrate(void)
{
    // Read current raw 8-bit values at neutral position
    uint8_t raw[4];
    raw[0] = (uint8_t)(adc_buffer[0] >> 4);  // left_x
    raw[1] = (uint8_t)(adc_buffer[1] >> 4);  // left_y
    raw[2] = (uint8_t)(adc_buffer[2] >> 4);  // right_y
    raw[3] = (uint8_t)(adc_buffer[3] >> 4);  // right_x

    // Calculate offset: how far hardware neutral is from 127
    for (int i = 0; i < 4; i++)
    {
        joy_cal_offset[i] = (int8_t)((int16_t)raw[i] - 127);
    }
}
