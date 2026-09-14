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
 *   Display rate limit -> the shown value is re-latched at most once every
 *   BATTERY_UPDATE_INTERVAL_MS, so the OLED percent/bar changes slowly enough
 *   to be read by eye. The EMA keeps integrating every sample in between.
 */
#define BATTERY_EMA_SHIFT           6U     // alpha = 1/(2^6) = 1/64
#define BATTERY_HYSTERESIS_PCT      2U
#define BATTERY_UPDATE_INTERVAL_MS  2000U  // displayed value updates max 1x per 2 s

/* DMA Buffer - Filled automatically by DMA */
/* volatile: ditulis oleh hardware DMA, dibaca oleh kode C. Tanpa ini, di -O2
 * compiler boleh meng-cache atau mengangkat pembacaannya keluar loop. */
volatile uint16_t adc_buffer[ADC_CHANNELS] = {0};

/* Calibration center values (raw 8-bit at neutral position) */
int8_t joy_cal_offset[4] = {0, 0, 0, 0};  // Offset from 127 per axis [LX, LY, RY, RX]

/* Private variables ---------------------------------------------------------*/
static uint8_t dma_started = 0;

/* 1 = konversi ADC tidak berjalan, isi adc_buffer TIDAK boleh dipercaya.
 *
 * Dulu HAL_ADC_Start_DMA() dipanggil sekali, return-nya dibuang, dan
 * dma_started dikunci ke 1 selamanya. Kalau DMA batal di tengah jalan
 * (overrun ADC adalah penyebab paling umum - OVR menghentikan stream),
 * adc_buffer BEKU di nilai terakhirnya tanpa batas waktu dan tidak ada satu
 * pun yang mendeteksinya: joystick macet diam-diam, dan kalau bekunya saat
 * stik terdefleksi, robot terus bergerak sementara link terlihat sehat
 * sempurna. Ini satu-satunya jalur yang bisa menggerakkan mesin tanpa gejala.
 */
static uint8_t dma_fault = 0;
static uint32_t dma_restart_count = 0;

/* Persentase baterai terakhir yang valid, dipakai saat ADC sedang fault
 * supaya tampilan tidak melonjak ke 0%. */
static uint8_t last_battery_percent = 0;

/* ---------------------------------------------------------------------------
 * Penyimpanan kalibrasi di flash (APPEND-ONLY)
 *
 * Dulu joy_cal_offset[] hanya ada di RAM: hilang tiap reset, jadi operator
 * harus kalibrasi ulang tiap kali remote dinyalakan.
 *
 * Kenapa append-only dan bukan "hapus lalu tulis"? Sektor flash F407 di atas
 * sektor 4 semuanya 128 KB, dan erase 128 KB makan tipikal 1 detik, maksimum
 * 3 detik. IWDG kita 800 ms, jadi satu erase = watchdog reset di tengah
 * penyimpanan. Flash hanya bisa mengubah bit 1 -> 0, jadi slot kosong bisa
 * ditulisi tanpa erase: tiap kalibrasi mengisi slot 8 byte berikutnya.
 * 128 KB / 8 = 16384 kali kalibrasi sebelum sektornya penuh - praktis tidak
 * pernah tercapai, sehingga erase tidak pernah dijalankan.
 *
 * Urutan tulis: offsets DULU, magic BELAKANGAN. Kalau daya putus di tengah,
 * slot itu tidak punya magic dan otomatis diabaikan - tidak ada record
 * setengah jadi yang terbaca sebagai kalibrasi valid.
 *
 * ponytail: kalau sektornya benar-benar penuh, penyimpanan berhenti diam-diam
 * dan kalibrasi kembali seperti dulu (hanya RAM). Kalau itu sampai terjadi,
 * tambahkan erase sektor di jalur boot (di sana blocking 3 detik aman karena
 * IWDG belum jalan).
 * ------------------------------------------------------------------------ */
#define CAL_FLASH_BASE     0x080E0000UL       /* sektor 11, 128 KB, di luar program (~67 KB) */
#define CAL_FLASH_SIZE     (128U * 1024U)
#define CAL_SLOT_SIZE      8U
#define CAL_MAGIC          0xCA11B00BUL

/**
  * @brief  Alamat slot terpakai TERAKHIR, atau 0 kalau belum ada
  */
static uint32_t cal_find_last(void)
{
    uint32_t last = 0;

    for (uint32_t a = CAL_FLASH_BASE; a < CAL_FLASH_BASE + CAL_FLASH_SIZE; a += CAL_SLOT_SIZE)
    {
        if (*(volatile uint32_t *)(a + 4) == CAL_MAGIC)
        {
            last = a;
        }
        else if (*(volatile uint32_t *)(a + 4) == 0xFFFFFFFFUL &&
                 *(volatile uint32_t *)a       == 0xFFFFFFFFUL)
        {
            break;   /* slot kosong pertama: sisanya pasti kosong juga */
        }
    }
    return last;
}

/**
  * @brief  Muat kalibrasi tersimpan ke joy_cal_offset[] kalau ada
  */
static void cal_load(void)
{
    uint32_t slot = cal_find_last();
    if (slot == 0) return;                    /* belum pernah dikalibrasi */

    uint32_t packed = *(volatile uint32_t *)slot;
    for (int i = 0; i < 4; i++)
    {
        joy_cal_offset[i] = (int8_t)((packed >> (i * 8)) & 0xFFU);
    }
}

/**
  * @brief  Simpan joy_cal_offset[] ke slot flash kosong berikutnya
  */
static void cal_save(void)
{
    /* Cari slot yang BENAR-BENAR kosong (kedua word masih 0xFFFFFFFF), bukan
     * sekadar "sesudah yang terakhir valid". Slot yang tertulis separuh karena
     * daya putus saat menyimpan tidak punya magic, dan menimpanya akan gagal:
     * flash tidak bisa mengembalikan bit 0 menjadi 1. Dengan cara ini slot
     * rusak itu cukup dilewati. */
    uint32_t slot = 0;
    for (uint32_t a = CAL_FLASH_BASE; a < CAL_FLASH_BASE + CAL_FLASH_SIZE; a += CAL_SLOT_SIZE)
    {
        if (*(volatile uint32_t *)a       == 0xFFFFFFFFUL &&
            *(volatile uint32_t *)(a + 4) == 0xFFFFFFFFUL)
        {
            slot = a;
            break;
        }
    }

    if (slot == 0) return;   /* penuh - lihat catatan ponytail di atas */

    uint32_t packed = 0;
    for (int i = 0; i < 4; i++)
    {
        packed |= ((uint32_t)(uint8_t)joy_cal_offset[i]) << (i * 8);
    }

    if (HAL_FLASH_Unlock() != HAL_OK) return;

    /* offsets dulu, magic terakhir - lihat catatan torn-write di atas */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, slot, packed) == HAL_OK)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, slot + 4, CAL_MAGIC);
    }

    HAL_FLASH_Lock();
}

/**
  * @brief  Apakah konversi ADC benar-benar masih jalan?
  * @note   Bit EN stream ikut ter-clear saat DMA batal, dan flag OVR menandai
  *         overrun yang menghentikannya. Memeriksa "apakah nilainya berubah"
  *         tidak bisa dipakai: stik yang benar-benar diam memang tidak berubah.
  * @retval 1 kalau sehat
  */
static uint8_t dma_is_running(void)
{
    if (hadc1.DMA_Handle == NULL) return 0;
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR)) return 0;

    return ((hadc1.DMA_Handle->Instance->CR & DMA_SxCR_EN) != 0U) ? 1 : 0;
}

/**
  * @brief  Hentikan, bersihkan, lalu jalankan ulang ADC+DMA
  * @retval None
  */
static void dma_restart(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
    dma_started = 0;
    dma_restart_count++;
    Joystick_StartDMA();
}

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
    static uint32_t last_update_ms = 0;

    if (!ema_init)
    {
        ema_acc  = (uint32_t)adc_raw << BATTERY_EMA_SHIFT;
        ema_init = 1;
    }
    else
    {
        ema_acc = ema_acc - (ema_acc >> BATTERY_EMA_SHIFT) + adc_raw;
    }

    // Rate limit: between update windows just hold the last shown value so
    // the percent/bar on the OLED is stable long enough to actually read.
    // (ema_init doubles as "first call": latch immediately so the display
    // shows a real value right after boot instead of 0% for 2 seconds.)
    uint32_t now = HAL_GetTick();
    if (last_update_ms != 0 && (now - last_update_ms) < BATTERY_UPDATE_INTERVAL_MS)
    {
        return last_pct;
    }
    last_update_ms = now | 1u;  // |1 so the timestamp is never 0

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
    // Pulihkan kalibrasi tersimpan sebelum ADC jalan, supaya pembacaan pertama
    // sudah terkalibrasi dan operator tidak perlu kalibrasi ulang tiap nyala.
    cal_load();

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
        // Start ADC with DMA (Circular mode - runs continuously).
        // Return-nya DIPERIKSA: kalau gagal, dma_started tidak dikunci ke 1
        // sehingga Joystick_Read() mencobanya lagi di iterasi berikutnya.
        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS) == HAL_OK)
        {
            dma_started = 1;
            dma_fault = 0;
        }
        else
        {
            dma_fault = 1;
        }
    }
}

/**
  * @brief  Apakah pembacaan joystick boleh dipercaya?
  * @retval 1 kalau ADC sehat, 0 kalau data beku / DMA sedang bermasalah
  */
uint8_t Joystick_IsHealthy(void)
{
    return (uint8_t)(!dma_fault);
}

/**
  * @brief  Berapa kali ADC+DMA harus dijalankan ulang sejak boot (diagnostik)
  */
uint32_t Joystick_GetDmaRestarts(void)
{
    return dma_restart_count;
}

/**
  * @brief  Read all joystick and potentiometer data from DMA buffer
  * @param  data: Pointer to Joystick_Data_t structure
  * @retval None
  */
void Joystick_Read(Joystick_Data_t* data)
{
    // Pastikan konversi ADC benar-benar jalan, bukan sekadar "pernah distart".
    if (!dma_started)
    {
        Joystick_StartDMA();
    }
    else if (!dma_is_running())
    {
        dma_fault = 1;
        dma_restart();
    }

    // Data beku tidak boleh dikirim sebagai perintah. Netralkan keempat sumbu:
    // robot berhenti, dan kalau DMA pulih di iterasi berikutnya pembacaan
    // normal langsung kembali. Baterai dipertahankan di nilai terakhir supaya
    // tampilan tidak melonjak ke 0%.
    if (dma_fault)
    {
        data->left_x   = 127;
        data->left_y   = 127;
        data->right_x  = 127;
        data->right_y  = 127;
        data->battery_percent = last_battery_percent;
        data->reserved = 0;
        return;
    }

    // Read from DMA buffer and convert 12-bit to 8-bit
    // DMA Buffer Order:
    // [0] = joy_left_y  (PC1 - IN11)
    // [1] = joy_left_x  (PC3 - IN13)
    // [2] = joy_right_y (PA5 - IN5)
    // [3] = joy_right_x (PA7 - IN7)
    // [4] = battery     (PA0 - IN0) via voltage divider
    // (PA2 not in scan anymore - removed to kill cross-talk noise)

    // Convert 12-bit to 8-bit, apply median-of-3 glitch filter, then calibration
    uint8_t raw_lx = (uint8_t)(adc_buffer[0] >> 4);
    uint8_t raw_ly = (uint8_t)(adc_buffer[1] >> 4);
    uint8_t raw_ry = (uint8_t)(adc_buffer[2] >> 4);
    uint8_t raw_rx = (uint8_t)(adc_buffer[3] >> 4);

    data->left_x          = apply_calibration(median3_filter(0, raw_lx), joy_cal_offset[0]);
    data->left_y          = apply_calibration(median3_filter(1, raw_ly), joy_cal_offset[1]);
    data->right_y         = apply_calibration(median3_filter(2, raw_ry), joy_cal_offset[2]);
    data->right_x         = apply_calibration(median3_filter(3, raw_rx), joy_cal_offset[3]);
    data->battery_percent = calculate_battery_percent(adc_buffer[4]);  // PA0
    last_battery_percent  = data->battery_percent;
    data->reserved        = 0;  // PA2 removed from scan
}

/**
  * @brief  Calibrate joystick by reading current position as neutral (center)
  *         Call this when joystick is at rest position
  *         After calibration, neutral = 127 (0%), full range preserved
  * @retval None
  */
void Joystick_Calibrate(void)
{
    // Jangan kalibrasi dari data beku.
    if (dma_fault) return;

    // Baca nilai raw 8-bit di posisi netral, LEWAT median filter.
    //
    // Versi lama membaca adc_buffer langsung, melewati median3_filter yang
    // justru ada untuk menolak spike. Satu spike ADC tepat pada saat kalibrasi
    // membiaskan sumbu itu secara permanen.
    uint8_t raw[4];
    raw[0] = median3_filter(0, (uint8_t)(adc_buffer[0] >> 4));  // left_x
    raw[1] = median3_filter(1, (uint8_t)(adc_buffer[1] >> 4));  // left_y
    raw[2] = median3_filter(2, (uint8_t)(adc_buffer[2] >> 4));  // right_y
    raw[3] = median3_filter(3, (uint8_t)(adc_buffer[3] >> 4));  // right_x

    // Calculate offset: how far hardware neutral is from 127
    for (int i = 0; i < 4; i++)
    {
        joy_cal_offset[i] = (int8_t)((int16_t)raw[i] - 127);
    }

    cal_save();   // simpan ke flash supaya tidak hilang saat daya dimatikan
}
