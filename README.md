# Demolition Robot — Remote Control System (STM32F407)

Sistem kendali jarak jauh untuk **robot demolisi hidrolik** (mini excavator / demolition machine) berbasis tiga board **STM32F407VGT6 (STM32F407G-DISC1 Discovery)**.

Operator memegang **remote** (transmitter) dengan 2 joystick analog + 13 switch. Perintah dikirim lewat radio **NRF24L01+ (2.4 GHz)** ke **bridge** di robot, lalu diteruskan lewat **CAN bus (CANopen / CiA-301)** ke **control board** yang menggerakkan **20 kanal PWM** untuk katup proporsional hidrolik (silinder, slew, track, outrigger, breaker) plus relay emergency dan motor starter.

```
   ┌──────────────────────┐         NRF24L01+          ┌──────────────────────┐        CAN 500 kbps       ┌──────────────────────┐
   │  TRANSMITTER (remote)│  ~~~~~~ 2.476 GHz ~~~~~~>  │  RECEIVER (bridge)   │ ====== CANopen ========>  │  CONTROL (robot)     │
   │  STM32F407 Discovery │   8-byte payload, Auto-ACK │  STM32F407 Discovery │   TPDO1 (COB-ID 0x181)    │  STM32F407 Discovery │
   │                      │                            │                      │   tiap 50 ms              │                      │
   │  2x joystick + 13 sw │                            │  Node-ID 1           │                           │  Node-ID 2           │
   │  OLED 128x64         │                            │  NRF24 -> CAN        │                           │  20x PWM + 2 relay   │
   │  battery monitor     │                            │                      │                           │  katup hidrolik      │
   └──────────────────────┘                            └──────────────────────┘                           └──────────────────────┘
```

---

## Daftar Isi

1. [Struktur Repository](#1-struktur-repository)
2. [Arsitektur Sistem](#2-arsitektur-sistem)
3. [Protokol Paket Data (8 byte)](#3-protokol-paket-data-8-byte)
4. [Pin Mapping — Transmitter](#4-pin-mapping--transmitter-remote)
5. [Pin Mapping — Receiver / Bridge](#5-pin-mapping--receiver--bridge)
6. [Pin Mapping — Control Board](#6-pin-mapping--control-board-robot)
7. [Daftar 20 Kanal PWM & Limitnya](#7-daftar-20-kanal-pwm--limitnya)
8. [Cara Penggunaan Remote (Panduan Operator)](#8-cara-penggunaan-remote-panduan-operator)
9. [Peta Kontrol per Mode](#9-peta-kontrol-per-mode)
10. [Tampilan OLED](#10-tampilan-oled)
11. [Sistem Keselamatan (Failsafe)](#11-sistem-keselamatan-failsafe)
12. [Build & Flash](#12-build--flash)
13. [Debugging via USB CDC](#13-debugging-via-usb-cdc)
14. [Troubleshooting](#14-troubleshooting)

---

## 1. Struktur Repository

| Folder | Board | Fungsi |
|---|---|---|
| `transmitter_demolition_robot/` | Remote di tangan operator | Baca joystick + switch, tampilkan status di OLED, kirim paket 8 byte via NRF24 |
| `receiver_demolition_robot/` | Bridge di robot | Terima paket NRF24, publikasikan 8 byte yang sama ke CAN bus sebagai CANopen **Node-ID 1** (TPDO1) |
| `control_demolition_robot/` | Control board di robot | Konsumsi RPDO1 sebagai CANopen **Node-ID 2**, decode paket, hasilkan 20 kanal PWM + relay |

Ketiganya project **STM32CubeIDE** (`.ioc` + HAL). Folder `Debug/` dan `Release/` di-*gitignore* (hanya artefak build).

---

## 2. Arsitektur Sistem

**Kenapa dipecah jadi 3 board?**
Dulu NRF24 langsung menempel di control board. Sekarang jalur radio dipisah dari jalur daya/PWM: bridge menerima radio lalu meneruskan lewat CAN. Kabel CAN jauh lebih tahan noise dari switching solenoid hidrolik dibanding SPI, dan control board bisa ditempatkan dekat blok katup tanpa menarik antena ke sana.

**Rantai data:**

1. **Transmitter** — loop bebas delay: baca ADC (DMA) + GPIO switch → kemas jadi 8 byte → `NRF24_SendBinary()` dengan Auto-ACK.
2. **Receiver (bridge)** — `NRF24_GetRawPayload()` → salin 8 byte ke Object Dictionary `0x2000` → CANopen **TPDO1** auto-transmit tiap **50 ms** (event timer).
3. **Control** — **RPDO1** (COB-ID `0x181`) menulis `OD 0x2000` → `ctrl_decode()` → `Control_Update()` → `PWM_SetDutyCycle()`.

**Parameter link:**

| Parameter | Nilai |
|---|---|
| Frekuensi RF | Channel 76 = **2476 MHz** (di luar mayoritas kanal WiFi) |
| Alamat NRF24 | `E7 E7 E7 E7 E7` (5 byte, harus sama di TX & RX) |
| Data rate RF | 1 Mbps, daya 0 dBm |
| Auto-ACK | Aktif pipe 0, retry 2x, delay 500 µs → dipakai untuk hitung **link quality** |
| Payload | 8 byte, fixed |
| CAN bitrate | **500 kbps** (`PD0`=RX, `PD1`=TX, butuh transceiver mis. SN65HVD230 / MCP2551) |
| CANopen Node-ID | Bridge = **1**, Control = **2** |
| Heartbeat | Bridge produce 1000 ms; Control produce 200 ms (dipantau bridge untuk self-heal) |

Definisi bersama ada di `ctrl_link.h` (ada di project receiver & control — **kalau diubah, ubah di kedua-duanya**).

---

## 3. Protokol Paket Data (8 byte)

Layout identik di NRF24 dan di CAN (`Transmitter_Data_t` di `var.h`, `ctrl_link.h` di sisi robot):

| Byte | Isi | Range |
|---|---|---|
| 0 | `joy_left_x` — joystick kiri sumbu X | 0–255, tengah **127** |
| 1 | `joy_left_y` — joystick kiri sumbu Y | 0–255, tengah 127 |
| 2 | `joy_right_x` — joystick kanan sumbu X | 0–255, tengah 127 |
| 3 | `joy_right_y` — joystick kanan sumbu Y | 0–255, tengah 127 |
| 4 | `battery_percent` — baterai remote | 0–100 % |
| 5 | reserved | 0 |
| 6 | switch bit 0–7 | bitfield |
| 7 | switch bit 8–15 | bitfield |

**Bit switch (byte 7 << 8 | byte 6):**

| Bit | Nama | Arti |
|---|---|---|
| 0 | `joy_left_btn1` | tombol joystick kiri 1 |
| 1 | `joy_left_btn2` | tombol joystick kiri 2 → **trigger breaker** (mode UPPER) |
| 2 | `joy_right_btn1` | tombol joystick kanan 1 → **enable dual-track** (mode DUAL) |
| 3 | `joy_right_btn2` | tombol joystick kanan 2 → **shift ke Cylinder 1** (mode UPPER) |
| 4 | `s0` | **1 = normal, 0 = EMERGENCY STOP** |
| 5 | `s1_1` | hold → keluar dari SLEEP mode |
| 6 | `s1_2` | hold (saat SLEEP) → kalibrasi joystick |
| 7 | `s2_1` | hold → start motor (self-holding) |
| 8 | `s2_2` | cadangan |
| 9 | `s4_1` | cadangan |
| 10 | `s4_2` | cadangan |
| 11 | `s5_1` | **pemilih mode** |
| 12 | `s5_2` | **pemilih mode** |
| 13 | `motor_active` | status motor starter (dihasilkan transmitter, bukan switch langsung) |
| 14–15 | reserved | — |

> **Catatan:** byte 4 dulunya potensiometer R8 (flow breaker). Sekarang byte itu dipakai untuk persentase baterai remote, dan control board **memaksa `r8 = 0`** supaya katup flow breaker tidak ikut level baterai.

---

## 4. Pin Mapping — Transmitter (Remote)

Board: STM32F407G-DISC1. Clock: **84 MHz** (HSE 8 MHz + PLL, dengan fallback HSI kalau HSE gagal start).

### Input analog — ADC1 + DMA (5 kanal, scan mode, sampling 480 cycle)

| Sinyal | Pin | Kanal ADC | Urutan DMA |
|---|---|---|---|
| Joystick kiri Y | **PC1** | IN11 | `adc_buffer[0]` |
| Joystick kiri X | **PC3** | IN13 | `adc_buffer[1]` |
| Joystick kanan Y | **PA5** | IN5 | `adc_buffer[2]` |
| Joystick kanan X | **PA7** | IN7 | `adc_buffer[3]` |
| Baterai (voltage divider) | **PA0** | IN0 | `adc_buffer[4]` |

Pengolahan: 12-bit → 8-bit, **median-of-3 filter** (buang spike), lalu **kalibrasi offset** per sumbu.
PA2 sengaja **dikeluarkan dari scan** — pin mengambang di situ menimbulkan cross-talk yang membuat kanal lain terbaca 100%.

Baterai: divider 20 kΩ / 10 kΩ (3:1), rentang **6.0 V (0%) – 8.4 V (100%)** — cocok untuk 2S Li-ion/LiPo. Difilter EMA (α = 1/64) + histeresis 2% + rate-limit 2 detik. Ubah di `joystick.c` (`BATTERY_*`) kalau baterai/resistor berbeda.

### Input digital — switch & tombol (semua **active HIGH**, internal pull-down)

| Sinyal | Pin | Fungsi |
|---|---|---|
| `S0` | **PB0** (EXTI0) | **Emergency stop** — interrupt, prioritas tertinggi |
| `S1_1` | **PE4** | Hold ~0.1 s → keluar SLEEP mode |
| `S1_2` | **PE5** | Hold ~0.1 s (saat SLEEP) → kalibrasi joystick |
| `S2_1` | **PE1** | Hold ~0.1 s → start motor |
| `S2_2` | **PB8** | cadangan |
| `S4_1` | **PD6** | cadangan |
| `S4_2` | **PB3** | cadangan |
| `S5_1` | **PB5** | pemilih mode |
| `S5_2` | **PB7** | pemilih mode |
| `JOY_LEFT_BTN1` | **PA3** | tombol joystick kiri 1 |
| `JOY_LEFT_BTN2` | **PA1** | tombol joystick kiri 2 |
| `JOY_RIGHT_BTN1` | **PB1** | tombol joystick kanan 1 |
| `JOY_RIGHT_BTN2` | **PC5** | tombol joystick kanan 2 |

> **PE3 jangan dipakai untuk switch.** Di Discovery, PE3 adalah chip-select accelerometer LIS302DL/LIS3DSH. Dengan pull-down, idle = LOW = accelerometer *terpilih* dan ikut menempel di jalur PA5/PA7 → bacaan joystick kanan rusak (bug "CYL4 98%"). Firmware sekarang memaksa **PE3 = HIGH** permanen, dan S1_1 dipindah ke PE4.

### NRF24L01+ — SPI2

| Sinyal NRF24 | Pin STM32 |
|---|---|
| SCK | **PB13** |
| MISO | **PC2** |
| MOSI | **PB15** |
| CSN | **PC6** |
| CE | **PC7** |
| IRQ | **PC8** (didefinisikan, polling dipakai) |
| VCC | **3.3 V** (wajib, + kapasitor 10 µF dekat modul) |
| GND | GND |

SPI clock = APB1/16 ≈ 2.6 MHz (aman, NRF24 max 10 MHz).

### OLED SSD1306 128×64 — I2C3 (400 kHz)

| Sinyal | Pin |
|---|---|
| SCL | **PA8** |
| SDA | **PC9** |

### LED indikator

| LED | Pin | Arti |
|---|---|---|
| Merah `LED_R` | **PD7** | Emergency aktif / NRF24 gagal init / menunggu S0 = 1 (berkedip) |
| Hijau `LED_G` | **PD5** | TX sukses (kedip tiap paket ter-ACK) |
| Biru `LED_B` | **PD3** | TX gagal |

Debug: **USB CDC** (Virtual COM Port) lewat konektor USB OTG FS.

---

## 5. Pin Mapping — Receiver / Bridge

Board: STM32F407G-DISC1. Clock: **168 MHz** (HSE + PLL, fallback HSI). Firmware sangat ramping — hanya NRF24 → CAN.

### NRF24L01+ — SPI1

| Sinyal NRF24 | Pin STM32 |
|---|---|
| SCK | **PA5** |
| MISO | **PA6** |
| MOSI | **PA7** |
| CE | **PE4** |
| CSN | **PE5** |
| VCC / GND | 3.3 V / GND |

**PE3** (CS accelerometer onboard) dipaksa **HIGH** supaya tidak ikut nempel di bus SPI1.

### CAN1

| Sinyal | Pin |
|---|---|
| CAN1_RX | **PD0** |
| CAN1_TX | **PD1** |

Butuh **CAN transceiver** (mis. SN65HVD230 3.3 V, atau MCP2551 dengan level shifter) + terminasi **120 Ω** di kedua ujung bus.

**Timer:** TIM14 = time base 1 ms untuk CANopenNode.

---

## 6. Pin Mapping — Control Board (Robot)

Board: STM32F407G-DISC1. Clock: **168 MHz**. Board ini **CAN-only** (tidak ada radio).

### CAN1

| Sinyal | Pin |
|---|---|
| CAN1_RX | **PD0** |
| CAN1_TX | **PD1** |

### Output digital (relay / driver)

| Sinyal | Pin | Fungsi |
|---|---|---|
| **Emergency relay** | **PB8** | HIGH = sistem armed (S0 = 1). LOW = emergency / link putus |
| **Motor starter** | **PE6** | HIGH saat `motor_active = 1`. LOW saat emergency / sleep |
| **Breaker Valve 1 (Tool 1)** | **PB1** | ON/OFF digital, dipicu `joy_left_btn2` |

### Output PWM (20 kanal) — TIM1, TIM2, TIM3, TIM4, TIM8 @ **500 Hz**

Setiap kanal menggerakkan basis **TIP122** (Darlington) lewat resistor, ke solenoid katup proporsional.

| # | Kanal | Timer/CH | Pin |
|---|---|---|---|
| 1 | Cylinder 1 OUT | TIM8_CH1 | **PC6** |
| 2 | Cylinder 1 IN | TIM8_CH2 | **PC7** |
| 3 | Cylinder 2 OUT | TIM8_CH4 | **PC9** |
| 4 | Cylinder 2 IN | TIM3_CH2 | **PB5** |
| 5 | Cylinder 3 OUT (bucket) | TIM2_CH2 | **PA1** |
| 6 | Cylinder 3 IN (bucket) | TIM3_CH3 | **PB0** |
| 7 | Cylinder 4 OUT | TIM2_CH3 | **PA2** |
| 8 | Cylinder 4 IN | TIM2_CH1 | **PA0** |
| 9 | Tool 1 (breaker valve 1) | *(GPIO, bukan PWM)* | **PB1** |
| 10 | Tool 2 (breaker flow valve) | TIM8_CH3 | **PC8** |
| 11 | Slew CW | TIM4_CH4 | **PD15** |
| 12 | Slew CCW | TIM4_CH3 | **PD14** |
| 13 | Outrigger Left UP | TIM4_CH1 | **PD12** |
| 14 | Outrigger Left DOWN | TIM3_CH1 | **PB4** |
| 15 | Outrigger Right UP | TIM2_CH4 | **PA3** |
| 16 | Outrigger Right DOWN | TIM1_CH1 | **PE9** |
| 17 | Track Right FORWARD | TIM1_CH2 | **PE11** |
| 18 | Track Right BACKWARD | TIM1_CH3 | **PE13** |
| 19 | Track Left FORWARD | TIM4_CH2 | **PD13** |
| 20 | Track Left BACKWARD | TIM1_CH4 | **PE14** |

**Timer:** TIM14 = time base 1 ms untuk CANopenNode.
Debug: **USB CDC** lewat USB OTG FS.

---

## 7. Daftar 20 Kanal PWM & Limitnya

Setiap kanal punya **duty minimum & maksimum** sendiri (di `control.c`, array `pwm_limits[]`). Defleksi joystick 0–100% dipetakan linier ke rentang `min`–`max`, bukan ke 0–100%.

- **`min`** = duty terkecil yang sudah cukup membuka katup (di bawah itu solenoid cuma berdengung, tidak bergerak).
- **`max`** = dibatasi (≤ 70%) supaya **TIP122 tidak saturasi dalam** — pada duty 100% Darlington menumpuk muatan basis dan bisa **nyangkut ON** meski PWM sudah 0%.

| Kanal | min % | max % |
|---|---|---|
| 1 — Cylinder 1 OUT | 30 | 70 |
| 2 — Cylinder 1 IN | 30 | 55 |
| 3 — Cylinder 2 OUT | 30 | 55 |
| 4 — Cylinder 2 IN | 20 | 50 |
| 5 — Cylinder 3 OUT | 32 | 70 |
| 6 — Cylinder 3 IN | 32 | 60 |
| 7 — Cylinder 4 OUT | 32 | 50 |
| 8 — Cylinder 4 IN | 32 | 60 |
| 9 — Tool 1 | 0 | 60 |
| 10 — Tool 2 (breaker flow) | 0 | 50 |
| 11 — Slew CW | 25 | 46 |
| 12 — Slew CCW | 25 | 49 |
| 13 — Outrigger Left UP | 10 | 70 |
| 14 — Outrigger Left DOWN | 10 | 70 |
| 15 — Outrigger Right UP | 10 | 70 |
| 16 — Outrigger Right DOWN | 10 | 70 |
| 17 — Track Right FWD | 11 | 46 |
| 18 — Track Right BWD | 11 | 46 |
| 19 — Track Left FWD | 18 | 53 |
| 20 — Track Left BWD | 21 | 66 |

Deadzone joystick: **±5** di sekitar 127. PWM ramping & smooth-curve saat ini **OFF** (respon instan).

---

## 8. Cara Penggunaan Remote (Panduan Operator)

### Urutan menyalakan (wajib berurutan)

```
  [1] S0 = 1 ─────► [2] Semua netral ─────► [3] Hold S1_1 ─────► [4] Hold S2_1 ─────► [5] Pilih mode S5 ─────► SIAP KERJA
      lepas EMG        joystick center         keluar SLEEP         motor START           UPPER/DUAL/LOWER
```

**Langkah 1 — Nyalakan remote, pastikan S0 = 1**
Saat boot, firmware **menahan diri di loop tunggu** selama S0 masih di posisi emergency (0). LED merah berkedip 2 Hz. Putar/lepas tombol emergency ke posisi normal → sistem lanjut, OLED menampilkan splash screen.

**Langkah 2 — Sistem masuk SLEEP MODE (otomatis, demi keselamatan)**
Saat boot, remote **selalu** mulai di SLEEP MODE: semua joystick dipaksa 127 dan semua switch dipaksa 0 dalam paket yang dikirim, apapun posisi fisiknya. Robot tidak akan bergerak.

Untuk bisa keluar dari SLEEP, **safety interlock** harus lulus:
- keempat sumbu joystick berada di tengah (**127 ± 25**), dan
- semua tombol/switch (kecuali S0, S1_1, S1_2) dalam posisi **0**.

OLED menampilkan `** SLEEP MODE **` dan status apakah safety sudah `OK` atau belum.

**Langkah 3 — (Opsional) Kalibrasi joystick: hold S1_2**
Kalau joystick tidak benar-benar netral di 127 (drift), letakkan kedua joystick bebas di posisi diam, lalu **tahan S1_2 ± 0.1 detik** saat masih di SLEEP MODE. Posisi saat itu direkam sebagai titik tengah baru; rentang penuh 0–255 tetap terjaga. Kalibrasi hanya bisa **sekali per siklus** SLEEP (reset kalau emergency ditekan). OLED menampilkan progress bar kalibrasi.

**Langkah 4 — Keluar SLEEP: hold S1_1**
Dengan safety interlock hijau, **tahan S1_1 ± 0.1 detik**. OLED memperlihatkan progress bar; setelah penuh, SLEEP dilepas dan layar berubah jadi `MOTOR READY`.

**Langkah 5 — Start motor: hold S2_1**
**Tahan S2_1 ± 0.1 detik** → `motor_active = 1`, control board menaikkan **PE6 (motor starter)** ke HIGH. Sifatnya **self-holding**: sekali aktif, tetap aktif walau S2_1 dilepas. Selama `motor_active = 0`, **semua PWM dipaksa 0** — tidak ada aktuator yang bergerak.

**Langkah 6 — Pilih mode kerja dengan S5_1 / S5_2**

| S5_1 | S5_2 | Mode | Untuk apa |
|:---:|:---:|---|---|
| 0 | 0 | **UPPER** | Excavator: silinder boom/arm/bucket, slew, breaker |
| 1 | 0 | **DUAL** | Kedua track jalan bersamaan (perlu tahan tombol joystick kanan 1) |
| 0 | 1 | **LOWER** | Mobilitas: track kiri/kanan, outrigger kiri/kanan |
| 1 | 1 | *INVALID* | OLED menampilkan `MODE: INVALID` — perbaiki posisi switch |

Mode boleh diganti kapan saja saat operasi; kanal dari mode lain otomatis di-nol-kan.

### Menghentikan robot

| Tindakan | Efek |
|---|---|
| **Tekan S0 → 0 (EMERGENCY)** | Interrupt langsung: relay emergency (PB8) LOW, **semua PWM = 0**, motor starter (PE6) LOW, breaker OFF. Sistem kembali ke SLEEP; motor & kalibrasi ter-reset. OLED: `EMERGENCY STOP`, LED merah menyala. |
| **Kembalikan S0 → 1** | Kembali ke splash screen + **SLEEP MODE** — harus ulang dari Langkah 2 (interlock, S1_1, S2_1). Tidak ada "resume" otomatis. |
| **Radio putus / remote mati** | Bridge mengirim paket nol setelah **500 ms** tanpa sinyal; control board juga mendeteksi RPDO basi setelah **200 ms**. Keduanya menghasilkan `s0 = 0` → emergency. Robot berhenti sendiri. |

---

## 9. Peta Kontrol per Mode

### Mode UPPER (S5_1 = 0, S5_2 = 0) — Excavator

| Input | Arah | Aksi |
|---|---|---|
| **Joystick kiri — Y** | atas | Cylinder 3 (Bucket) **OUT** |
| | bawah | Cylinder 3 (Bucket) **IN** |
| **Joystick kiri — X** | kanan | **Slew CW** (putar kanan) |
| | kiri | **Slew CCW** (putar kiri) |
| **Joystick kanan — Y** *(normal)* | atas | Cylinder 2 **IN** |
| | bawah | Cylinder 2 **OUT** |
| **Joystick kanan — Y** *(sambil tahan **tombol kanan 2**)* | atas | Cylinder 1 **OUT** |
| | bawah | Cylinder 1 **IN** |
| **Joystick kanan — X** | kanan | Cylinder 4 **UP** |
| | kiri | Cylinder 4 **DOWN** |
| **Tombol joystick kiri 2** | tekan | **Breaker ON** (valve 1, PB1 HIGH) |
| | lepas | Breaker OFF |

Semua kanal mobilitas (track, outrigger) dipaksa 0% di mode ini.

### Mode LOWER (S5_1 = 0, S5_2 = 1) — Mobilitas

| Input | Arah | Aksi |
|---|---|---|
| **Joystick kiri — Y** | atas | Track **kiri** maju |
| | bawah | Track **kiri** mundur |
| **Joystick kanan — Y** | atas | Track **kanan** maju |
| | bawah | Track **kanan** mundur |
| **Joystick kiri — X** | kanan | Outrigger **kanan** naik |
| | kiri | Outrigger **kanan** turun |
| **Joystick kanan — X** | kanan | Outrigger **kiri** turun |
| | kiri | Outrigger **kiri** naik |

> Sumbu X sengaja "bersilang" (stik kiri → outrigger kanan, stik kanan → outrigger kiri) mengikuti pengkabelan aktual di robot. Bukan bug.

Semua kanal excavator (silinder, slew, breaker) dipaksa 0% di mode ini.

### Mode DUAL (S5_1 = 1, S5_2 = 0) — Kedua Track Bersamaan

| Input | Aksi |
|---|---|
| **Tahan tombol joystick kanan 1** + **joystick kanan Y ke atas** | **Kedua track maju** bersamaan |
| **Tahan tombol joystick kanan 1** + **joystick kanan Y ke bawah** | **Kedua track mundur** bersamaan |
| Tombol kanan 1 dilepas | Kedua track berhenti |

Dipakai untuk jalan lurus. Semua kanal lain (silinder, slew, outrigger, breaker) dipaksa 0%.

---

## 10. Tampilan OLED

| Layar | Kapan muncul | Isi |
|---|---|---|
| **Splash** | Setelah boot / setelah keluar dari emergency | Judul sistem |
| **`** SLEEP MODE **`** | Selama SLEEP | Status safety interlock (OK / belum), progress bar hold **S1_1**, progress bar kalibrasi **S1_2** |
| **`MOTOR READY`** | Setelah keluar SLEEP, motor belum jalan | Progress bar hold **S2_1** |
| **`MODE: UPPER` / `DUAL`** | Motor aktif, mode excavator | `CYL2/CYL3/CYL4 UP=..% DOWN=..%`, `SLEW CCW=..% CW=..%` |
| **`MODE: LOWER`** | Motor aktif, mode mobilitas | `TRK L/R F=..% B=..%`, `OUT L/R UP=..% DOWN=..%` |
| **`MODE: INVALID`** | S5_1 = 1 **dan** S5_2 = 1 | Peringatan perbaiki posisi switch |
| **`EMERGENCY STOP`** | S0 = 0 | Peringatan besar |

Selain itu ditampilkan **persen baterai remote** dan **link quality** (0–100%, dari rasio paket ter-ACK NRF24 dalam jendela 16 paket terakhir). Nilai link quality sengaja **di-glide** maksimal 10 poin/detik supaya angkanya bisa dibaca mata, sementara pengukuran aslinya tetap jalan penuh.

---

## 11. Sistem Keselamatan (Failsafe)

| Lapisan | Mekanisme |
|---|---|
| **Boot gate** | Firmware transmitter tidak akan masuk main loop selama **S0 = 0**. LED merah berkedip. |
| **SLEEP mode saat start** | Selalu mulai di SLEEP. Robot tidak bisa bergerak sebelum operator sengaja keluar dari SLEEP. |
| **Safety interlock** | Keluar SLEEP hanya boleh kalau joystick center (127 ± 25) **dan** semua switch nol. Mencegah robot menyentak karena stik ketinggalan di posisi ekstrem. |
| **Motor gate** | Selama `motor_active = 0`, control board memaksa **semua 20 PWM = 0**, motor starter LOW, breaker OFF. |
| **Emergency (S0) via interrupt** | EXTI0 di PB0 — tidak menunggu polling loop. Di control board, `s0 = 0` adalah cabang pertama `Control_Update()` dan langsung `return`. |
| **Radio timeout (bridge)** | Tidak ada paket > **500 ms** → bridge mem-publish 8 byte nol (= S0 emergency). |
| **CAN timeout (control)** | RPDO1 terakhir > **200 ms** → paket dianggap mati, dipakai nol → emergency. |
| **CAN self-heal (bridge)** | Heartbeat Node-2 hilang > 2 detik → bridge menjalankan `canopen_app_resetCommunication()` untuk memulihkan bus. |
| **Clock fallback** | Ketiga board mencoba HSE 5x; kalau gagal, jatuh ke HSI dengan PLL yang disesuaikan agar SYSCLK & bit-timing CAN tetap sama (dulu board hang di `Error_Handler` saat cold boot). |
| **NRF24 retry saat boot** | Receiver mengulang `NRF24_Configure()` sampai 10x — dulu modul gagal init kalau supply-nya belum stabil, dan butuh tekan reset manual. |
| **PWM max ≤ 70%** | Mencegah TIP122 saturasi dalam dan nyangkut ON. |

> **PERINGATAN:** ini mesin hidrolik berat. Selalu tes dengan **outrigger terangkat / tanpa beban / pompa mati** dulu setelah mengubah firmware, dan pastikan tombol emergency fisik (E-stop daya) terpasang **terpisah** dari S0.

---

## 12. Build & Flash

### Lewat STM32CubeIDE (cara termudah)

1. `File → Open Projects from File System…` → arahkan ke folder repo.
2. Import ketiga project (`transmitter_…`, `receiver_…`, `control_…`).
3. Build (Ctrl+B), lalu Run/Debug dengan ST-LINK ke masing-masing board.

`.cproject` / `.project` / `.settings` sengaja **di-commit**, jadi project bisa langsung di-build setelah clone.

### Lewat command line (`make`)

Harus memakai **GCC bawaan STM32CubeIDE (12.3.x)** — makefile hasil generate memakai flag `-fcyclomatic-complexity` yang tidak dikenal oleh GCC ARM standalone (mis. 10.2.1, akan error).

```bash
export PATH="/opt/st/stm32cubeide_1.15.1/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.linux64_1.0.100.202403111256/tools/bin:$PATH"

make -C transmitter_demolition_robot/Debug all
make -C receiver_demolition_robot/Debug all
make -C control_demolition_robot/Debug all
```

Warning yang wajar muncul: variabel/fungsi tidak terpakai, dan warning linker "RWX LOAD segment".

Flash dengan ST-LINK:

```bash
st-flash --reset write transmitter_demolition_robot/Debug/transmitter_demolition_robot.bin 0x8000000
```

### Regenerasi dari CubeMX

Kalau `.ioc` di-regenerate: pin switch di transmitter **harus** dikonfigurasi ulang dengan internal pull-down. Itu sudah ditangani oleh `MX_GPIO_ConfigureSwitchPullDown()` yang dipanggil di `main()` setelah `MX_GPIO_Init()` — jangan hapus.

---

## 13. Debugging via USB CDC

Transmitter dan control board mengekspos **Virtual COM Port** lewat konektor **USB OTG FS** (bukan port ST-LINK). Colok kabel USB kedua ke port itu, lalu:

```bash
screen /dev/ttyACM0 115200      # atau: minicom -D /dev/ttyACM0
```

**Transmitter** mencetak tiap ~1 detik:

```
=== Transmitter Demolition Robot (NRF24 + Discovery) ===
System Clock: 84 MHz (HSE), USB CDC Ready
NRF24: OK
  NRF: ST=0x0E | OK=1234 FAIL=5 NRDY=0 | LQ=97%
```

- `ST` = register STATUS NRF24, `OK`/`FAIL` = hitungan TX, `LQ` = link quality.

**Control board** mencetak tiap 1 detik:

```
LINK=UP  | RAW:[7F 7F 7F 7F 00 00 10 20]
  LX=127 LY=127 RX=127 RY=127 | S0=1 S5=00 | M=1
```

- `LINK=DOWN` berarti tidak ada RPDO1 selama > 200 ms (bridge mati / kabel CAN putus / bitrate beda).

Detail lebih lanjut: `transmitter_demolition_robot/USB_CDC_DEBUG_README.md`.

---

## 14. Troubleshooting

| Gejala | Kemungkinan penyebab & solusi |
|---|---|
| **`NRF24: FAIL` saat boot transmitter** | Cek wiring SPI2 (PB13/PC2/PB15) + CSN PC6 / CE PC7. Modul NRF24 **wajib 3.3 V**, dan pasang kapasitor **10 µF** di kaki VCC-GND modul — brownout saat TX adalah penyebab paling umum. |
| **Receiver baru jalan setelah tombol reset ditekan** | Sudah diperbaiki: receiver mengulang `NRF24_Configure()` hingga 10x saat boot. Kalau masih terjadi, supply modul terlalu lambat naik — perbesar kapasitor. |
| **LED merah transmitter berkedip terus, tidak masuk main loop** | S0 masih di posisi emergency (0). Kembalikan ke normal. |
| **Robot tidak bergerak, OLED `SLEEP MODE`, safety belum OK** | Ada joystick tidak di tengah atau ada switch masih ON. Netralkan semuanya. Kalau joystick drift, lakukan kalibrasi (hold S1_2). |
| **Robot tidak bergerak walau sudah keluar SLEEP** | Motor belum di-start — hold **S2_1**. Selama `motor_active = 0` semua PWM dipaksa 0. |
| **`LINK=DOWN` di control board** | Cek transceiver CAN, terminasi 120 Ω di kedua ujung, PD0/PD1 tidak tertukar, dan bitrate 500 kbps sama di kedua board. |
| **Silinder bergerak tapi lemah / cuma berdengung** | Duty di bawah ambang buka katup — naikkan `min` kanal terkait di `pwm_limits[]` (`control.c`). |
| **Solenoid nyangkut ON setelah joystick dilepas** | TIP122 saturasi dalam — **turunkan `max`** kanal itu (jangan pernah 100%). |
| **Joystick kanan melompat ke ~98% sendiri** | Regresi lama akibat PE3 (CS accelerometer) dipakai sebagai switch. Pastikan S1_1 ada di **PE4** dan PE3 dipaksa HIGH. |
| **Bacaan ADC bergejolak / kanal lain ikut naik** | Jangan tambahkan pin mengambang ke sequence ADC. PA2 sengaja dikeluarkan dari scan. |
| **OLED blank / macet** | I2C3 = PA8 (SCL) / PC9 (SDA), alamat SSD1306 default `0x3C`. Layar emergency di-redraw periodik supaya glitch I2C pulih sendiri. |
| **USB CDC tidak muncul** | Kalau HSE gagal dan board jatuh ke HSI, toleransi clock HSI (~1%) tidak memenuhi spek USB (±0.25%) — USB bisa tidak stabil, tapi SPI/NRF24/OLED tetap normal. Debug print saat boot akan menyebutkan `HSI fallback`. |

---

## Lisensi

Kode HAL & middleware STM32 mengikuti lisensi STMicroelectronics masing-masing. CANopenNode mengikuti Apache-2.0.
