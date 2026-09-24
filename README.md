# Demolition Robot — Firmware Remote dan Control Board

Firmware STM32 untuk remote, bridge radio/CAN, dan control board robot demolisi hidrolik. Repository ini memiliki **tujuh proyek STM32CubeIDE**: jalur bridge CANopen, control yang menerima NRF24 langsung atau CAN kabel, serta varian mesin **110** dan **250**. Pilih binary menurut board dan pengkabelan mesin.

```text
Remote STM32F407 ── NRF24 ───────────────────────────> Control disc_nrf24 (110) / control_250
       │                                                   ↑
       ├── CAN kabel, frame 0x181 ──────────────────────────┘
       │
       └── NRF24 ──> Bridge F407 / F103 ── CANopen 0x181 ──> Control CAN-only
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
| `transmitter_demolition_robot/` | STM32F407 Discovery | Remote utama: joystick, switch, OLED, NRF24, dan CAN kabel yang dipilih otomatis saat heartbeat control diterima |
| `transmitter_demolition_robot_disc_nrf24/` | STM32F407 Discovery | Varian remote NRF24 pada Discovery |
| `receiver_demolition_robot/` | STM32F407 Discovery | Bridge NRF24 → CANopen, Node-ID 1 |
| `receiver_demolition_robot_stm32f103c8t6/` | STM32F103C8 | Alternatif bridge NRF24 → CANopen, Node-ID 1 |
| `control_demolition_robot/` | STM32F407 Discovery | Control CANopen-only, Node-ID 2 |
| `control_demolition_robot_disc_nrf24/` | STM32F407 Discovery | Control mesin 110; menerima NRF24 langsung atau CAN kabel |
| `control_demolition_robot_250/` | STM32F407 Discovery | Control mesin 250; radio/CAN dengan logika brake dan katup Cylinder 1 khusus |

Masing-masing adalah proyek STM32CubeIDE (`.ioc`, `.project`, `.cproject`). `Debug/` dan `Release/` berisi hasil build dan diabaikan Git. Jangan menukar binary mesin 110 dengan 250.

---

## 2. Arsitektur Sistem

**Jalur radio langsung:** transmitter mengirim payload 8 byte via NRF24 ke `control_demolition_robot_disc_nrf24` atau `control_demolition_robot_250`. Kedua control membaca CAN lebih dulu bila frame tersedia, kemudian NRF24. NRF24 control dipasang pada SPI2, bukan pin radio pada peta CAN-only lama.

**Jalur CAN kabel:** `transmitter_demolition_robot` menerima heartbeat standar CAN `0x702` dari control setiap 100 ms. Bila heartbeat masih segar (batas 300 ms), remote memilih CAN 500 kbps, mengirim payload pada ID `0x181` setiap 20 ms, lalu mematikan radio. Saat heartbeat hilang, remote mengaktifkan lagi NRF24. Transmitter memakai frame CAN biasa; ID dan payloadnya kompatibel dengan jalur CANopen.

**Jalur bridge CANopen:** receiver F407 atau F103 menerima NRF24 dan menulis 8 byte ke Object Dictionary `0x2000`. Node-ID 1 menerbitkan TPDO1 (COB-ID `0x181`) setiap 50 ms. `control_demolition_robot` sebagai Node-ID 2 mengonsumsi RPDO1 dan menggerakkan output. Jalur ini tetap tersedia untuk pemasangan dengan radio terpisah dari control board.

| Parameter | Nilai |
|---|---|
| Radio | NRF24L01+, channel 76 = 2476 MHz, 1 Mbps, alamat `E7 E7 E7 E7 E7`, Auto-ACK |
| Payload | 8 byte tetap; layout sama pada radio dan CAN |
| CAN | 500 kbps; butuh transceiver dan terminasi 120 Ω di kedua ujung bus |
| CANopen bridge | Node-ID 1, TPDO1 `0x181`, OD `0x2000` |
| CANopen control | Node-ID 2, RPDO1 `0x181`; timeout paket 200 ms |
| Control radio/CAN langsung | ID kendali `0x181`, heartbeat `0x702`, timeout paket 500 ms |

Definisi payload ada di `ctrl_link.h` dan decoder `nrf24.c` pada proyek terkait. Saat mengubah protokol, sinkronkan semua pasangan firmware.

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
| 2 | `joy_right_btn1` | tombol joystick kanan 1 → **enable dual-track** (DUAL); pada mesin 250 juga membuka katup Cylinder 1 (UPPER) |
| 3 | `joy_right_btn2` | tombol joystick kanan 2 → **shift ke Cylinder 1** (UPPER pada CAN-only/mesin 110); tidak dipakai untuk fungsi itu pada 250 |
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
| 14 | `unlocked` | status keluar SLEEP pada transmitter utama; decoder control langsung menerimanya |
| 15 | reserved | — |

> **Catatan:** byte 4 dulunya potensiometer R8 (flow breaker). Sekarang byte itu dipakai untuk persentase baterai remote. Control CAN-only memaksa `r8 = 0`; control langsung mendecode byte tersebut tetapi tidak memakainya untuk flow breaker.

---

## 4. Pin Mapping — Transmitter (Remote)

Bagian ini merujuk pada `transmitter_demolition_robot/`; periksa `.ioc` varian lain sebelum menyamakan wiring.

Board: STM32F407G-DISC1. Clock: **84 MHz** (HSE 8 MHz + PLL, dengan fallback HSI kalau HSE gagal start).

### Input analog — ADC1 + DMA (5 kanal, scan mode, sampling 480 cycle)

| Sinyal | Pin | Kanal ADC | Urutan DMA |
|---|---|---|---|
| Joystick kiri X | **PC1** | IN11 | `adc_buffer[0]` |
| Joystick kiri Y | **PC3** | IN13 | `adc_buffer[1]` |
| Joystick kanan Y | **PA5** | IN5 | `adc_buffer[2]` |
| Joystick kanan X | **PA7** | IN7 | `adc_buffer[3]` |
| Baterai (voltage divider) | **PA0** | IN0 | `adc_buffer[4]` |

Pengolahan: 12-bit → 8-bit, **median-of-3 filter** (buang spike), lalu **kalibrasi offset** per sumbu.
PA2 sengaja **dikeluarkan dari scan pada `adc.c`** — pin mengambang di situ menimbulkan cross-talk yang membuat kanal lain terbaca 100%. Berkas `.ioc` masih mencantumkan PA2/6 konversi; setelah regenerasi CubeMX, pertahankan konfigurasi 5 kanal pada source dan periksa urutan DMA lagi.

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

Peta berikut untuk `receiver_demolition_robot/` (F407). Varian STM32F103C8 memakai pin berbeda; gunakan `.ioc` proyek tersebut.

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

### Control CAN-only: `control_demolition_robot/`

Board STM32F407G-DISC1, clock 168 MHz. CAN1 RX `PD0`, TX `PD1`; diperlukan transceiver CAN. Relay emergency `PB8`, motor starter `PE6`, Tool 1 digital `PB1`. TIM14 menjadi time base CANopen 1 ms. Output proporsional 500 Hz:

| # | Kanal | Timer/CH | Pin |
|---|---|---|---|
| 1 | Cylinder 1 OUT | TIM8_CH1 | PC6 |
| 2 | Cylinder 1 IN | TIM8_CH2 | PC7 |
| 3 | Cylinder 2 OUT | TIM8_CH4 | PC9 |
| 4 | Cylinder 2 IN | TIM3_CH2 | PB5 |
| 5 | Cylinder 3 OUT | TIM2_CH2 | PA1 |
| 6 | Cylinder 3 IN | TIM3_CH3 | PB0 |
| 7 | Cylinder 4 OUT | TIM2_CH3 | PA2 |
| 8 | Cylinder 4 IN | TIM2_CH1 | PA0 |
| 9 | Tool 1 | GPIO digital | PB1 |
| 10 | Tool 2 | TIM8_CH3 | PC8 |
| 11 | Slew CW | TIM4_CH4 | PD15 |
| 12 | Slew CCW | TIM4_CH3 | PD14 |
| 13 | Outrigger kiri UP | TIM4_CH1 | PD12 |
| 14 | Outrigger kiri DOWN | TIM3_CH1 | PB4 |
| 15 | Outrigger kanan UP | TIM2_CH4 | PA3 |
| 16 | Outrigger kanan DOWN | TIM1_CH1 | PE9 |
| 17 | Track kanan FORWARD | TIM1_CH2 | PE11 |
| 18 | Track kanan BACKWARD | TIM1_CH3 | PE13 |
| 19 | Track kiri FORWARD | TIM4_CH2 | PD13 |
| 20 | Track kiri BACKWARD | TIM1_CH4 | PE14 |

### Control NRF24 langsung: mesin 110 dan 250

Kedua varian memakai CAN1 `PD0`/`PD1`, relay emergency `PB8`, motor starter `PE6`, serta NRF24 di SPI2 (`PB13` SCK, `PB14` MISO, `PB15` MOSI, `PE4` CE, `PE5` CSN). Frekuensi PWM 500 Hz. Peta logis ke pin sudah berubah; tabel CAN-only di atas **tidak berlaku** untuk kedua varian ini.

| # | Mesin 110 (`disc_nrf24`) | Mesin 250 | Pin kedua varian |
|---:|---|---|---|
| 1 | Cylinder 1 OUT | Brake | PA3 |
| 2 | Cylinder 1 IN | Katup Cylinder 1 ON | PA1 |
| 3–4 | Cylinder 2 OUT; IN | Sama | PE9; PB1 |
| 5–6 | Cylinder 3 OUT; IN | Sama | PE13; PE11 |
| 7–8 | Cylinder 4 OUT; IN | Sama | PD15; PD13 |
| 9–10 | Tool 1; Tool 2 (GPIO digital) | Sama | PD12; PD14 |
| 11–12 | Slew CW; CCW | Sama | PB0; PE14 |
| 13–14 | Outrigger kiri UP; DOWN | Sama | PA0; PA2 |
| 15–16 | Outrigger kanan UP; DOWN | Sama | PC8; PB4 |
| 17–18 | Track kanan FORWARD; BACKWARD | Sama | PC7; PC6 |
| 19–20 | Track kiri FORWARD; BACKWARD | Sama | PB5; PC9 |

Sumber kebenaran akhir adalah switch `PWM_SetDutyCycle()` pada `Core/Src/pwm.c` masing-masing proyek. Periksa ulang kanal fisik setelah perubahan wiring atau `.ioc`.

---

## 7. Daftar 20 Kanal PWM & Limitnya

`pwm_limits[]` di `Core/Src/control.c` memetakan defleksi joystick ke duty minimum/maksimum. Indeks 9 (dan 10 pada varian radio langsung) adalah GPIO digital, bukan PWM proporsional. Deadzone joystick control adalah sekitar titik tengah 127; lihat `JOYSTICK_DEADZONE` di source untuk nilainya.

**Control CAN-only** memiliki kalibrasi berikut di source saat ini. Ini berlaku untuk peta pin CAN-only pada §6, bukan untuk varian 110/250 radio langsung.

| Kanal | min % | max % | Kanal | min % | max % |
|---|---:|---:|---|---:|---:|
| 1 Cylinder 1 OUT | 30 | 70 | 2 Cylinder 1 IN | 30 | 55 |
| 3 Cylinder 2 OUT | 30 | 55 | 4 Cylinder 2 IN | 20 | 50 |
| 5 Cylinder 3 OUT | 32 | 70 | 6 Cylinder 3 IN | 32 | 60 |
| 7 Cylinder 4 OUT | 32 | 50 | 8 Cylinder 4 IN | 32 | 60 |
| 9 Tool 1 (GPIO) | 0 | 60 | 10 Tool 2 | 0 | 50 |
| 11 Slew CW | 25 | 100 | 12 Slew CCW | 25 | 100 |
| 13 Outrigger kiri UP | 10 | 70 | 14 Outrigger kiri DOWN | 10 | 70 |
| 15 Outrigger kanan UP | 10 | 70 | 16 Outrigger kanan DOWN | 10 | 70 |
| 17 Track kanan FWD | 11 | 46 | 18 Track kanan BWD | 11 | 46 |
| 19 Track kiri FWD | 18 | 53 | 20 Track kiri BWD | 21 | 66 |

**Control `disc_nrf24` dan `250`:** tabel `pwm_limits[]` saat ini berisi `{0, 100}` untuk semua indeks. Nilai lama **jangan disalin** ke varian ini: pasangan arah dan pemetaan kanal ke pin berubah, sehingga nomor kanal yang sama belum tentu menggerakkan solenoid fisik yang sama. Pada 250, kanal 1 (brake) dan 2 (katup Cylinder 1) memang bekerja sebagai output 0%/100%. Kalibrasi duty varian 110/250 memerlukan verifikasi tiap kanal di bench, tanpa beban dan dengan pompa mati; status build varian 250 sudah dicatat, tetapi belum diuji di hardware.

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
Saat boot, remote **selalu** mulai di SLEEP MODE. Ketika interlock belum aman, paket yang dikirim memaksa joystick ke 127 dan switch kendali ke 0. Setelah posisi fisik aman, remote dapat menerima perintah `S1_1` untuk keluar dari SLEEP.

Untuk bisa keluar dari SLEEP, **safety interlock** harus lulus:
- keempat sumbu joystick berada di tengah (**127 ± 25**), dan
- semua tombol/switch (kecuali S0, S1_1, S1_2) dalam posisi **0**.

OLED menampilkan `** SLEEP MODE **` dan status apakah safety sudah `OK` atau belum.

**Langkah 3 — (Opsional) Kalibrasi joystick: hold S1_2**
Kalau joystick tidak benar-benar netral di 127 (drift), letakkan kedua joystick bebas di posisi diam, lalu **tahan S1_2 ± 0.1 detik** saat masih di SLEEP MODE. Posisi saat itu direkam sebagai titik tengah baru; rentang penuh 0–255 tetap terjaga. Kalibrasi hanya bisa **sekali per siklus** SLEEP (reset kalau emergency ditekan). OLED menampilkan progress bar kalibrasi.

**Langkah 4 — Keluar SLEEP: hold S1_1**
Dengan safety interlock hijau, **tahan S1_1 ± 0.1 detik**. OLED memperlihatkan progress bar; setelah penuh, SLEEP dilepas dan layar berubah jadi `MOTOR READY`.

**Langkah 5 — Start motor: hold S2_1**
**Tahan S2_1 ± 0.1 detik** → `motor_active = 1`, control board menaikkan **PE6 (motor starter)** ke HIGH. Sifatnya **self-holding**: sekali aktif, tetap aktif walau S2_1 dilepas. Pada **control CAN-only**, `motor_active = 0` menahan semua PWM di 0. Pada **control `disc_nrf24`/`250`**, motor starter tetap OFF tetapi unlock `S1_1` dapat mengizinkan PWM untuk pengujian solenoid dengan pompa mati. Jangan mengandalkan status motor sebagai satu-satunya interlock.

**Langkah 6 — Pilih mode kerja dengan S5_1 / S5_2**

| S5_1 | S5_2 | Mode | Untuk apa |
|:---:|:---:|---|---|
| 0 | 0 | **UPPER** | Excavator: silinder boom/arm/bucket, slew, breaker |
| 1 | 0 | **DUAL** | Kedua track jalan bersamaan (perlu tahan tombol joystick kanan 1) |
| 0 | 1 | **LOWER** | Mobilitas: track kiri/kanan, outrigger kiri/kanan |
| 1 | 1 | *INVALID* | OLED menampilkan `MODE: INVALID` — perbaiki posisi switch |

Pada control `disc_nrf24`/`250`, setelah mode diubah, seluruh output gerak ditahan sampai keempat sumbu joystick kembali netral (127 ±10). Posisi `S5_1 = 1` dan `S5_2 = 1` menolkan output gerak.

### Menghentikan robot

| Tindakan | Efek |
|---|---|
| **Tekan S0 → 0 (EMERGENCY)** | Interrupt langsung: relay emergency (PB8) LOW, **semua PWM = 0**, motor starter (PE6) LOW, breaker OFF. Sistem kembali ke SLEEP; motor & kalibrasi ter-reset. OLED: `EMERGENCY STOP`, LED merah menyala. |
| **Kembalikan S0 → 1** | Kembali ke splash screen + **SLEEP MODE** — harus ulang dari Langkah 2 (interlock, S1_1, S2_1). Tidak ada "resume" otomatis. |
| **Link putus / remote mati** | Jalur bridge CANopen menolkan paket setelah 500 ms tanpa radio; control CAN-only menolak RPDO yang lebih tua dari 200 ms. Control `disc_nrf24`/`250` mendeteksi tidak ada paket radio/CAN selama 500 ms, menurunkan relay dan motor, lalu meramp output ke nol. Saat link pulih, motor harus di-start ulang. |

---

## 9. Peta Kontrol per Mode

Peta berikut berlaku untuk **control CAN-only** dan logika dasar **mesin 110**. Pin output keduanya berbeda (§6). Khusus mesin 250, lihat perubahan setelah tabel DUAL.

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
| **Tombol joystick kiri 2** | tekan | **Breaker ON** (Tool 1: PB1 pada CAN-only, PD12 pada 110/250) |
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

### Perbedaan mesin 250

Pada **UPPER**, kanal 1 (`PA3`) adalah brake dan selalu 100%; kanal 2 (`PA1`) membuka katup Cylinder 1 saat tombol joystick **kanan 1** ditekan. Katup ini ON/OFF, terpasang paralel antara Cylinder 1 dan Cylinder 2. Joystick kanan Y selalu mengendalikan Cylinder 2; kombinasi tombol kanan 2 + sumbu Y yang dipakai mesin 110 **tidak berlaku**. Pada **LOWER** dan **DUAL**, brake dan katup Cylinder 1 ditutup ke 0%. Pada DUAL, tombol kanan 1 kembali berfungsi sebagai izin kedua track dan joystick kanan Y memilih maju/mundur.

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

Selain itu ditampilkan **persen baterai remote** dan **link quality** (0–100%, dari rasio paket ter-ACK NRF24 dalam jendela 16 paket terakhir). Nilai link quality sengaja **di-glide** maksimal 10 poin/detik supaya angkanya bisa dibaca mata. Saat remote memilih CAN kabel dan radio dimatikan, angka kualitas NRF24 tidak mengukur kualitas CAN; status kabel ditentukan oleh heartbeat `0x702`.

---

## 11. Sistem Keselamatan (Failsafe)

| Lapisan | Mekanisme |
|---|---|
| Boot dan SLEEP remote | `S0 = 0` menahan boot remote. Saat start, remote masuk SLEEP; joystick harus netral dan switch lain OFF sebelum `S1_1` bisa melepasnya. |
| Kalibrasi dan hold | `S1_2` hanya mengkalibrasi saat SLEEP. Hold `S1_1`, `S1_2`, dan `S2_1` dihitung berdasarkan waktu: 20 langkah × 5 ms ≈ 100 ms. |
| Emergency `S0` | `S0 = 0` diproses sebagai prioritas tertinggi; relay `PB8`, starter `PE6`, PWM, dan tool dimatikan. Setelah emergency dilepas, urutan SLEEP/unlock/start diulang. |
| Pergantian mode control langsung | Pada `disc_nrf24`/`250`, perubahan `S5` menahan output sampai empat sumbu kembali netral (127 ±10). Kombinasi mode `11` menolkan seluruh PWM/tool. |
| Link bridge CANopen | Bridge mengirim paket aman bila radio tidak mengirim >500 ms. Control CAN-only menganggap RPDO basi setelah 200 ms; bridge juga melakukan reset komunikasi jika heartbeat Node 2 hilang >2 detik. |
| Link radio/CAN langsung | Control `disc_nrf24`/`250` memantau usia paket bersama; setelah >500 ms relay dan starter turun, output diramp ke nol, dan diperlukan re-arm motor setelah link kembali. Remote juga menghapus `motor_active` setelah link hilang berkepanjangan. |
| Fault dan watchdog control langsung | Jalur `Error_Handler` mematikan output sebelum reset; independent watchdog sekitar 800 ms memulihkan MCU bila loop macet. |
| NRF24 saat boot | Bridge mengulang konfigurasi radio hingga 10 kali saat power-on. Control langsung juga memiliki pemeriksaan status/FIFO dan pemulihan radio. |

**E-stop fisik yang memutus daya aktuator harus terpisah dari sinyal `S0` di firmware.** Sesudah perubahan firmware atau wiring, uji output dengan pompa mati dan tanpa beban. Batas PWM varian 110/250 belum dikalibrasi (§7).

---

## 12. Build & Flash

### STM32CubeIDE

1. `File → Open Projects from File System…` → arahkan ke folder repository.
2. Impor **proyek yang sesuai** dengan board dan jalur komunikasi (§1–2). Untuk mesin 110/250, pilih varian control yang benar; jangan mengimpor asumsi pin dari proyek CAN-only.
3. Pilih konfigurasi Debug atau Release, build, lalu flash masing-masing board lewat ST-LINK.

Berkas `.project`, `.cproject`, `.ioc`, dan pengaturan proyek disimpan di Git. Folder `Debug/` dan `Release/` adalah artefak build.

### Command line (`make`)

Jika memakai makefile hasil CubeIDE, gunakan toolchain GCC yang sesuai dengan versi CubeIDE pembuat makefile. Contoh lokasi instalasi yang pernah dipakai proyek ini (sesuaikan dengan mesin Anda):

```bash
export PATH="/opt/st/stm32cubeide_1.15.1/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.linux64_1.0.100.202403111256/tools/bin:$PATH"
make -C transmitter_demolition_robot/Debug all
make -C control_demolition_robot_250/Debug all
```

Bangun dulu proyek di CubeIDE jika direktori `Debug/` dan makefile belum dibuat. Contoh flash binary transmitter:

```bash
st-flash --reset write transmitter_demolition_robot/Debug/transmitter_demolition_robot.bin 0x8000000
```

Periksa nama file output sebenarnya dan pastikan target board cocok sebelum flash. Commit penambahan `control_demolition_robot_250` mencatat build bersih, tetapi **belum diuji pada hardware**.

### Regenerasi dari CubeMX

Sesudah `.ioc` diregenerasi, tinjau ulang pin, kode pada area `USER CODE`, dan inisialisasi tambahan. Pada transmitter, `MX_GPIO_ConfigureSwitchPullDown()` memasang pull-down switch setelah `MX_GPIO_Init()`; jangan hilangkan. Pada control langsung, verifikasi lagi SPI2 NRF24, CAN1, dan pemetaan PWM terhadap `PWM_SetDutyCycle()`.

---

## 13. Debugging via USB CDC

Transmitter utama dan control board yang mengaktifkan USB CDC mengekspos **Virtual COM Port** lewat konektor **USB OTG FS** (bukan port ST-LINK). Colok kabel USB kedua ke port itu, lalu:

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

- Pada control CAN-only, `LINK=DOWN` berarti RPDO1 tidak diterima selama >200 ms. Pada control `disc_nrf24`/`250`, lihat diagnostik sumber paket NRF24/CAN dan timeout 500 ms di USB CDC.

Detail lebih lanjut: `transmitter_demolition_robot/USB_CDC_DEBUG_README.md`.

---

## 14. Troubleshooting

| Gejala | Periksa |
|---|---|
| `NRF24: FAIL` pada remote | SPI2 transmitter: PB13 SCK, PC2 MISO, PB15 MOSI, PC6 CSN, PC7 CE. NRF24 harus 3,3 V; pasang kapasitor dekat modul dan periksa ground bersama. |
| Radio control langsung tidak menerima | SPI2 control: PB13/PB14/PB15, PE4 CE, PE5 CSN. Periksa channel 76, alamat radio, supply, dan register `CFG`, `ST`, `CH`, `FIFO_STATUS` pada log USB CDC. |
| Receiver baru hidup setelah reset | Periksa supply modul NRF24 saat cold boot. Firmware bridge mengulang `NRF24_Configure()` hingga 10 kali. |
| LED merah remote berkedip terus | `S0` masih di posisi emergency (`0`). |
| OLED menunjukkan SLEEP atau safety belum OK | Netralkan semua joystick/switch. Bila nilai tengah drift, lakukan kalibrasi dengan `S1_2` saat SLEEP, lalu unlock `S1_1`. |
| Motor tidak mau start | Tahan `S2_1` sekitar 100 ms. Setelah link putus, lepas lalu tekan lagi untuk re-arm. Periksa heartbeat `0x702` pada CAN kabel atau Auto-ACK pada radio. |
| Mode berubah tetapi aktuator diam | Pada control langsung, lepaskan keempat sumbu joystick ke tengah setelah pindah mode; kombinasi `S5=11` adalah invalid dan menolkan output. |
| `LINK=DOWN` pada control CAN-only | Periksa transceiver, terminasi 120 Ω di kedua ujung, PD0/PD1, 500 kbps, bridge Node-ID 1, dan frame `0x181`. |
| Solenoid lemah atau arah salah | Cocokkan nomor kanal, pin fisik, dan arah pada `PWM_SetDutyCycle()` sebelum mengubah `pwm_limits[]`. Kalibrasi lama CAN-only tidak berlaku pada 110/250. |
| Solenoid tetap ON setelah joystick netral | Matikan daya aktuator, lalu ukur driver dan sinyal PWM. Periksa saturasi transistor dan batas duty kanal; tabel 110/250 saat ini masih `{0,100}`. |
| Joystick kanan melonjak sendiri | Pastikan PE3 (CS accelerometer Discovery) dijaga HIGH dan switch `S1_1` ada di PE4. Jangan menambah ADC pin mengambang ke scan. |
| OLED blank atau USB CDC tidak muncul | Periksa I2C3 PA8/PC9 dan konektor USB OTG FS. Jika clock jatuh ke HSI fallback, USB bisa tidak stabil; lihat pesan boot pada log. |

Detail debug transmitter ada di `transmitter_demolition_robot/USB_CDC_DEBUG_README.md`.

---

## Lisensi

Kode HAL & middleware STM32 mengikuti lisensi STMicroelectronics masing-masing. CANopenNode mengikuti Apache-2.0.
