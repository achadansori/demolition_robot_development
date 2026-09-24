# Firmware Demolition Robot

Kumpulan proyek STM32CubeIDE untuk remote, penerima radio/CAN, dan control board robot demolisi. Paket kendali berukuran 8 byte dan dipakai bersama oleh jalur NRF24L01+ serta CAN. Pilih firmware control sesuai mesin dan pengkabelan yang dipakai sebelum melakukan flash.

## Proyek di repository

| Proyek | MCU | Peran |
|---|---|---|
| [`transmitter_demolition_robot/`](transmitter_demolition_robot/) | STM32F407VG | Remote utama. Membaca joystick dan switch, menampilkan OLED, mengirim lewat NRF24 atau CAN kabel. CAN dipilih saat heartbeat control board terdeteksi. |
| [`transmitter_demolition_robot_disc_nrf24/`](transmitter_demolition_robot_disc_nrf24/) | STM32F407VG | Varian transmitter Discovery dengan NRF24. |
| [`receiver_demolition_robot/`](receiver_demolition_robot/) | STM32F407VG | Bridge NRF24 ke CANopen, Node-ID 1. |
| [`receiver_demolition_robot_stm32f103c8t6/`](receiver_demolition_robot_stm32f103c8t6/) | STM32F103C8 | Alternatif bridge NRF24 ke CANopen, Node-ID 1. |
| [`control_demolition_robot/`](control_demolition_robot/) | STM32F407VG | Control board untuk jalur CANopen, Node-ID 2. |
| [`control_demolition_robot_disc_nrf24/`](control_demolition_robot_disc_nrf24/) | STM32F407VG | Control board mesin 110; menerima paket langsung dari NRF24 atau CAN kabel. |
| [`control_demolition_robot_250/`](control_demolition_robot_250/) | STM32F407VG | Control board mesin 250, berbasis firmware `disc_nrf24` dengan logika brake dan katup Cylinder 1 khusus mesin 250. |

Setiap folder merupakan proyek STM32CubeIDE tersendiri dengan berkas `.ioc`, `.project`, dan `.cproject`. Jangan mencampur binary dari varian 110 dan 250.

## Jalur komunikasi

```text
Remote STM32F407 ── NRF24 ────────────────> Control disc_nrf24 / 250
       │                                      ↑
       ├── CAN kabel, ID 0x181 ──────────────┘
       │
       └── NRF24 ──> Bridge F407 / F103 ── CANopen TPDO1, ID 0x181 ──> Control CAN
```

Transmitter utama mengirim lewat radio saat tidak menerima heartbeat CAN `0x702` dari control board. Jika heartbeat terdeteksi, transmitter memakai CAN kabel pada 500 kbps, mengirim frame standar `0x181` setiap 20 ms, dan mematikan radio. Batas usia heartbeat adalah 300 ms. Control `disc_nrf24` dan `250` membaca CAN lebih dulu jika frame tersedia, lalu NRF24. Bridge CANopen meneruskan payload radio yang sama melalui object dictionary `0x2000` dan TPDO1. Jalur CAN memerlukan transceiver dan terminasi bus yang sesuai.

Parameter bersama: NRF24 channel 76 (2476 MHz), alamat `E7 E7 E7 E7 E7`, payload tetap 8 byte; CAN 500 kbps. Definisi paket ada di `ctrl_link.h` pada proyek terkait dan harus tetap sinkron.

### Format paket kendali

| Byte | Isi |
|---|---|
| 0–3 | Joystick kiri X/Y, kanan X/Y; nilai tengah 127 |
| 4 | Persentase baterai remote; bukan input potensiometer flow |
| 5 | Cadangan |
| 6–7 | Bit switch, little endian |

Bit switch: 0–3 tombol joystick kiri 1/2 dan kanan 1/2; 4 `S0` (1 normal, 0 emergency); 5 `S1_1` (unlock); 6 `S1_2` (kalibrasi); 7 `S2_1` (start motor); 8–10 switch cadangan; 11–12 `S5_1`/`S5_2` (mode); 13 `motor_active`. Transmitter utama juga menggunakan bit 14 untuk status `unlocked`; periksa `var.h` dan decoder pada firmware yang dipasangkan.

## Operasi dan keselamatan

1. Nyalakan remote dengan `S0 = 1`. Remote mulai dalam SLEEP; semua joystick harus netral dan switch lain harus OFF.
2. Jika perlu, tahan `S1_2` sekitar 100 ms saat SLEEP untuk kalibrasi joystick. Tahan `S1_1` sekitar 100 ms untuk keluar dari SLEEP.
3. Tahan `S2_1` sekitar 100 ms untuk mengaktifkan motor. Setelah link terputus dan pulih, lepas lalu tekan lagi `S2_1` untuk re-arm.
4. Pilih mode dengan `S5_1`/`S5_2`: `00` UPPER, `01` LOWER, `10` DUAL. Kombinasi `11` mematikan seluruh output gerak.

Pada control `disc_nrf24` dan `250`, perubahan mode menahan output gerak sampai keempat sumbu joystick kembali ke tengah (toleransi ±10). `S0 = 0` mematikan relay emergency, motor starter, dan output aktuator. Hilangnya paket radio/CAN lebih dari 500 ms memulai transisi output menuju nol dan mewajibkan start motor ulang. Firmware control ini juga menggunakan independent watchdog sekitar 800 ms dan mematikan output pada jalur fault.

**Mesin hidrolik harus tetap memiliki emergency stop fisik yang memutus daya aktuator secara independen dari firmware.** Setelah mengubah pin atau kalibrasi PWM, uji tiap kanal di bench dengan pompa mati dan tanpa beban sebelum mengoperasikan mesin.

## Mesin 250: logika dan pin output saat ini

Varian 250 memakai 20 indeks kanal: 18 output PWM pada 500 Hz dan dua output tool digital. Kanal 1 adalah **brake** (`PA3`): 100% pada UPPER, 0% pada LOWER/DUAL. Kanal 2 adalah **katup Cylinder 1** (`PA1`): ON/OFF oleh tombol joystick kanan 1 pada UPPER. Joystick kanan Y mengendalikan Cylinder 2; fungsi pindah sumbu ke Cylinder 1 dari varian 110 tidak digunakan. Pada DUAL, tombol kanan 1 bersama joystick kanan Y menggerakkan kedua track.

| Kanal | Fungsi | Pin |
|---:|---|---|
| 1–2 | Brake; katup Cylinder 1 ON | PA3; PA1 |
| 3–4 | Cylinder 2 OUT; IN | PE9; PB1 |
| 5–6 | Cylinder 3 OUT; IN | PE13; PE11 |
| 7–8 | Cylinder 4 OUT; IN | PD15; PD13 |
| 9–10 | Tool 1; Tool 2 (GPIO digital) | PD12; PD14 |
| 11–12 | Slew CW; CCW | PB0; PE14 |
| 13–14 | Outrigger kiri UP; DOWN | PA0; PA2 |
| 15–16 | Outrigger kanan UP; DOWN | PC8; PB4 |
| 17–18 | Track kanan FORWARD; BACKWARD | PC7; PC6 |
| 19–20 | Track kiri FORWARD; BACKWARD | PB5; PC9 |

Relay emergency: `PB8`; motor starter: `PE6`. NRF24 di control 250 memakai SPI2 (`PB13` SCK, `PB14` MISO, `PB15` MOSI), `PE4` CE, `PE5` CSN. CAN1 memakai `PD0` RX dan `PD1` TX. Sumber kebenaran untuk pemetaan output adalah `PWM_SetDutyCycle()` di [`control_demolition_robot_250/Core/Src/pwm.c`](control_demolition_robot_250/Core/Src/pwm.c).

**Limit PWM varian 250 saat ini masih `{0, 100}` pada semua kanal.** Angka kalibrasi dari versi lama belum diterapkan karena pemetaan kanal ke pin sudah berubah. Verifikasi arah, solenoid fisik, dan batas duty tiap kanal di bench sebelum digunakan. Commit penambahan varian 250 mencatat build berhasil, tetapi belum ada pengujian hardware.

## Build dan flash

1. Buka STM32CubeIDE dan impor folder proyek yang dibutuhkan melalui **File → Open Projects from File System**.
2. Pilih konfigurasi **Debug** atau **Release**, lalu build proyek tersebut.
3. Flash firmware yang sesuai ke tiap board melalui ST-LINK. Cocokkan MCU, pengkabelan, pasangan transmitter/control, dan varian mesin sebelum menyalakan daya aktuator.

Direktori `Debug/` dan `Release/` adalah hasil build dan tidak disimpan di Git. Jika `.ioc` diregenerasi, tinjau ulang kode di bagian `USER CODE`, konfigurasi GPIO, dan peta pin sebelum flash. Panduan debug USB CDC transmitter ada di [`USB_CDC_DEBUG_README.md`](transmitter_demolition_robot/USB_CDC_DEBUG_README.md).

## Lisensi

HAL dan middleware STM32 mengikuti lisensi STMicroelectronics dalam folder masing-masing. CANopenNode mengikuti lisensi yang disertakan bersama sumbernya.
