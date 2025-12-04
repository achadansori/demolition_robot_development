# PWM Control untuk TIP122 Transistor Switching

## Spesifikasi PWM
- **Frequency:** 10kHz (100μs period)
- **Duty Cycle:** 0-100%
- **Output Voltage:** 0-3.3V (STM32F401 GPIO)
- **Terlihat di Oscilloscope:** ✅ Ya (10kHz clearly visible)

## TIP122 Wiring

```
STM32 GPIO (PWM) ──┬── 1kΩ Resistor ──── TIP122 Base (pin 1)
                   │
                   └── GND ──────────────── TIP122 Emitter (pin 2)

Load (+) ───────────────────────────────── TIP122 Collector (pin 3)
Load (-) ───────────────────────────────── GND
```

### Resistor Calculation
- **Base Resistor:** 1kΩ - 2.2kΩ (recommended)
- STM32 max current per pin: 25mA
- TIP122 base current needed: ~5-10mA for full saturation
- Formula: R = (3.3V - 0.7V) / 10mA ≈ 260Ω (gunakan 1kΩ untuk safety)

## PWM to Voltage Mapping

| Duty Cycle | Average Voltage | TIP122 State | Load Power |
|------------|-----------------|--------------|------------|
| 0%         | 0V              | OFF          | 0%         |
| 25%        | 0.825V          | Partial      | ~25%       |
| 50%        | 1.65V           | Partial      | ~50%       |
| 75%        | 2.475V          | Partial      | ~75%       |
| 100%       | 3.3V            | Fully ON     | 100%       |

## Oscilloscope View

### 0% Duty Cycle (OFF)
```
     _______________________________________________
    |
____|
```

### 50% Duty Cycle (Half Power)
```
     ____      ____      ____      ____      ____
    |    |    |    |    |    |    |    |    |    |
____|    |____|    |____|    |____|    |____|    |____
    <--100μs-->
```

### 100% Duty Cycle (Fully ON)
```
    ________________________________________________
   |
___|
```

## Code Usage

```c
// Initialize PWM system
PWM_Init();

// Set cylinder 3 OUT to 50% power (1.65V average to TIP122 base)
PWM_SetDutyCycle(PWM_5_CYLINDER_3_OUT, 50);

// Set track right forward to full power (3.3V to TIP122 base)
PWM_SetDutyCycle(PWM_17_TRACK_RIGHT_FORWARD, 100);

// Turn OFF cylinder 3 IN
PWM_SetDutyCycle(PWM_6_CYLINDER_3_IN, 0);

// Emergency stop all channels
PWM_StopAll();
```

## Important Notes

⚠️ **TIP122 Heat Dissipation**
- TIP122 dapat panas saat switching high current
- Gunakan heatsink untuk beban > 1A
- Maximum collector current: 5A continuous, 8A peak

⚠️ **Flyback Diode for Inductive Loads**
- Jika load adalah motor/solenoid/relay, tambahkan flyback diode
- Diode 1N4007 atau equivalent
- Cathode ke Collector, Anode ke GND

```
         TIP122
           |
    Load  ___
   (motor) |
           |
          ===  Flyback Diode (1N4007)
          |||  (Cathode up, Anode down)
           |
          GND
```

## Pin Mapping Summary

| PWM # | Function              | STM32 Pin | Timer    | TIP122 Base |
|-------|-----------------------|-----------|----------|-------------|
| 1     | Cylinder 1 OUT        | PC6       | TIM8_CH1 | Via 1kΩ     |
| 2     | Cylinder 1 IN         | PC7       | TIM8_CH2 | Via 1kΩ     |
| 5     | Cylinder 3 OUT        | PA1       | TIM2_CH2 | Via 1kΩ     |
| 6     | Cylinder 3 IN         | PB0       | TIM3_CH3 | Via 1kΩ     |
| 17    | Track Right Forward   | PE11      | TIM1_CH2 | Via 1kΩ     |
| 18    | Track Right Backward  | PE13      | TIM1_CH3 | Via 1kΩ     |
| 19    | Track Left Forward    | PD13      | TIM4_CH2 | Via 1kΩ     |
| 20    | Track Left Backward   | PE14      | TIM1_CH4 | Via 1kΩ     |
| ...   | (See pwm.h for all)   | ...       | ...      | ...         |

## Testing dengan Oscilloscope

1. **Probe Setup:**
   - Channel 1: STM32 GPIO PWM output
   - Ground: STM32 GND
   - Trigger: Rising edge, ~1.5V

2. **Expected Waveform:**
   - Frequency: 10kHz (period 100μs)
   - Amplitude: 0V to 3.3V
   - Duty cycle varies based on `PWM_SetDutyCycle()`

3. **Timebase:** 20μs/div (untuk melihat ~5 cycles)

4. **Voltage Scale:** 1V/div
