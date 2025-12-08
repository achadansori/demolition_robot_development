# PWM Monitor - Fast Real-time Display

## Usage

### Default (ACM0):
```bash
python3 pwm_monitor_fast.py
```

### Custom ACM Port:
```bash
python3 pwm_monitor_fast.py -p /dev/ttyACM1
```

### Custom Baudrate:
```bash
python3 pwm_monitor_fast.py -p /dev/ttyACM1 -b 115200
```

### Help:
```bash
python3 pwm_monitor_fast.py --help
```

## Features
- Real-time PWM monitoring with < 5ms latency
- Buffered serial reading for optimal performance
- ANSI escape codes for smooth display updates
- No screen clearing - selective line updates only
- 20 PWM channels displayed in grid format

## Requirements
- Python 3
- pyserial library: `pip3 install pyserial`
