#!/bin/bash
GCC=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-gcc

INCLUDES="-I./Core/Inc"
INCLUDES="$INCLUDES -I./Drivers/CMSIS/Device/ST/STM32F4xx/Include"
INCLUDES="$INCLUDES -I./Drivers/CMSIS/Include"
INCLUDES="$INCLUDES -I./Drivers/STM32F4xx_HAL_Driver/Inc"
INCLUDES="$INCLUDES -I./USB_DEVICE/App"
INCLUDES="$INCLUDES -I./USB_DEVICE/Target"
INCLUDES="$INCLUDES -I./Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
INCLUDES="$INCLUDES -I./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"

DEFINES="-DSTM32F401xE -DUSE_HAL_DRIVER"
CFLAGS="-mcpu=cortex-m4 -mthumb -std=gnu11 -Wall -Wextra -fsyntax-only"

echo "=========================================="
echo "COMPILE TEST - All Project Files"
echo "=========================================="
echo ""

FILES=(
    "Core/Src/nrf24.c"
    "Core/Src/spi.c"
    "Core/Src/main.c"
    "Core/Src/var.c"
    "Core/Src/switch.c"
    "Core/Src/joystick.c"
    "Core/Src/battery.c"
    "Core/Src/oled.c"
    "Core/Src/usb.c"
    "Core/Src/adc.c"
    "Core/Src/gpio.c"
    "Core/Src/i2c.c"
)

PASS=0
FAIL=0

for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        printf "Testing %-30s ... " "$(basename $file)"
        if $GCC $CFLAGS $INCLUDES $DEFINES "$file" -o /dev/null 2>/dev/null; then
            echo "✓ OK"
            ((PASS++))
        else
            echo "✗ FAIL"
            ((FAIL++))
        fi
    else
        printf "Testing %-30s ... " "$(basename $file)"
        echo "⊗ NOT FOUND"
    fi
done

echo ""
echo "=========================================="
echo "Results: $PASS passed, $FAIL failed"
echo "=========================================="

if [ $FAIL -eq 0 ]; then
    echo "✓ All files compiled successfully!"
    exit 0
else
    echo "✗ Some files have errors"
    exit 1
fi
