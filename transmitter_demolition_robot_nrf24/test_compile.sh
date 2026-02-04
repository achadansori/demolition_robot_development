#!/bin/bash
# Simple syntax check for NRF24 driver

GCC=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-gcc

# Include paths
INCLUDES="-I./Core/Inc"
INCLUDES="$INCLUDES -I./Drivers/CMSIS/Device/ST/STM32F4xx/Include"
INCLUDES="$INCLUDES -I./Drivers/CMSIS/Include"
INCLUDES="$INCLUDES -I./Drivers/STM32F4xx_HAL_Driver/Inc"

# Defines
DEFINES="-DSTM32F401xE -DUSE_HAL_DRIVER"

# Compiler flags
CFLAGS="-mcpu=cortex-m4 -mthumb -std=gnu11 -Wall -Wextra -fsyntax-only"

echo "Checking NRF24 driver syntax..."
$GCC $CFLAGS $INCLUDES $DEFINES Core/Src/nrf24.c -o /dev/null 2>&1

if [ $? -eq 0 ]; then
    echo "✓ nrf24.c syntax OK"
else
    echo "✗ nrf24.c has errors"
fi

echo ""
echo "Checking SPI driver syntax..."
$GCC $CFLAGS $INCLUDES $DEFINES Core/Src/spi.c -o /dev/null 2>&1

if [ $? -eq 0 ]; then
    echo "✓ spi.c syntax OK"
else
    echo "✗ spi.c has errors"
fi

echo ""
echo "Checking main.c syntax..."
$GCC $CFLAGS $INCLUDES $DEFINES -I./USB_DEVICE/App -I./USB_DEVICE/Target -I./Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc Core/Src/main.c -o /dev/null 2>&1

if [ $? -eq 0 ]; then
    echo "✓ main.c syntax OK"
else
    echo "✗ main.c has errors (showing first 20 lines):"
    $GCC $CFLAGS $INCLUDES $DEFINES -I./USB_DEVICE/App -I./USB_DEVICE/Target -I./Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc Core/Src/main.c -o /dev/null 2>&1 | head -20
fi
