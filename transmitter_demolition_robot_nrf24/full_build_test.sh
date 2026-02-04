#!/bin/bash
GCC=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-gcc

INCLUDES="-I./Core/Inc"
INCLUDES="$INCLUDES -I./Drivers/CMSIS/Device/ST/STM32F4xx/Include"
INCLUDES="$INCLUDES -I./Drivers/CMSIS/Include"
INCLUDES="$INCLUDES -I./Drivers/STM32F4xx_HAL_Driver/Inc"
INCLUDES="$INCLUDES -I./Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
INCLUDES="$INCLUDES -I./USB_DEVICE/App"
INCLUDES="$INCLUDES -I./USB_DEVICE/Target"
INCLUDES="$INCLUDES -I./Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
INCLUDES="$INCLUDES -I./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"

DEFINES="-DSTM32F401xE -DUSE_HAL_DRIVER"
CFLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -std=gnu11 -Wall -fsyntax-only"

echo "=========================================="
echo "FULL BUILD TEST"
echo "=========================================="
echo ""

PASS=0
FAIL=0
ERRORS=""

# Test Core files
echo "Testing Core files..."
for file in Core/Src/*.c; do
    if [[ ! "$file" =~ \.bak ]]; then
        printf "  %-35s ... " "$(basename $file)"
        ERROR_MSG=$($GCC $CFLAGS $INCLUDES $DEFINES "$file" -o /dev/null 2>&1)
        if [ $? -eq 0 ]; then
            echo "✓"
            ((PASS++))
        else
            echo "✗ FAIL"
            ((FAIL++))
            ERRORS="$ERRORS\n\n=== $(basename $file) ===\n$ERROR_MSG"
        fi
    fi
done

# Test USB_DEVICE files
echo ""
echo "Testing USB_DEVICE files..."
for file in USB_DEVICE/App/*.c USB_DEVICE/Target/*.c; do
    printf "  %-35s ... " "$(basename $file)"
    ERROR_MSG=$($GCC $CFLAGS $INCLUDES $DEFINES "$file" -o /dev/null 2>&1)
    if [ $? -eq 0 ]; then
        echo "✓"
        ((PASS++))
    else
        echo "✗ FAIL"
        ((FAIL++))
        ERRORS="$ERRORS\n\n=== $(basename $file) ===\n$ERROR_MSG"
    fi
done

# Test Middleware files
echo ""
echo "Testing Middleware files..."
for file in Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c; do
    printf "  %-35s ... " "$(basename $file)"
    ERROR_MSG=$($GCC $CFLAGS $INCLUDES $DEFINES "$file" -o /dev/null 2>&1)
    if [ $? -eq 0 ]; then
        echo "✓"
        ((PASS++))
    else
        echo "✗ FAIL"
        ((FAIL++))
        ERRORS="$ERRORS\n\n=== $(basename $file) ===\n$ERROR_MSG"
    fi
done

echo ""
echo "=========================================="
echo "TOTAL: $PASS passed, $FAIL failed"
echo "=========================================="

if [ $FAIL -gt 0 ]; then
    echo ""
    echo "ERRORS FOUND:"
    echo -e "$ERRORS" | head -100
fi

exit $FAIL
