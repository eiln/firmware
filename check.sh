#!/usr/bin/env sh

# clang-tidy source/g4_testing/main.c -- \
#     -DSTM32G474xx -I./ \
#     -I./common/STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include/ \
#     -I./common/STM32CubeG4/Drivers/CMSIS/Core/Include -I./common/freertos \
#     -I./common/STM32CubeG4/Middlewares/Third_Party/FreeRTOS/Source/include/ \
#     -I./common/STM32CubeG4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F

for file in $(find source/g4_testing -type f \( -iname '*.c' -o -iname '*.h' \)); do
  clang-tidy "$file" -- \
    -DSTM32G474xx \
    -I./ \
    -I./common/STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include/ \
    -I./common/STM32CubeG4/Drivers/CMSIS/Core/Include \
    -I./common/freertos \
    -I./common/STM32CubeG4/Middlewares/Third_Party/FreeRTOS/Source/include/ \
    -I./common/STM32CubeG4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F

done
