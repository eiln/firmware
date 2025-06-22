
#ifndef PHAL_G4_H_
#define PHAL_G4_H_

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(STM32G474xx)
#include "stm32g4xx.h"
#include "common/STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g474xx.h"
#else
#error "Please define a MCU arch"
#endif

#endif // PHAL_G4_H_
