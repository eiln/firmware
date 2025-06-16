
#ifndef __PHAL_G4_H__
#define __PHAL_G4_H__

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(STM32G474xx)
#include "stm32g4xx.h"
#else
#error "Please define a MCU arch"
#endif

#endif // __PHAL_G4_H__
