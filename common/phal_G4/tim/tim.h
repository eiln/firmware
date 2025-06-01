
#ifndef __PHAL_G4_TIM_H__
#define __PHAL_G4_TIM_H__

#include <inttypes.h>
#include <stdbool.h>

#if defined(STM32G474xx)
#include "stm32g4xx.h"
#else
#error "Please define a MCU arch"
#endif

// PWM configuration structure
typedef struct {
    TIM_TypeDef *tim;
    bool enable_comp;
    uint32_t frequency_hz;
    float duty_cycle; // 0.0 to 1.0
} PWM_Config;

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;

#endif // __PHAL_G4_TIM_H__
