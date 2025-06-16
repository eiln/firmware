
#ifndef __PHAL_G4_TIM_H__
#define __PHAL_G4_TIM_H__

#include "common/phal_G4/phal_g4.h"

// PWM configuration structure
typedef struct {
	TIM_TypeDef *tim;
	bool enable_comp;
	uint32_t frequency_hz;
	float duty_cycle; // 0.0 to 1.0
} PWM_Config;

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
bool PHAL_TIM_PWMInit(const PWM_Config *config);

#endif // __PHAL_G4_TIM_H__
