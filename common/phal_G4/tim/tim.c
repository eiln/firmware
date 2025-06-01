
#include "common/phal_G4/tim/tim.h"

bool PHAL_TIM_PWMInit(const PWM_Config *config)
{
    TIM_TypeDef *tim = config->tim;

    if (tim == TIM1)
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    else
        return false;

    // // Set alternate function: PA8 -> AF6 (TIM1_CH1), PA7 -> AF6 (TIM1_CH1N)
    // GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7;
    // GPIOA->AFR[0] |= (6U << GPIO_AFRL_AFSEL7_Pos);
    // GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8;
    // GPIOA->AFR[1] |= (6U << (8U - 8) * 4);  // AFRH starts at pin 8
    // // Set output type (push-pull)
    // GPIOA->OTYPER &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);

    // Calculate timer prescaler and auto-reload for desired frequency
    uint32_t period = (APB2ClockRateHz / config->frequency_hz) - 1;
    tim->PSC = 0; // Prescaler
    tim->ARR = period;

    // Set duty cycle
    tim->CCR1 = (uint32_t)(period * config->duty_cycle);

    // PWM mode 1 on CH1 (active while CNT < CCR1)
    tim->CCMR1 &= ~TIM_CCMR1_OC1M;
    tim->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);  // PWM Mode 1
    tim->CCMR1 |= TIM_CCMR1_OC1PE;             // Preload enable

    // Enable output compare on CH1/CH1N
    tim->CCER |= TIM_CCER_CC1E;
    if (config->enable_comp)
        tim->CCER |= TIM_CCER_CC1NE;

    // Enable auto-reload preload
    tim->CR1 |= TIM_CR1_ARPE;

    // Enable main output (MOE) – needed for advanced timers
    if (tim == TIM1 || tim == TIM8 || tim == TIM20)
    {
        tim->BDTR |= TIM_BDTR_MOE;
    }

    // Enable counter
    tim->CR1 |= TIM_CR1_CEN;

    // Force update generation to load registers
    tim->EGR |= TIM_EGR_UG;

    return true;
}
