#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/usart/usart.h"

#if defined(STM32F407xx)
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#endif

#include "main.h"
#include "svpwm.h"

volatile uint32_t tick_ms = 0; // Systick 1ms counter
volatile uint32_t tim_counter = 0;

#define TIM_ARR_VAL (0xffff)  // timer counter/period
#define SVPWM_PERIOD TIM_ARR_VAL

// TIM1: Fast timer, timer that generates PWM
// TIM7: Slow timer, timer to generate an interrupt to set updated PWM duty cycles

// pins on F4 discovery board
#define TIM1_AF 1
GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_AF(GPIOE, 8, TIM1_AF, GPIO_OUTPUT_ULTRA_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN), // TIM1_CH1N
    GPIO_INIT_AF(GPIOE, 9, TIM1_AF, GPIO_OUTPUT_ULTRA_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN), // TIM1_CH1
    GPIO_INIT_AF(GPIOE, 10, TIM1_AF, GPIO_OUTPUT_ULTRA_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN), // TIM1_CH2N
    GPIO_INIT_AF(GPIOE, 11, TIM1_AF, GPIO_OUTPUT_ULTRA_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN), // TIM1_CH2
    GPIO_INIT_AF(GPIOE, 12, TIM1_AF, GPIO_OUTPUT_ULTRA_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN), // TIM1_CH3N
    GPIO_INIT_AF(GPIOE, 13, TIM1_AF, GPIO_OUTPUT_ULTRA_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN), // TIM1_CH3
};

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .vco_output_rate_target_hz  =160000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

void HardFault_Handler();
void timer_init();
void timer_start();

int main()
{
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);

    timer_init();
    timer_start();

    while (1)
    {
        ;
    }

    return 0;
}

void init_slow_tim(uint32_t fs)
{
    // init IRQ tim (TIM7)
    if (RCC->CFGR & RCC_CFGR_PPRE1_2) fs *= 2; // RM0394 pg 188 (timer clock doubles if apb1 prescaler != 0)
    #if (defined(STM32F407xx) || defined(STM32F732xx))
        RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    #else
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
    #endif
    TIM7->CR1 = 0;
    TIM7->PSC = (APB1ClockRateHz / fs) - 1; // 1 MHz
    TIM7->ARR = TIM_ARR_VAL; // 1 MHz / 1 => 1 KHz if 1000 samples for full period
    TIM7->DIER |= TIM_DIER_UIE;
}

void init_fast_tim(TIM_TypeDef *TIMX)
{
    // init PWM tim (TIM1/TIM8)
    if (TIMX == TIM1)
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    else if (TIMX == TIM8)
        RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    else
        HardFault_Handler();
    TIMX->CR1 &= ~TIM_CR1_CEN;
    TIMX->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIMX->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIMX->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
    /* Enable auto-reload preload, leave the default of upcounting */
    TIMX->CR1 |= TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CMS_1;
    /* Enable first channel (OC1), leave the default of active high */
    TIMX->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    /* Enable main output */
    TIMX->BDTR |= TIM_BDTR_MOE | 0b111; // 0b111: max deadtime for active complementary
    TIMX->PSC = 1;
    TIMX->ARR = TIM_ARR_VAL;

    TIMX->CCR1 = 0;
    TIMX->CCR2 = 0;
    TIMX->CCR3 = 0;
}

void timer_init()
{
    init_slow_tim(APB1ClockRateHz / 2);
    init_fast_tim(TIM1);
}

void timer_start()
{
    TIM7->CR1     |= TIM_CR1_CEN;
    NVIC->ISER[1] |= 1 << (TIM7_IRQn - 32);
    TIM1->CR1 |= TIM_CR1_CEN;
}

static void set_pwm_sixstep(void)
{
    // 6 step mode
    tim_counter += 1;

    int x, y, z;
    int c = tim_counter % 3;
    if (c == 0)
    {
        x = 1;
        y = 0;
        z = 0;
    }
    else if (c == 1)
    {
        x = 0;
        y = 1;
        z = 0;
    }
    else if (c == 2)
    {
        x = 0;
        y = 0;
        z = 1;
    }
    #if 0
    TIM1->CCR1 = (uint16_t)(x * SVPWM_PERIOD);
    TIM1->CCR2 = (uint16_t)(y * SVPWM_PERIOD);
    TIM1->CCR3 = (uint16_t)(z * SVPWM_PERIOD);
    #else
    // If using inverted channel pair (ie TIM1_CH1N, TIM1_CH2N, TIM1_CH3N)
    TIM1->CCR1 = (uint16_t)((1 - z) * SVPWM_PERIOD);
    TIM1->CCR2 = (uint16_t)((1 - y) * SVPWM_PERIOD);
    TIM1->CCR3 = (uint16_t)((1 - x) * SVPWM_PERIOD);
    #endif
}

static void set_pwm_svpwm(void)
{
    // sin wave/svpwm mode
    tim_counter += 4; // increment by +4 to move faster

    int32_t angle = tim_counter % 360;
    float theta = DEG2RAD(angle);

    //int32_t ia = sin_wav_lut1[tim_counter % LUT_COUNT];
    //int32_t ib = sin_wav_lut2[tim_counter % LUT_COUNT];
    int32_t vdc = 256; // resolution of ia/ib/ic
    int32_t ia = (vdc / 2) * sin(theta); // make fake ia/ib/ic instead of actual i readings
    int32_t ib = (vdc / 2) * sin(theta + 2 * M_PI / 3);

    int32_t ialpha, ibeta;
    clarke_transform(ia, ib, &ialpha, &ibeta); // clarke
    int32_t id, iq;
    park_transform(ialpha, ibeta, theta, &id, &iq); // park

    int32_t vd = id; // no PID for now
    int32_t vq = iq;
    int32_t valpha, vbeta;
    park_transform(vd, vq, -theta, &valpha, &vbeta); // ipark
    valpha = valpha * SQRT3VAL; // vout_max = vdc * sqrt(3) / 2, already did /2 in ia generation
    vbeta = vbeta * SQRT3VAL;

    int32_t t1, t2, s1, s2, s3;
    int sector;
    svpwm_calc_weight(valpha, vbeta, &t1, &t2, &sector);
    svpwm_calc_duty(t1, t2, sector, vdc, &s1, &s2, &s3);

    TIM1->CCR1 = (uint16_t)(((float)s1 / vdc) * SVPWM_PERIOD);
    TIM1->CCR2 = (uint16_t)(((float)s2 / vdc) * SVPWM_PERIOD);
    TIM1->CCR3 = (uint16_t)(((float)s3 / vdc) * SVPWM_PERIOD);
}

void TIM7_IRQHandler() // IRQ for slow timer, i.e. setting new PWM duty cycle
{
	TIM7->SR &= ~TIM_SR_UIF;
    #if 0
    set_pwm_sixstep();
    #else
    set_pwm_svpwm();
    #endif
}

void SysTick_Handler(void)
{
    tick_ms++;
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}
