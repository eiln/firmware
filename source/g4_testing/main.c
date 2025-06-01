#include "common/phal_G4/gpio/gpio.h"
#include "common/phal_G4/rcc/rcc.h"

#define LED_GREEN_PORT  GPIOB
#define LED_GREEN_PIN   7
#define LED_RED_PORT    GPIOB
#define LED_RED_PIN     5
#define LED_BLUE_PORT   GPIOA
#define LED_BLUE_PIN    15
#define LED_ORANGE_PORT GPIOB
#define LED_ORANGE_PIN  1

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_OUTPUT(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(LED_RED_PORT, LED_RED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(LED_ORANGE_PORT, LED_ORANGE_PIN, GPIO_OUTPUT_LOW_SPEED),
};

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .clock_source               =CLOCK_SOURCE_HSI,
    .use_pll                    =false,
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

int main()
{
    if (0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }

    if (!PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }

    PHAL_writeGPIO(LED_GREEN_PORT, LED_GREEN_PIN, 1);
    PHAL_writeGPIO(LED_RED_PORT, LED_RED_PIN, 1);
    PHAL_writeGPIO(LED_BLUE_PORT, LED_BLUE_PIN, 1);
    PHAL_writeGPIO(LED_ORANGE_PORT, LED_ORANGE_PIN, 1);
    while (1)
    {
        ;
    }

    return 0;
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}

