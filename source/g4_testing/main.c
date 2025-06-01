
#include "common/phal_G4/gpio/gpio.h"
#include "common/phal_G4/rcc/rcc.h"

#include "common/freertos/freertos.h"

#include "main.h"

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

static void ledblink1(void);
static void ledblink2(void);
static void ledblink3(void);
static void ledblink4(void);

defineThreadStack(ledblink1, 250, osPriorityNormal, 64);
defineThreadStack(ledblink2, 300, osPriorityNormal, 64);
defineThreadStack(ledblink3, 500, osPriorityNormal, 64);
defineThreadStack(ledblink4, 1000, osPriorityNormal, 64);

int main()
{
    osKernelInitialize();

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

    // Create threads
    createThread(ledblink1);
    createThread(ledblink2);
    createThread(ledblink3);
    createThread(ledblink4);

    osKernelStart(); // Go!

    while (1)
    {
        ;
    }

    return 0;
}

static void ledblink1(void)
{
    PHAL_toggleGPIO(LED_GREEN_PORT, LED_GREEN_PIN);
}

static void ledblink2(void)
{
    PHAL_toggleGPIO(LED_RED_PORT, LED_RED_PIN);
}

static void ledblink3(void)
{
    PHAL_toggleGPIO(LED_BLUE_PORT, LED_BLUE_PIN);
}

static void ledblink4(void)
{
    PHAL_toggleGPIO(LED_ORANGE_PORT, LED_ORANGE_PIN);
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}

