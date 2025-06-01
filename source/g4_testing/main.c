
#include "common/phal_G4/adc/adc.h"
#include "common/phal_G4/dma/dma.h"
#include "common/phal_G4/gpio/gpio.h"
#include "common/phal_G4/rcc/rcc.h"

#include "common/freertos/freertos.h"

#include "main.h"

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_OUTPUT(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(LED_RED_PORT, LED_RED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(LED_ORANGE_PORT, LED_ORANGE_PIN, GPIO_OUTPUT_LOW_SPEED),

    GPIO_INIT_ANALOG(ADC1_CH1_GPIO_Port, ADC1_CH1_Pin),
    GPIO_INIT_ANALOG(ADC1_CH2_GPIO_Port, ADC1_CH2_Pin),
    GPIO_INIT_ANALOG(ADC1_CH3_GPIO_Port, ADC1_CH3_Pin),
    GPIO_INIT_ANALOG(ADC1_CH4_GPIO_Port, ADC1_CH4_Pin),
};

volatile raw_adc_values_t raw_adc_values;

/* ADC Configuration */
ADCInitConfig_t adc_config = {
    .periph          = ADC1,
    .clock_prescaler = ADC_CLK_PRESC_6,
    .resolution      = ADC_RES_12_BIT,
    .data_align      = ADC_DATA_ALIGN_RIGHT,
    .cont_conv_mode  = false,
    .dma_mode        = ADC_DMA_OFF,
};

ADCChannelConfig_t adc_channel_config[] = {
    {.channel = ADC_CHANNEL_1,  .rank = 1,  .sampling_time = ADC_CHN_SMP_CYCLES_480},
    {.channel = ADC_CHANNEL_2,  .rank = 2,  .sampling_time = ADC_CHN_SMP_CYCLES_480},
    {.channel = ADC_CHANNEL_3,  .rank = 3,  .sampling_time = ADC_CHN_SMP_CYCLES_480},
    {.channel = ADC_CHANNEL_4,  .rank = 4,  .sampling_time = ADC_CHN_SMP_CYCLES_480},
};

dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) &raw_adc_values, sizeof(raw_adc_values) / sizeof(raw_adc_values.val1), 0b01);

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

    if (!PHAL_initADC(&adc_config, adc_channel_config, sizeof(adc_channel_config) / sizeof(ADCChannelConfig_t)))
    {
        HardFault_Handler();
    }
    // if (!PHAL_initDMA(&adc_dma_config))
    // {
    //     HardFault_Handler();
    // }
    // PHAL_startTxfer(&adc_dma_config);
    PHAL_startADC(&adc_config);

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
        raw_adc_values.val1 = PHAL_readADC(&adc_config);
        raw_adc_values.val2 = PHAL_readADC(&adc_config);
        raw_adc_values.val3 = PHAL_readADC(&adc_config);
        raw_adc_values.val4 = PHAL_readADC(&adc_config);
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

