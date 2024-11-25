
#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/psched/psched.h"

// basic hello world test template for f4 disco
#include "f4_testing_common.h" // include header
// then define ifdef guard for file
#ifdef F4_TESTING_PDU_25_BRINGUP

#define MAIN_CTRL_GPIO_Port GPIOD
#define MAIN_CTRL_Pin 15

#define AUX1_CTRL_GPIO_Port GPIOD
#define AUX1_CTRL_Pin 12

#define FAN2_CTRL_GPIO_Port GPIOD
#define FAN2_CTRL_Pin 11

#define FAN1_CTRL_GPIO_Port GPIOD
#define FAN1_CTRL_Pin 9

#define PUMP1_CTRL_GPIO_Port GPIOB
#define PUMP1_CTRL_Pin 10

#define PUMP2_CTRL_GPIO_Port GPIOE
#define PUMP2_CTRL_Pin 15

#define BLT_CTRL_GPIO_Port GPIOE
#define BLT_CTRL_Pin 13

#define FV_FAN_CTRL_GPIO_Port GPIOE
#define FV_FAN_CTRL_Pin 8

#define FV_NCRIT_CTRL_GPIO_Port GPIOE
#define FV_NCRIT_CTRL_Pin 6

#define DAQ_CTRL_GPIO_Port GPIOE
#define DAQ_CTRL_Pin 4

#define FV_CRIT_CTRL_GPIO_Port GPIOE
#define FV_CRIT_CTRL_Pin 2

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_OUTPUT(GPIOC, 13, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOC, 14, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOC, 15, GPIO_OUTPUT_LOW_SPEED),

    GPIO_INIT_OUTPUT(MAIN_CTRL_GPIO_Port, MAIN_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(AUX1_CTRL_GPIO_Port, AUX1_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(FAN2_CTRL_GPIO_Port, FAN2_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(FAN1_CTRL_GPIO_Port, FAN1_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(PUMP1_CTRL_GPIO_Port, PUMP1_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(PUMP2_CTRL_GPIO_Port, PUMP2_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(BLT_CTRL_GPIO_Port, BLT_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(FV_FAN_CTRL_GPIO_Port, FV_FAN_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(FV_NCRIT_CTRL_GPIO_Port, FV_NCRIT_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(DAQ_CTRL_GPIO_Port, DAQ_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(FV_CRIT_CTRL_GPIO_Port, FV_CRIT_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
};

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .vco_output_rate_target_hz  =16000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

static void test_pdu(void);

int pdu_25_bringup_main(void)
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

    schedInit(APB1ClockRateHz);
    taskCreate(test_pdu, 250);
    schedStart();

    return 0;
}

static void test_pdu(void)
{
    PHAL_toggleGPIO(GPIOC, 13);
    PHAL_toggleGPIO(GPIOC, 14);
    PHAL_toggleGPIO(GPIOC, 15);

    PHAL_toggleGPIO(MAIN_CTRL_GPIO_Port, MAIN_CTRL_Pin);
    PHAL_toggleGPIO(AUX1_CTRL_GPIO_Port, AUX1_CTRL_Pin);
    PHAL_toggleGPIO(FAN2_CTRL_GPIO_Port, FAN2_CTRL_Pin);
    PHAL_toggleGPIO(FAN1_CTRL_GPIO_Port, FAN1_CTRL_Pin);
    PHAL_toggleGPIO(PUMP1_CTRL_GPIO_Port, PUMP1_CTRL_Pin);
    PHAL_toggleGPIO(PUMP2_CTRL_GPIO_Port, PUMP2_CTRL_Pin);
    PHAL_toggleGPIO(BLT_CTRL_GPIO_Port, BLT_CTRL_Pin);
    PHAL_toggleGPIO(FV_FAN_CTRL_GPIO_Port, FV_FAN_CTRL_Pin);
    PHAL_toggleGPIO(FV_NCRIT_CTRL_GPIO_Port, FV_NCRIT_CTRL_Pin);
    PHAL_toggleGPIO(DAQ_CTRL_GPIO_Port, DAQ_CTRL_Pin);
    PHAL_toggleGPIO(FV_CRIT_CTRL_GPIO_Port, FV_CRIT_CTRL_Pin);
}

#endif // F4_TESTING_PDU_25_BRINGUP
