#include "common/phal/rcc.h"
#include "common/phal/gpio.h"
#include "common/phal/adc.h"
#include "common/phal/dma.h"
#include "common/phal/spi.h"
#include "common/phal/usart.h"
#include "common/freertos/freertos.h"
#include "common/log/log.h"

#include "main.h"
#include "adbms/adbms.h"
#include "bms/charge.h"

dma_init_t spi_rx_dma_config = SPI2_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi_tx_dma_config = SPI2_TXDMA_CONT_CONFIG(NULL, 1);
SPI_InitConfig_t bms_spi_config = {
    .data_len  = 8,
    .nss_sw = false,
    .nss_gpio_port = SPI_CS_PORT,
    .nss_gpio_pin = SPI_CS_PIN,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config,
    .periph = SPI2,
};

dma_init_t usart_tx_dma_config = USART2_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART2_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t usart_config = {
   .baud_rate   = 115200,
   .word_length = WORD_8,
   .stop_bits   = SB_ONE,
   .parity      = PT_NONE,
   .hw_flow_ctl = HW_DISABLE,
   .ovsample    = OV_16,
   .obsample    = OB_DISABLE,
   .periph      = USART2,
   .wake_addr = false,
   .usart_active_num = USART2_ACTIVE_IDX,
   .tx_dma_cfg = &usart_tx_dma_config,
   .rx_dma_cfg = &usart_rx_dma_config
};
DEBUG_PRINTF_USART_DEFINE(&usart_config)

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_OUTPUT(LED_PORT_ORANGE, LED_PIN_ORANGE, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(LED_PORT_GREEN, LED_PIN_GREEN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(LED_PORT_RED, LED_PIN_RED, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(LED_PORT_BLUE, LED_PIN_BLUE, GPIO_OUTPUT_LOW_SPEED),

    GPIO_INIT_USART2TX_PA2, // PA2
    GPIO_INIT_USART2RX_PA3, // PA3

    GPIO_INIT_OUTPUT(SPI_CS_PORT, SPI_CS_PIN, GPIO_OUTPUT_HIGH_SPEED), // PB12
    GPIO_INIT_AF(SPI_SCK_PORT, SPI_SCK_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN), // PB13
    GPIO_INIT_AF(SPI_MISO_PORT, SPI_MISO_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN), // PB14
    GPIO_INIT_AF(SPI_MOSI_PORT, SPI_MOSI_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN), // PB15

    GPIO_INIT_INPUT(CHARGE_ENABLED_PORT, CHARGE_ENABLED_PIN, GPIO_INPUT_PULL_DOWN),
};

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

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

int main(void);
void HardFault_Handler(void);
static void bms_create_threads(void);
static void bms_heartbeat(void);
static void bms_periodic(void);

bms_t bms = {
    .state = BMS_STATE_IDLE,

    .fault_global = 0,
    .fault = {0},
    .first_fault_time = {0},
    .last_fault_time = {0},
    .fault_aux = {0},
    .fault_cell = {0},

    .charger_fail_count = 0,
};

defineStaticSemaphore(spi1_lock);

int main(void)
{
    osKernelInitialize();

    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    if (!PHAL_SPI_init(&bms_spi_config))
    {
        HardFault_Handler();
    }
    PHAL_writeGPIO(SPI_CS_PORT, SPI_CS_PIN, 1);

    NVIC_EnableIRQ(SysTick_IRQn);

    if(!PHAL_initUSART(&usart_config, APB1ClockRateHz))
    {
        HardFault_Handler();
    }
    log_yellow("PER PER PER\n");

    PHAL_writeGPIO(LED_PORT_GREEN, LED_PIN_GREEN, 0);
    PHAL_writeGPIO(LED_PORT_BLUE, LED_PIN_BLUE, 0);
    PHAL_writeGPIO(LED_PORT_RED, LED_PIN_RED, 0);
    PHAL_writeGPIO(LED_PORT_ORANGE, LED_PIN_ORANGE, 0);

    spi1_lock = createStaticSemaphore(spi1_lock);
    bms_create_threads();

    osKernelStart(); // Go!

    return 0;
}

// ADBMS shuts off after ~2200ms
defineThreadStack(bms_heartbeat, 250, osPriorityNormal, 128);
defineThreadStack(bms_periodic, 500, osPriorityNormal, 2056);
defineThreadStack(bms_error_handler, 250, osPriorityNormal, 1024);

static void bms_create_threads(void)
{
    createThread(bms_heartbeat);
    createThread(bms_periodic);
    createThread(bms_error_handler);
}

static void bms_heartbeat(void)
{
    PHAL_toggleGPIO(LED_PORT_BLUE, LED_PIN_BLUE);
    // TODO send state, etc over CAN
}

static void bms_update_connection(void)
{
    adbms_checkalive();
    if (bms_any_fault(BMS_ERROR_RXPEC) || bms_any_fault(BMS_ERROR_SID))
    {
        PHAL_writeGPIO(LED_PORT_GREEN, LED_PIN_GREEN, 0);
        bms_error("Lost connection to %d AFEs!", TOTAL_AD68);
        bms_error("Retrying!...\n");
        // TODO send over CAN
        bms.state = BMS_STATE_IDLE; // to not proceed in state machine
    }
    else
    {
        if (bms.state == BMS_STATE_IDLE)
        {
            bms.state = BMS_STATE_CONNECTED;
            printf("Connected to %d AFEs!\n", TOTAL_AD68);
        }
        PHAL_writeGPIO(LED_PORT_GREEN, LED_PIN_GREEN, 1);
    }
}

static void bms_periodic(void)
{
    bms_update_connection();
    if (bms.state < BMS_STATE_CONNECTED)
    {
        return;
    }

    if (bms.state == BMS_STATE_CONNECTED)
    {
        bms_init();
        if (bms_any_fault(BMS_ERROR_RXPEC) || bms_any_fault(BMS_ERROR_CONFIG))
        {
            bms.state = BMS_STATE_IDLE;
            return;
        }
        bms_monitor_cells_start();
        if (bms_any_fault(BMS_ERROR_RXPEC))
        {
            bms.state = BMS_STATE_IDLE;
            return;
        }
        bms.state = BMS_STATE_DISCHARGE;
    }

    if (bms.state >= BMS_STATE_DISCHARGE)
    {
        // Run regular tasks first then enter charge mode
        bms_monitor_cells();
        bms_monitor_temps();
        bms_charge_task();
    }
}

void HardFault_Handler()
{
    while(1)
    {
        PHAL_writeGPIO(LED_PORT_RED, LED_PIN_RED, 1);
        __asm__("nop");
    }
}
