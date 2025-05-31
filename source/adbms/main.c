#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/adc/adc.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/phal_F4_F7/usart/usart.h"
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
static void bms_error_handler(void);

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

    spi1_lock = createStaticSemaphore(spi1_lock);
    bms_create_threads();

    osKernelStart(); // Go!

    return 0;
}

// ADBMS shuts off after ~2200ms
defineThreadStack(bms_heartbeat, 250, osPriorityNormal, 128);
defineThreadStack(bms_periodic, 2500, osPriorityNormal, 2056);
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

static void bms_check_connection(void)
{
    // Clear faults BMS_ERROR_SID && BMS_ERROR_RXPEC
    bool ret = adbms_checkalive();
    uint32_t sid_faults = bms_pack_faults(BMS_ERROR_SID);
    uint32_t rxpec_faults = bms_pack_faults(BMS_ERROR_RXPEC);
    if (ret == true && !sid_faults && !rxpec_faults) // faults == 0: All device IDs read
    {
        if (bms.state == BMS_STATE_IDLE)
        {
            bms.state = BMS_STATE_CONNECTED;
            printf("Connected to %d AFEs!\n", TOTAL_AD68);
        }
        PHAL_writeGPIO(LED_PORT_GREEN, LED_PIN_GREEN, 1);
    }
    else
    {
        PHAL_writeGPIO(LED_PORT_GREEN, LED_PIN_GREEN, 0);
        bms_error("Lost connection to %d AFEs!", TOTAL_AD68);

        printf("SID faults: ");
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            if (sid_faults & (1 << ic))
                printf("%d ", ic);
        }
        printf("RXPEC faults: ");
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            if (rxpec_faults & (1 << ic))
                printf("%d ", ic);
        }
        printf("\n");
        printf("Retrying!...\n");
        // TODO send over CAN
        bms.state = BMS_STATE_IDLE; // to not proceed in state machine
    }
}

static void bms_periodic(void)
{
    bms_check_connection();
    if (bms.state < BMS_STATE_CONNECTED)
    {
        return;
    }

    bms_init();
    if (bms_pack_faults(BMS_ERROR_RXPEC) || bms_pack_faults(BMS_ERROR_CONFIG))
    {
        return;
    }

    // Run regular tasks first then enter charge mode
    bms_monitor_cells();
    bms_monitor_temps();
    bms_charge_task();
}

static bool is_any_error(void)
{
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        if (bms.fault[ic]) return true;
    }
    return false;
}

#define print_bms_fault(ic, field) do {\
    if (bms.fault[ic] & BMS_GET_ERROR_MASK(field))\
        printf("\t " #field " time: %d last: %d\n", bms_get_fault_duration(ic, field), bms.last_fault_time[ic][field]);\
} while (0);

#define print_bms_cell_fault(field) do {\
    if (bms.fault_cell[ic][cell] & BMS_GET_ERROR_MASK(field))\
        printf("\t [CELL%2d]" #field "\n", cell);\
} while (0);

static void bms_error_handler(void)
{
    if (is_any_error())
    {
        PHAL_toggleGPIO(LED_PORT_RED, LED_PIN_RED);
        printf("BMS State: 0x%02x\n", bms.state);

        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            printf("BMS Error IC[%d]: 0x%08x\n", ic, bms.fault[ic]);
            print_bms_fault(ic, BMS_ERROR_SID);
            print_bms_fault(ic, BMS_ERROR_RXPEC);
            print_bms_fault(ic, BMS_ERROR_CONFIG);
            print_bms_fault(ic, BMS_ERROR_POLL_TIMEOUT);
            print_bms_fault(ic, BMS_ERROR_VPV);
            print_bms_fault(ic, BMS_ERROR_VMV);
            print_bms_fault(ic, BMS_ERROR_VA_UV);
            print_bms_fault(ic, BMS_ERROR_VA_OV);
            print_bms_fault(ic, BMS_ERROR_VD_UV);
            print_bms_fault(ic, BMS_ERROR_VD_OV);
            print_bms_fault(ic, BMS_ERROR_VREG);
            print_bms_fault(ic, BMS_ERROR_VREF2);
            print_bms_fault(ic, BMS_ERROR_ITMP_UT);
            print_bms_fault(ic, BMS_ERROR_ITMP_OT);

            for (int cell = 0; cell < TOTAL_CELL; cell++)
            {
                print_bms_cell_fault(BMS_ERROR_CELL_OW);
                print_bms_cell_fault(BMS_ERROR_CELL_UV);
                print_bms_cell_fault(BMS_ERROR_CELL_OV);
                print_bms_cell_fault(BMS_ERROR_CELL_REDUN);
            }
        }
        /* Clear Errors */
    }
    else
    {
        PHAL_writeGPIO(LED_PORT_RED, LED_PIN_RED, 0);
    }
}

#if 0

    /* handle potential errors */
    if ((sys_stat.byte & BQ769X0_SYS_STAT_ERROR_MASK) != 0) {
        if (dev_data->error_seconds_counter < 0) {
            dev_data->error_seconds_counter = 0;
        }

        err = 0;

        if (sys_stat.DEVICE_XREADY) {
            /* datasheet recommendation: try to clear after waiting a few seconds */
            if (dev_data->error_seconds_counter % 3 == 0) {
                LOG_DBG("Attempting to clear XR error");
                err |= bq769x0_write_byte(dev, BQ769X0_SYS_STAT, BQ769X0_SYS_STAT_DEVICE_XREADY);
            }
        }

#endif

void HardFault_Handler()
{
    while(1)
    {
        PHAL_writeGPIO(LED_PORT_RED, LED_PIN_RED, 1);
        __asm__("nop");
    }
}
