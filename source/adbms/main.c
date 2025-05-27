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
    GPIO_INIT_OUTPUT(SPI_MSTR_PORT, SPI_MSTR_PIN, GPIO_OUTPUT_HIGH_SPEED), // PB11
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

void HardFault_Handler(void);
void SysTick_Handler(void);
static void bms_create_threads(void);
static void bms_heartbeat(void);
static void bms_periodic(void);
static void bms_error_handler(void);

bms_t bmsmaster = {
    .state = BMS_STATE_IDLE,
    .error = BMS_ERROR_NONE,
    .fault = {0},
    .conn = 0,
};

defineStaticSemaphore(spi1_lock);

int main()
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
defineThreadStack(bms_heartbeat, 500, osPriorityNormal, 128);
defineThreadStack(bms_periodic, 2500, osPriorityNormal, 1024);
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
}

#define ADBMS_6830B_CONN ((uint32_t)0b1) // n times many TOTAL_AD68

static void bms_periodic(void)
{
    bmsmaster.conn = adbms_checkalive();
    if (bmsmaster.conn == ADBMS_6830B_CONN)
    {
        if (bmsmaster.state == BMS_STATE_IDLE)
        {
            bmsmaster.state = BMS_STATE_CONNECTED;
            printf("Connected to %d AFEs!\n", TOTAL_AD68);
        }
        PHAL_writeGPIO(LED_PORT_GREEN, LED_PIN_GREEN, 1);
        bmsmaster.error &= ~BMS_ERROR_CONN;
    }
    else
    {
        PHAL_writeGPIO(LED_PORT_GREEN, LED_PIN_GREEN, 0);
        printf("Lost connection to %d AFEs! Index: ", TOTAL_AD68);
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            if (!(bmsmaster.conn & (1 << ic)))
            printf("%d ", ic);
        }
        printf("\n");
        printf("Retrying!...\n");
        // TODO send over CAN
        bmsmaster.state = BMS_STATE_IDLE;
        bmsmaster.error |= BMS_ERROR_CONN;
    }

    switch (bmsmaster.state)
    {
        case BMS_STATE_CONNECTED:
        {
            if (bms_init())
                bmsmaster.state = BMS_STATE_ACTIVE;
        }
        break;
        case BMS_STATE_ACTIVE:
        {
            if (!bms_init())
            {
                return;
            }
            bms_monitor_cells();
            bms_monitor_temps();
            printf("--------------------------------------------------\n");
        }
        break;
        default:
        break;
    }
}

static bool is_error(void)
{
    bool ret = false;
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        if (bmsmaster.fault[ic])
        ret = true;
    }
    return ret;
}

#define print_bms_error(x) do {\
    if (bmsmaster.error & x)\
        printf("\t " #x "\n");\
} while (0);

#define print_bms_fault(ic, x) do {\
    if (bmsmaster.fault[ic] & x)\
        printf("\t IC[%d]" #x "\n", ic);\
} while (0);

static void bms_error_handler(void)
{
    if (bmsmaster.error || is_error())
    {
        PHAL_toggleGPIO(LED_PORT_RED, LED_PIN_RED);
        printf("BMS Error: 0x%08x\n", bmsmaster.error);

        print_bms_error(BMS_ERROR_CONN);
        print_bms_error(BMS_ERROR_TX);
        // TODO report error over CAN
        //if (bmsmaster.error & BMS_ERROR_TX)
        //printf("\t BMS_ERROR_TX\n");
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            printf("BMS Error IC[%d]: 0x%08x\n", ic, bmsmaster.fault[ic]);
            print_bms_fault(ic, BMS_ERROR_RXPEC);
            print_bms_fault(ic, BMS_ERROR_VREG);
            print_bms_fault(ic, BMS_ERROR_ITMP);
        }
        /* Clear Errors */
    }
    else
    {
        PHAL_writeGPIO(LED_PORT_RED, LED_PIN_RED, 0);
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
