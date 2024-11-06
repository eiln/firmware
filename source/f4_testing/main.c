#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/psched/psched.h"
#include "common/phal_F4_F7/spi/spi.h"

#include "common/modules/Wiznet/W5500/Ethernet/W5500/w5500.h"

#define _W5500_SPI_VDM_OP_          0x00
#define _W5500_SPI_FDM_OP_LEN1_     0x01
#define _W5500_SPI_FDM_OP_LEN2_     0x02
#define _W5500_SPI_FDM_OP_LEN4_     0x03

volatile uint32_t tick_ms; // Systick 1ms counter
#define ETH_PHY_RESET_PERIOD_MS 10
#define ETH_PHY_LINK_TIMEOUT_MS 5000

// SPI1 NSS PA4
#define DAQ_SPI1_NSS_PORT GPIOA
#define DAQ_SPI1_NSS_PIN  4
// SPI1 SCK PA5
#define DAQ_SPI1_SCK_PORT GPIOA
#define DAQ_SPI1_SCK_PIN  5
// SPI1 MISO PA6
#define DAQ_SPI1_MISO_PORT GPIOA
#define DAQ_SPI1_MISO_PIN  6
// SPI1 MOSI PA7
#define DAQ_SPI1_MOSI_PORT GPIOA
#define DAQ_SPI1_MOSI_PIN  7

#define DAQ_ETH_RST_PORT   GPIOE
#define DAQ_ETH_RST_PIN    3

GPIOInitConfig_t gpio_config[] = {
    // DAQ LEDs
    GPIO_INIT_OUTPUT(GPIOA, 8, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOA, 9, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOA, 10, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOD, 13, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOD, 14, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOD, 15, GPIO_OUTPUT_LOW_SPEED),
#if 1
    // DAQ W5500 SPI
    GPIO_INIT_OUTPUT(DAQ_SPI1_NSS_PORT, DAQ_SPI1_NSS_PIN, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_AF(DAQ_SPI1_SCK_PORT, DAQ_SPI1_SCK_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(DAQ_SPI1_MISO_PORT, DAQ_SPI1_MISO_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_AF(DAQ_SPI1_MOSI_PORT, DAQ_SPI1_MOSI_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_OUTPUT_OPEN_DRAIN(DAQ_ETH_RST_PORT, DAQ_ETH_RST_PIN, GPIO_OUTPUT_LOW_SPEED),
#endif
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

dma_init_t spi1_rx_dma_config = SPI1_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi1_tx_dma_config = SPI1_TXDMA_CONT_CONFIG(NULL, 1);

SPI_InitConfig_t daq_spi1_config = {
    .data_len  = 8,
    .nss_sw = false,
    .nss_gpio_port = DAQ_SPI1_NSS_PORT,
    .nss_gpio_pin = DAQ_SPI1_NSS_PIN,
    .rx_dma_cfg = &spi1_rx_dma_config,
    .tx_dma_cfg = &spi1_tx_dma_config,
    .periph = SPI1,
};

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

void HardFault_Handler();
void ledblink();
void ledoff();

static void eth_init(void)
{
    uint32_t timeout_ms;
    timeout_ms = tick_ms;
    PHAL_writeGPIO(DAQ_ETH_RST_PORT, DAQ_ETH_RST_PIN, 0);
    while (tick_ms - timeout_ms < ETH_PHY_RESET_PERIOD_MS);
    PHAL_writeGPIO(DAQ_ETH_RST_PORT, DAQ_ETH_RST_PIN, 1);
    timeout_ms = tick_ms;
    while (tick_ms - timeout_ms < ETH_PHY_RESET_PERIOD_MS);
}

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

#if 1
    PHAL_writeGPIO(DAQ_ETH_RST_PORT, DAQ_ETH_RST_PIN, 1);

    if (!PHAL_SPI_init(&daq_spi1_config))
        HardFault_Handler();

    SysTick_Config(SystemCoreClock / 1000);

    ledoff();

    /* Task Creation */
    //schedInit(APB1ClockRateHz);
    /* Schedule Periodic tasks here */
    //taskCreate(ledblink, 250);
    //schedStart();

    PHAL_writeGPIO(DAQ_SPI1_NSS_PORT, DAQ_SPI1_NSS_PIN, 1);
    eth_init();
    PHAL_writeGPIO(DAQ_SPI1_NSS_PORT, DAQ_SPI1_NSS_PIN, 1);

    uint32_t addr =  VERSIONR;
    addr |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);

#if 0
    PHAL_writeGPIO(DAQ_SPI1_NSS_PORT, DAQ_SPI1_NSS_PIN, 0);
    uint8_t out_data[3] = {0, 0, 0};
    uint8_t in_data[3] = {0, 0, 0};
    #if 1
	in_data[0] = (addr & 0x00FF0000) >> 16;
	in_data[1] = (addr & 0x0000FF00) >> 8;
	in_data[2] = (addr & 0x000000FF) >> 0;
    #endif

    PHAL_SPI_transfer_noDMA(&daq_spi1_config, (uint8_t *)&in_data, 3, 3, out_data);
    PHAL_writeGPIO(DAQ_SPI1_NSS_PORT, DAQ_SPI1_NSS_PIN, 1);
#endif

    uint32_t ret = PHAL_WSPI_noDMA_read32(&daq_spi1_config, addr);

    while (PHAL_SPI_busy(&daq_spi1_config))
        ;

    if (ret == 0x4)
    {
        ledblink();
        //ledoff();
        //schedStart();
    }
    else
    {
        //ledoff();
        //ledblink();
    }
#else
    schedInit(APB1ClockRateHz);
    taskCreate(ledblink, 250);
    schedStart();
#endif

    return 0;
}

void ledoff()
{
    PHAL_writeGPIO(GPIOA, 8, 0);
    PHAL_writeGPIO(GPIOA, 9, 0);
    PHAL_writeGPIO(GPIOA, 10, 0);

    PHAL_writeGPIO(GPIOD, 13, 0);
    PHAL_writeGPIO(GPIOD, 14, 0);
    PHAL_writeGPIO(GPIOD, 15, 0);
}

void ledblink()
{
    PHAL_toggleGPIO(GPIOA, 8);
    PHAL_toggleGPIO(GPIOA, 9);
    PHAL_toggleGPIO(GPIOA, 10);

    PHAL_toggleGPIO(GPIOD, 13);
    PHAL_toggleGPIO(GPIOD, 14);
    PHAL_toggleGPIO(GPIOD, 15);
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