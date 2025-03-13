#ifndef _MAIN_H_
#define _MAIN_H_

#include "common/phal_F4_F7/spi/spi.h"
#include "common/log/log.h"

#define LED_PORT_ORANGE GPIOD
#define LED_PORT_GREEN  GPIOD
#define LED_PORT_RED    GPIOD
#define LED_PORT_BLUE   GPIOD
#define LED_PIN_ORANGE  13
#define LED_PIN_GREEN   12
#define LED_PIN_RED     14
#define LED_PIN_BLUE    15

// SPI2
#define SPI_CS_PORT   GPIOB
#define SPI_SCK_PORT  GPIOB
#define SPI_MISO_PORT GPIOB
#define SPI_MOSI_PORT GPIOB
#define SPI_CS_PIN    12
#define SPI_SCK_PIN   13
#define SPI_MISO_PIN  14
#define SPI_MOSI_PIN  15

extern SPI_InitConfig_t eth_spi_config;
extern volatile uint32_t tick_ms; // Systick 1ms counter

static inline void catch_error(void)
{
    while(1)
    {
        PHAL_writeGPIO(LED_PORT_ORANGE, LED_PIN_ORANGE, 1);
        __asm__("bkpt");
        __asm__("nop");
    }
}

#endif // _MAIN_H_
