#ifndef _MAIN_H_
#define _MAIN_H_

#include "common/freertos/freertos.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/log/log.h"
#define printf debug_printf

// F4 Disco
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
#define SPI_MSTR_PORT GPIOB // random pull high
#define SPI_CS_PIN    12
#define SPI_SCK_PIN   13
#define SPI_MISO_PIN  14
#define SPI_MOSI_PIN  15
#define SPI_MSTR_PIN  11

extern SPI_InitConfig_t bms_spi_config;

static inline void catch_error(void)
{
    while(1)
    {
        PHAL_writeGPIO(LED_PORT_ORANGE, LED_PIN_ORANGE, 1);
        __asm__("bkpt");
        __asm__("nop");
    }
}

static inline void mdelay2(uint32_t delay)
{
    //uint32_t start = tick_ms;
    //while (tick_ms - start < delay);
    mDelay(delay);
}

static inline uint32_t bms_gettick(void)
{
    return xTaskGetTickCount();
}

typedef enum
{
    BMS_STATE_ERROR = 0,
    BMS_STATE_IDLE,
    BMS_STATE_CONNECTED,
    BMS_STATE_ACTIVE,
    BMS_STATE_CHARGING,
} bms_state_t;

typedef struct
{
    bms_state_t state;
    uint32_t conn;
} bms_t;

void bms_monitor_cells(void);
void bms_monitor_temps(void);

#endif // _MAIN_H_
