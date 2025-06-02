#ifndef _MAIN_H_
#define _MAIN_H_

#include "adbms/adbms.h"
#include "faults.h"

#include "common/freertos/freertos.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/log/log.h"
#define printf debug_printf

// F4 Disco
#define LED_PORT_ORANGE (GPIOD)
#define LED_PORT_GREEN  (GPIOD)
#define LED_PORT_RED    (GPIOD)
#define LED_PORT_BLUE   (GPIOD)
#define LED_PIN_GREEN   (12)
#define LED_PIN_ORANGE  (13)
#define LED_PIN_RED     (14)
#define LED_PIN_BLUE    (15)

// SPI2
#define SPI_CS_PORT   (GPIOB)
#define SPI_SCK_PORT  (GPIOB)
#define SPI_MISO_PORT (GPIOB)
#define SPI_MOSI_PORT (GPIOB)
#define SPI_CS_PIN    (12)
#define SPI_SCK_PIN   (13)
#define SPI_MISO_PIN  (14)
#define SPI_MOSI_PIN  (15)

#define CHARGE_ENABLED_PORT  (GPIOC)
#define CHARGE_ENABLED_PIN   (12)

extern SPI_InitConfig_t bms_spi_config;

typedef enum
{
    BMS_STATE_IDLE = 0,
    BMS_STATE_CONNECTED,
    BMS_STATE_DISCHARGE,
    BMS_STATE_CHARGE,
} bms_state_t;

typedef struct {
    uint64_t internal_comms: 1;    // ADBMS ISOSPI
    uint64_t external_comms: 1;    // Vehicle CAN/Elcon CAN
    uint64_t internal_hardware: 1; // Undervoltage/Overvoltage/etc
    uint64_t hv_isolation: 1;      // IMD fault
    uint64_t input_psu: 1;         // Power rails

    uint64_t cell_open_wire: 1;
    uint64_t cell_uv: 1;
    uint64_t cell_ov: 1;
    uint64_t weak_cell: 1;
    uint64_t weak_pack: 1;

    uint64_t thermistor_open_wire: 1;
    uint64_t thermistor_bad: 1;
    uint64_t pack_overheat: 1;
} bms_errors_t;
static_assert(sizeof(bms_errors_t) == sizeof(uint64_t));

typedef struct
{
    bms_state_t state;

    uint32_t fault[TOTAL_AD68]; // bitfield of bms_error_t
    uint32_t first_fault_time[TOTAL_AD68][BMS_ERROR_COUNT];
    uint32_t last_fault_time[TOTAL_AD68][BMS_ERROR_COUNT];

    uint32_t fault_global;
    uint32_t fault_aux[TOTAL_AD68][TOTAL_AUX];
    uint32_t fault_cell[TOTAL_AD68][TOTAL_CELL];

    uint8_t  txData[TOTAL_AD68][DATA_LEN];
    uint8_t  rxData[TOTAL_AD68][DATA_LEN];
    uint16_t rxPec[TOTAL_AD68];
    uint8_t  rxCc[TOTAL_AD68];

    // charger
    int charger_fail_count;
    bms_errors_t errors;

} bms_t;

extern bms_t bms;
extern SemaphoreHandle_t spi1_lock;

#define bms_error log_red
#define bms_warn  log_yellow

void bms_monitor_cells(void);
void bms_monitor_temps(void);
void bms_monitor_cells_start(void);

static inline void catch_error(void)
{
    while(1)
    {
        PHAL_writeGPIO(LED_PORT_ORANGE, LED_PIN_ORANGE, 1);
        __asm__("bkpt");
        __asm__("nop");
    }
}

void HardFault_Handler(void);

#endif // _MAIN_H_
