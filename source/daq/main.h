#ifndef _MAIN_H_
#define _MAIN_H_

// Enable the CAN2 bus
//#define EN_CAN2 1
#define ID_LWS_STANDARD 0x2b0 // hehe

typedef enum {
    RX_TAIL_CAN_RX, //!< CAN rx message parsing
    RX_TAIL_SD,     //!< SD Card
    RX_TAIL_UDP,    //!< UDP Broadcast
    RX_TAIL_USB,    //!< USB Send
    RX_TAIL_COUNT,
} rx_tail_t;

typedef enum {
    TCP_RX_TAIL_CAN_TX,
    TCP_RX_TAIL_SD,
    TCP_RX_TAIL_COUNT,
} tcp_rx_tail_t;

#include "common/freertos/freertos.h"
#include "common/log/log.h"
#include "daq_hub.h"
#include "buffer.h"

// LEDs
// #define BUILD_BACKUP_FIRMWARE
#ifndef BUILD_BACKUP_FIRMWARE
#define HEARTBEAT_LED_PORT   GPIOD
#define HEARTBEAT_LED_PIN    13
#define CONNECTION_LED_PORT  GPIOD
#define CONNECTION_LED_PIN   14
#else // BUILD_BACKUP_FIRMWARE (swap LED colors for backup firmware)
#define HEARTBEAT_LED_PORT   GPIOD
#define HEARTBEAT_LED_PIN    14
#define CONNECTION_LED_PORT  GPIOD
#define CONNECTION_LED_PIN   13
#endif // BUILD_BACKUP_FIRMWARE
#define ERROR_LED_PORT       GPIOD
#define ERROR_LED_PIN        15

// SD
#define SD_ACTIVITY_LED_PORT GPIOA
#define SD_ACTIVITY_LED_PIN  9
#define SD_ERROR_LED_PORT    GPIOA
#define SD_ERROR_LED_PIN     8
#define SD_DETECT_LED_PORT GPIOA
#define SD_DETECT_LED_PIN  10
#define SD_CD_PORT         GPIOD
#define SD_CD_PIN          4

// W5500 ETH SPI1
#define ETH_CS_PORT   GPIOA
#define ETH_CS_PIN    4
#define ETH_SCK_PORT  GPIOA
#define ETH_SCK_PIN   5
#define ETH_MISO_PORT GPIOA
#define ETH_MISO_PIN  6
#define ETH_MOSI_PORT GPIOA
#define ETH_MOSI_PIN  7
#define ETH_RST_PORT  GPIOE
#define ETH_RST_PIN   3

// LTE USART6
#define LTE_UART_TX_PORT GPIOC
#define LTE_UART_TX_PIN  6
#define LTE_UART_RX_PORT GPIOC
#define LTE_UART_RX_PIN  7

#define PWR_LOSS_PORT GPIOE
#define PWR_LOSS_PIN  15
#define LOG_ENABLE_PORT GPIOC
#define LOG_ENABLE_PIN  15

#define PER 1
#define GREAT PER

extern volatile uint32_t tick_ms; // Systick 1ms counter
extern b_handle_t b_rx_can;

extern void HardFault_Handler();

#define TCP_TX_ITEM_COUNT  32   // This is enough
#define DCAN_RX_ITEM_COUNT 4000 // TODO bump this up for bootloader
#define CAN2_TX_ITEM_COUNT 32
#define TCP_RX_ITEM_COUNT  100 // TODO bump

extern QueueHandle_t tcp_tx_queue;
extern QueueHandle_t dcan_rx_queue;
extern QueueHandle_t can2_tx_queue;
extern timestamped_frame_t tcp_rx_buf[TCP_RX_ITEM_COUNT];

extern SemaphoreHandle_t spi1_handle;
extern SemaphoreHandle_t tcp_rx_handle;
extern SemaphoreHandle_t ff_handle;
extern volatile uint64_t can_hit_count;

#endif
