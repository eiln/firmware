/**
 * @file daq_hub.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Data acquisition from CAN to:
 *          - SD Card
 *          - Ethernet
 *          - USB (maybe)
 * @version 0.1
 * @date 2024-02-08
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _DAQ_HUB_H_
#define _DAQ_HUB_H_

#include <stdint.h>
#include <assert.h>
#include "ff.h"
#include "common/phal_F4_F7/rtc/rtc.h"


/* Begin CAN Definitions from <linux/can.h> */

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28      : CAN identifier (11/29 bit)
 * bit 29        : error frame flag (0 = data frame, 1 = error frame)
 * bit 30        : remote transmission request flag (1 = rtr frame)
 * bit 31        : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef uint32_t canid_t;

/* End CAN Definitions from <linux/can.h> */

// 0 = CAN1, 1 = CAN2,
#define BUS_ID_CAN1 0
#define BUS_ID_CAN2 1
// TODO: add like UDP, USB, etc. ?
typedef uint8_t busid_t;

// from DAQ POV
#define DAQ_FRAME_CAN_RX  0  // RX to DAQ over CAN (interrupt), broadcast message
#define DAQ_FRAME_TCP2CAN 1  // RX to DAQ over TCP, relay to other nodes on CAN
#define DAQ_FRAME_TCP2UDS 2  // RX to DAQ over TCP, process within DAQ (encapsulates CAN msg intended for DAQ)
#define DAQ_FRAME_TCP_TX  3  // TX from DAQ over TCP
#define DAQ_FRAME_UDP_TX  4  // TX from DAQ over UDP

typedef struct __attribute__((packed))
{
    uint8_t  frame_type;   //!< command
    uint32_t tick_ms;      //!< ms timestamp of reception
    canid_t  msg_id;       //!< message id
    busid_t  bus_id;       //!< bus the message was rx'd on
    uint8_t  dlc;          //!< data length code
    uint8_t  data[8];      //!< message data
} timestamped_frame_t;

typedef enum
{
    TCP_CMD_HANDSHAKE = 0,
    TCP_CMD_CAN_FRAME = 1, // authentic CAN frame, i.e. daq can't TX CAN to itself
    TCP_CMD_UDS_FRAME = 2,
} tcp_cmd_t;

typedef enum
{
    SD_IDLE,
    SD_MOUNTED,
    SD_FILE_CREATED,
    SD_FAIL,
    SD_SHUTDOWN,
} sd_state_t;

typedef enum
{
    SD_ERROR_NONE = 0,
    SD_ERROR_MOUNT,
    SD_ERROR_FOPEN,
    SD_ERROR_FCLOSE,
    SD_ERROR_WRITE,
    SD_ERROR_DETEC,
} sd_error_t;

typedef enum __attribute__ ((__packed__))
{
    ETH_IDLE,
    ETH_LINK_DOWN,
    ETH_LINK_UP,
    ETH_FAIL,
} eth_state_t;
static_assert(sizeof(eth_state_t) == sizeof(uint8_t));

typedef enum
{
    ETH_TCP_IDLE = 0,
    ETH_TCP_LISTEN,
    ETH_TCP_ESTABLISHED,
    ETH_TCP_FAIL,
} eth_tcp_state_t;

typedef enum
{
    ETH_ERROR_NONE = 0,
    ETH_ERROR_INIT,
    ETH_ERROR_VERS,
    ETH_ERROR_UDP_SOCK,
    ETH_ERROR_UDP_SEND,
    ETH_ERROR_TCP_SOCK,
    ETH_ERROR_TCP_LISTEN,
} eth_error_t;

#define ETH_PHY_RESET_PERIOD_MS 10
#define ETH_PHY_LINK_TIMEOUT_MS 5000

// CAN Receive Buffer Configuration
#define RX_BUFF_ITEM_COUNT 2000

#define SD_NEW_FILE_PERIOD_MS   (2*60*1000) // 2 minutes
#define SD_MAX_WRITE_PERIOD_MS  500
#define SD_MAX_WRITE_COUNT      (500) // Assuming approx 1kHz  rx rate

#define UDP_MAX_WRITE_PERIOD_MS 50
#define UDP_MAX_WRITE_COUNT     (20)  // Assuming approx 1kHz  rx rate

// TCP Receive Buffer Configuration
#define TCP_RX_BUFF_ITEM_COUNT 10 // Shouldn't need to be much larger than max write count
#define TCP_MIN_RX_PERIOD_MS   10
#define TCP_MAX_WRITE_COUNT    (100)
#define TCP_MAX_CAN_TX_COUNT   (10)

typedef struct
{
    // Ethernet
    eth_state_t eth_state;
    bool eth_enable_udp_broadcast; // TODO: determine if I want this var
    bool eth_enable_tcp_reception;
    uint32_t eth_error_ct;
    uint32_t eth_last_error_time;
    eth_error_t eth_last_err;
    eth_tcp_state_t eth_tcp_state;
    uint32_t eth_tcp_last_rx_ms;
    // SD Card
    sd_state_t sd_state;
    FATFS fat_fs;
    volatile uint32_t my_watch;
    uint32_t sd_error_ct;
    sd_error_t sd_last_err;
    FRESULT sd_last_err_res;
    FIL log_fp;
    uint32_t log_start_ms;
    uint32_t last_write_ms;
    bool log_enable_sw; //!< Debounced switch state
    bool log_enable_tcp;
    bool log_enable_uds;
    // General
    uint32_t loop_time_max_ms;
    uint32_t loop_time_avg_ms;
    // FTP
    bool ftp_busy;
} daq_hub_t;
extern daq_hub_t dh;

void daq_init(void);
void daq_loop(void);
bool daq_request_sd_mount(void);

#endif
