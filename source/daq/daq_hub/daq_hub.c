/**
 * @file daq_hub.c
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

/* System Includes */
#include "common/common_defs/common_defs.h"
#include "common/modules/Wiznet/W5500/Ethernet/wizchip_conf.h"
#include "common/modules/Wiznet/W5500/Ethernet/socket.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rtc/rtc.h"
#include "common/daq/can_parse_base.h"

/* Module Includes */
#include "buffer.h"
#include "daq_hub.h"
#include "ftpd.h"
#include "main.h"
#include "sdio.h"

typedef struct
{
    wiz_NetInfo net_info;
    uint8_t udp_bc_addr[4];
    uint16_t udp_bc_port;
    uint8_t udp_bc_sock;
    uint16_t tcp_port;
    uint8_t tcp_sock;
} eth_config_t;

eth_config_t eth_config = {
    .net_info = {
        .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
        .ip 	= {192, 168, 10, 41},					// IP address of DAQ PCB
        .sn 	= {255, 255, 255, 0},					// Subnet mask
        .gw 	= {192, 168, 10, 1}	     				// Gateway address
    },
    .udp_bc_addr = {192, 168, 10, 255},                 // Broadcast address
    .udp_bc_port = 5005,
    .udp_bc_sock = 0,
    .tcp_port = 5005,
    .tcp_sock = 1,
    // NOTE: ftp uses 2, 3, 4
};

daq_hub_t dh;

uint8_t gFTPBUF[_FTP_BUF_SIZE];

// Local protoptypes
static void can_tx_periodic(void);
static void can_relay_can2_periodic(void);

static void sd_handle_error(sd_error_t err, FRESULT res);
static void sd_update_connection_state(void);
static bool sd_new_log_file(void);
static void sd_write_periodic(void);
static bool get_log_enable(void);

static int8_t eth_init(void);
static int8_t eth_get_link_up(void);
static void eth_update_connection_state(void);
static void eth_update_tcp_connection_state(void);
static bool eth_get_tcp_connected(void);
static int8_t eth_init_udp_broadcast(void);
static void eth_send_udp_periodic(void);

static void eth_rx_tcp_periodic(void);
static void eth_tx_tcp_periodic(void);

static void conv_tcp_frame_to_can_msg(timestamped_frame_t *t, CanMsgTypeDef_t *c);
static void conv_can_msg_to_tcp_frame(CanMsgTypeDef_t *c, timestamped_frame_t *t);
static void uds_receive_periodic(void);
static void shutdown(void);

// TODO: use can parse somehow to check for daq enable signal, etc

void daq_init(void)
{
    // Ethernet
    dh.eth_state = ETH_IDLE;
    dh.eth_enable_udp_broadcast = true;
    dh.eth_enable_tcp_reception = true;
    dh.eth_error_ct = 0;
    dh.eth_last_error_time = 0;
    dh.eth_last_err = ETH_ERROR_NONE;
    dh.eth_tcp_state = ETH_TCP_IDLE;
    dh.eth_tcp_last_rx_ms = 0;
    //bActivateTail(&b_rx_tcp, TCP_RX_TAIL_CAN_TX); // start active

    // SD Card
    dh.sd_state = SD_IDLE;
    dh.sd_error_ct = 0;
    dh.sd_last_err = SD_ERROR_NONE;
    dh.sd_last_err_res = 0;
    dh.log_enable_sw = false;
    dh.log_enable_tcp = false;
    dh.log_enable_uds = false;
    dh.my_watch = 0x420;

    // FTP
    ftpd_init(eth_config.net_info.ip);
    dh.ftp_busy = false;

    // General
    dh.loop_time_avg_ms = 0;
    dh.loop_time_max_ms = 0;
}

void daq_loop(void)
{
    uint32_t tic;
    uint32_t last_hb_toggle_ms = 0;

    while (PER == GREAT)
    {
        tic = tick_ms;
        sd_update_connection_state();
        eth_update_connection_state();

        //--------------------------------
        // Message Rx
        //--------------------------------
        sd_write_periodic();
        eth_send_udp_periodic();

        //--------------------------------
        // Message Tx
        //--------------------------------
        eth_rx_tcp_periodic();
        uds_receive_periodic(); // src-> CAN interrupt or TCP RX
        can_relay_can2_periodic();

        // TX
        can_tx_periodic(); // send accumulated can at the end
        eth_tx_tcp_periodic();

        // TODO: flow control, multiple at once?
        // Update timing info
        tic = tick_ms - tic;
        dh.loop_time_max_ms = MAX(tic, dh.loop_time_max_ms);
        dh.loop_time_avg_ms = (tic + dh.loop_time_avg_ms) / 2;
        if (tick_ms - last_hb_toggle_ms >= 500)
        {
            PHAL_toggleGPIO(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
            last_hb_toggle_ms = tick_ms;
        }
    }
}

// drain CAN1 TX queue
static void can_tx_periodic(void)
{
    CanMsgTypeDef_t tx_msg;
    // TODO: we only check if CAN1 mailbox is free -> create separate queue for
    // CAN2 is you are using it!!!!
    for (uint8_t i = 0; i < CAN_TX_MAILBOX_CNT; ++i)
    {
        if (PHAL_txMailboxFree(CAN1, i))
        {
            if (qReceive(&q_tx_can1_s[i], &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
            {
                PHAL_txCANMessage(&tx_msg, i);
                mbx_last_send_time[i] = tick_ms;
            }
        }
        else if (tick_ms - mbx_last_send_time[i] > CAN_TX_TIMEOUT_MS)
        {
            PHAL_txCANAbort(CAN1, i); // aborts tx and empties the mailbox
            can_stats.tx_fail++;
        }
    }
}

static void can_relay_can2_periodic(void)
{
    CanMsgTypeDef_t msg;
    while (qReceive(&q_tx_can2_to_can1, &msg) == SUCCESS_G)
    {
        canTxSendToBack(&msg);
    }
}

bool daq_request_sd_mount(void)
{
    FRESULT res;
    if (dh.sd_state == SD_MOUNTED || dh.sd_state == SD_FILE_CREATED)
    {
        return true;
    }

    if (!SD_Detect())
    {
        sd_handle_error(SD_ERROR_DETEC, 0);
    }
    else
    {
        res = f_mount(&dh.fat_fs, "", 1);
        if (res == FR_OK)
        {
            dh.sd_state = SD_MOUNTED;
            return true;
        }
        else
        {
            sd_handle_error(SD_ERROR_MOUNT, res);
            dh.sd_state = SD_FAIL; // force fail from idle
        }
    }
    return false;
}

static void sd_update_connection_state(void)
{
    FRESULT res;
    static uint32_t last_error_count;
    static uint32_t last_sw_check = 0;
    bool new_sw;

    // Log enable debouncing
    // TMP: switch doesnt work
    #if 0
    if (!last_sw_check || tick_ms - last_sw_check > 100)
    {
        new_sw = PHAL_readGPIO(LOG_ENABLE_PORT, LOG_ENABLE_PIN);
        if (new_sw != dh.log_enable_sw) dh.log_enable_tcp = new_sw; // switch trumps tcp on change
        dh.log_enable_sw = new_sw;
        last_sw_check = tick_ms;
    }
    #endif

    // Check if we should definitely disconnect
    if (dh.sd_state != SD_IDLE)
    {
        if (!SD_Detect()) sd_handle_error(SD_ERROR_DETEC, 0);
        else if (!get_log_enable() && !dh.ftp_busy) dh.sd_state = SD_SHUTDOWN;
    }
    else if (!get_log_enable())
    {
        dh.sd_error_ct = 0;
        dh.sd_last_err = SD_ERROR_NONE;
    }

    if (dh.sd_error_ct - last_error_count > 0) dh.sd_state = SD_FAIL;
    last_error_count = dh.sd_error_ct;

    switch (dh.sd_state)
    {
        case SD_IDLE:
            PHAL_writeGPIO(SD_ACTIVITY_LED_PORT, SD_ACTIVITY_LED_PIN, 0); // Activity LED disable
            if (get_log_enable() && dh.sd_error_ct == 0) // TODO: revert, ensuring no auto-reset for now
            {
                daq_request_sd_mount();
            }
            break;
        case SD_MOUNTED:
            PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, 1);
            if (get_log_enable())
            {
                bActivateTail(&b_rx_can, RX_TAIL_SD);
                if (sd_new_log_file())
                {
                    dh.sd_state = SD_FILE_CREATED;
                    dh.sd_last_err = SD_ERROR_NONE; // back to okay
                }
                else
                {
                    sd_handle_error(SD_ERROR_FOPEN, 0);
                }
            }
           break;
        case SD_FILE_CREATED:
            if ((tick_ms - dh.log_start_ms) > SD_NEW_FILE_PERIOD_MS)
            {
                // Swap to a new log file
                if ((res = f_close(&dh.log_fp)) != FR_OK) sd_handle_error(SD_ERROR_FCLOSE, res);
                else if (!sd_new_log_file()) sd_handle_error(SD_ERROR_FOPEN, 0);
            }
            if (!get_log_enable())
            {
                if ((res = f_close(&dh.log_fp)) != FR_OK) sd_handle_error(SD_ERROR_FCLOSE, res);
                dh.sd_state = SD_MOUNTED;
            }
            break;
        case SD_FAIL:
            // Intentional fall-through
        case SD_SHUTDOWN:
        default:
            f_close(&dh.log_fp);   // Close file
            f_mount(0, "", 1);     // Unmount drive
            SD_DeInit();           // Shutdown SDIO peripheral
            dh.sd_state = SD_IDLE;
            bDeactivateTail(&b_rx_can, RX_TAIL_SD);
            PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, 0);
        break;
    }

    // Blink status LED
    static uint8_t err_blink_tick;
    static uint32_t last_tick_ct;
    if (dh.sd_last_err != SD_ERROR_NONE)
    {
        if (tick_ms - last_tick_ct >= 250)
        {
            last_tick_ct = tick_ms;
            if (err_blink_tick/2 < dh.sd_last_err)
            {
                PHAL_toggleGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN);
                ++err_blink_tick;
            }
            else if (err_blink_tick/2 < dh.sd_last_err + 2)
            {
                ++err_blink_tick;
                PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, 0);
            }
            else
            {
                err_blink_tick = 0;
                PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, 0);
            }
        }
    }
    else
    {
        PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, 0);
    }
}

static bool sd_new_log_file(void)
{
    FRESULT ret;
    RTC_timestamp_t time;
    static uint32_t log_num;

    // Record creation time
    dh.log_start_ms = tick_ms;
    dh.last_write_ms = tick_ms;

    char f_name[30];

    if (PHAL_getTimeRTC(&time))
    {
        // Create file name from RTC
        sprintf(f_name, "log-20%02d-%02d-%02d--%02d-%02d-%02d.log",
                time.date.year_bcd, time.date.month_bcd,
                time.date.day_bcd,  time.time.hours_bcd,
                time.time.minutes_bcd, time.time.seconds_bcd);
    }
    else
    {
        sprintf(f_name, "log-%04d.log", log_num);
    }
    ret = f_open(&dh.log_fp, f_name, FA_OPEN_APPEND | FA_READ | FA_WRITE);
    if (ret == FR_OK) ++log_num;
    return ret == FR_OK;
}

static void sd_write_periodic(void)
{
    timestamped_frame_t *buf;
    uint32_t consecutive_items;
    UINT bytes_written;
    FRESULT res;
    if (dh.sd_state == SD_FILE_CREATED)
    {
        // Use the total item count, not contiguous for the threshold
        if (bGetItemCount(&b_rx_can, RX_TAIL_SD) >= SD_MAX_WRITE_COUNT ||
            (tick_ms - dh.last_write_ms >= SD_MAX_WRITE_PERIOD_MS))
        {
            if (bGetTailForRead(&b_rx_can, RX_TAIL_SD, (void**) &buf, &consecutive_items) == 0)
            {
                if (consecutive_items > SD_MAX_WRITE_COUNT) consecutive_items = SD_MAX_WRITE_COUNT; // limit
                // Write time :D
                if ((res = f_write(&dh.log_fp, buf, consecutive_items * sizeof(*buf), &bytes_written)) != FR_OK)
                {
                    sd_handle_error(SD_ERROR_WRITE, res);
                }
                else
                {
                    bCommitRead(&b_rx_can, RX_TAIL_SD, bytes_written / sizeof(*buf));
                    dh.last_write_ms = tick_ms;
                }
            }
        }
    }
}

static void sd_handle_error(sd_error_t err, FRESULT res)
{
    ++dh.sd_error_ct;
    dh.sd_last_err = err;
    dh.sd_last_err_res = res;
}

static void eth_update_connection_state(void)
{
    static uint32_t last_link_check_time = 0;
    static uint32_t last_blink_time = 0;
    static uint32_t last_init_time = 0;
    static uint32_t init_try_counter = 0;

#if 0
    static uint32_t last_error_count = 0;
    if (dh.eth_error_ct - last_error_count > 0)
    {
        dh.eth_state = ETH_FAIL;
    }
    last_error_count = dh.eth_error_ct;
#endif
    // TODO: on forceful dismount, reset the ports (maybe even ftp_init())

    switch(dh.eth_state)
    {
        case ETH_IDLE:
            if ((dh.eth_enable_tcp_reception || dh.eth_enable_udp_broadcast))
            {
                // Allow for module to power up
                if (!last_init_time)
                    last_init_time = tick_ms;
                else if ((tick_ms - last_init_time) > 500)
                {
                    last_init_time = tick_ms;
                    init_try_counter++;
                    if (eth_init() == ETH_ERROR_NONE)
                    {
                        dh.eth_state = ETH_LINK_DOWN;
                        if (dh.eth_enable_udp_broadcast)
                        {
                            // Chip initialized, setup UDP socket
                            if (eth_init_udp_broadcast() != ETH_ERROR_NONE)
                                dh.eth_state = ETH_FAIL;
                        }
                    }
                    else
                    {
                        dh.eth_state = ETH_FAIL;
                        dh.eth_last_error_time = tick_ms;
                    }
                }
            }
            break;
        case ETH_LINK_DOWN:
            if (!last_link_check_time || tick_ms - last_link_check_time > 100)
            {
                if (eth_get_link_up())
                {
                    dh.eth_state = ETH_LINK_UP;
                    if (dh.eth_enable_udp_broadcast) bActivateTail(&b_rx_can, RX_TAIL_UDP);
                }
                last_link_check_time = tick_ms;
            }
            break;
        case ETH_LINK_UP:
            if (!last_link_check_time || tick_ms - last_link_check_time > 1000)
            {
                if (!eth_get_link_up())
                {
                    dh.eth_state = ETH_LINK_DOWN;
                    if (dh.eth_enable_udp_broadcast) bDeactivateTail(&b_rx_can, RX_TAIL_UDP);
                }
                last_link_check_time = tick_ms;
            }
            ftpd_run(gFTPBUF);
            break;
        case ETH_FAIL:
            // Intentional fall-through
        default:
            // Do not re-attempt (immediately), something is very wrong
            PHAL_writeGPIO(ERROR_LED_PORT, ERROR_LED_PIN, 1);
            if (tick_ms - dh.eth_last_error_time > 500)
            {
                dh.eth_state = ETH_IDLE; // Retry
                last_init_time = tick_ms;
                PHAL_writeGPIO(ERROR_LED_PORT, ERROR_LED_PIN, 0);
            }
            bDeactivateTail(&b_rx_can, RX_TAIL_UDP);
            break;
    }

    eth_update_tcp_connection_state();

    // Update connection LED
    if (dh.eth_state == ETH_LINK_UP)
    {
        if (dh.eth_tcp_state == ETH_TCP_ESTABLISHED || dh.ftp_busy)
        {
            if (!last_blink_time || tick_ms - last_blink_time > 250)
            {
                last_blink_time = tick_ms;
                PHAL_toggleGPIO(CONNECTION_LED_PORT, CONNECTION_LED_PIN);
            }
        }
        else
        {
            PHAL_writeGPIO(CONNECTION_LED_PORT, CONNECTION_LED_PIN, 1);
        }
    }
    else
    {
        PHAL_writeGPIO(CONNECTION_LED_PORT, CONNECTION_LED_PIN, 0);
    }
}

static void eth_handle_error(eth_error_t err)
{
    ++dh.eth_error_ct;
    dh.eth_last_err = err;
    dh.eth_last_error_time = tick_ms;
}

static int8_t eth_init(void)
{
    uint32_t timeout_ms;
    timeout_ms = tick_ms;
    PHAL_writeGPIO(ETH_RST_PORT, ETH_RST_PIN, 0);
    while (tick_ms - timeout_ms < ETH_PHY_RESET_PERIOD_MS);
    PHAL_writeGPIO(ETH_RST_PORT, ETH_RST_PIN, 1);
    timeout_ms = tick_ms;
    while (tick_ms - timeout_ms < ETH_PHY_RESET_PERIOD_MS);

    // Check for sign of life
    if (getVERSIONR() != 0x04)
    {
        eth_handle_error(ETH_ERROR_VERS);
        return ETH_ERROR_VERS;
    }

    uint8_t rxBufSizes[] = {2, 2, 2, 2, 2, 2, 2, 2}; // kB
    uint8_t txBufSizes[] = {2, 2, 2, 8, 2, 0, 0, 0}; // kB

    if(wizchip_init(txBufSizes, rxBufSizes))
    {
        eth_handle_error(ETH_ERROR_INIT);
        return ETH_ERROR_INIT;
    }

    wizchip_setnetinfo(&eth_config.net_info);

    return ETH_ERROR_NONE;
}

static int8_t eth_get_link_up(void)
{
    volatile uint8_t phycfgr = getPHYCFGR();
    return phycfgr & PHY_LINK_ON;
}

static int8_t eth_init_udp_broadcast(void)
{
    // UDP Broadcast address
    // 255.255.255.255 is not forwarded by routers
    // local address | (~subnet address) is forwarded

    int8_t bc_sock;
    bc_sock = socket(eth_config.udp_bc_sock, SOCK_DGRAM, eth_config.udp_bc_port, 0);
    if (bc_sock != eth_config.udp_bc_sock)
    {
        eth_handle_error(ETH_ERROR_UDP_SOCK);
        return -ETH_ERROR_UDP_SOCK;
    }
    return ETH_ERROR_NONE;
}

static void eth_send_udp_periodic(void)
{
    int32_t ret;
    timestamped_frame_t *buf;
    uint32_t consecutive_items;
    static uint32_t last_send_ms;

    // Should be sending?
    if (!dh.eth_enable_udp_broadcast ||
        dh.eth_state != ETH_LINK_UP) return;

    if (bGetItemCount(&b_rx_can, RX_TAIL_UDP) >= UDP_MAX_WRITE_COUNT ||
        (tick_ms - last_send_ms) >= UDP_MAX_WRITE_PERIOD_MS)
    {
        if (bGetTailForRead(&b_rx_can, RX_TAIL_UDP, (void**) &buf, &consecutive_items) == 0)
        {
            if (consecutive_items > UDP_MAX_WRITE_COUNT) consecutive_items = UDP_MAX_WRITE_COUNT; // limit
            // Write time :D
            ret = sendto(eth_config.udp_bc_sock, (uint8_t *)buf,
                         consecutive_items * sizeof(*buf),
                         eth_config.udp_bc_addr,
                         eth_config.udp_bc_port);
            if (ret != consecutive_items * sizeof(*buf))
            {
                eth_handle_error(ETH_ERROR_UDP_SEND);
            }
            else
            {
                bCommitRead(&b_rx_can, RX_TAIL_UDP, consecutive_items);
                last_send_ms = tick_ms;
            }
        }
    }
}

static int8_t eth_init_tcp(void)
{
    // Open socket
    int8_t tcp_sock;
    tcp_sock = socket(eth_config.tcp_sock, SOCK_STREAM, eth_config.tcp_port, 0);
    if (tcp_sock != eth_config.tcp_sock)
    {
        eth_handle_error(ETH_ERROR_TCP_SOCK);
        return -ETH_ERROR_TCP_SOCK;
    }
    // Set to non-blocking
    uint8_t io_mode = SOCK_IO_NONBLOCK;
    ctlsocket(eth_config.tcp_sock, CS_SET_IOMODE, &io_mode);
    // Set socket to listen -> verify by checking SR went to ESTABLISHED
    if (listen(eth_config.tcp_sock) != SOCK_OK)
    {
        eth_handle_error(ETH_ERROR_TCP_LISTEN);
        return -ETH_ERROR_TCP_LISTEN;
    }

    dh.eth_tcp_state = ETH_TCP_LISTEN;

    return ETH_ERROR_NONE;
}

static bool eth_get_tcp_connected(void)
{
    uint8_t stat;
    if (dh.eth_tcp_state == ETH_IDLE) return false;
    getsockopt(eth_config.tcp_sock, SO_STATUS, &stat);
    if (stat == SOCK_ESTABLISHED)
    {
        dh.eth_tcp_state = ETH_TCP_ESTABLISHED;
        return true;
    }
    else if (stat == SOCK_LISTEN)
    {
        dh.eth_tcp_state = ETH_TCP_LISTEN;
    }
    else if (stat == SOCK_CLOSED)
        dh.eth_tcp_state = ETH_IDLE;
    return false;
}

static void eth_update_tcp_connection_state(void)
{
    static uint32_t last_link_check_time = 0;

    int32_t ret;
    // When to definitely restart
    if (dh.eth_state != ETH_LINK_UP)
        dh.eth_tcp_state = ETH_TCP_IDLE;

    switch(dh.eth_tcp_state)
    {
        case ETH_TCP_IDLE:
            if (dh.eth_state == ETH_LINK_UP && dh.eth_enable_tcp_reception)
            {
                if (eth_init_tcp() != ETH_ERROR_NONE)
                {
                    dh.eth_tcp_state = ETH_TCP_FAIL;
                }
                else
                {
                    dh.eth_tcp_state = ETH_TCP_LISTEN;
                }
            }
            break;
        case ETH_TCP_LISTEN:
            if (!last_link_check_time || tick_ms - last_link_check_time > 500)
            {
                if (eth_get_tcp_connected())
                {
                    // Reset buffer state
                    // b_rx_tcp._head = 0;
                    bActivateTail(&b_rx_tcp, TCP_RX_TAIL_CAN_TX);
                    dh.eth_tcp_state = ETH_TCP_ESTABLISHED;
                }
                last_link_check_time = tick_ms;
            }
            break;
        case ETH_TCP_ESTABLISHED:
            if (!last_link_check_time || tick_ms - last_link_check_time > 500)
            {
                if (!eth_get_tcp_connected())
                {
                    bDeactivateTail(&b_rx_tcp, TCP_RX_TAIL_CAN_TX);
                    dh.eth_tcp_state = ETH_TCP_LISTEN;
                }
                last_link_check_time = tick_ms;
            }
            break;
        case ETH_TCP_FAIL:
            // fall-through
        default:
            // staying here for now
            break;
    }
}

static void eth_read_tcp_periodic(void);

#if 0
static void eth_send_tcp_periodic(void)
{
    int32_t ret;
    timestamped_frame_t *buf;
    uint32_t consecutive_items;
    static uint32_t last_send_ms;

    // Should be sending?
    if (!dh.eth_enable_udp_broadcast ||
        dh.eth_state != ETH_LINK_UP) return;

    if (bGetItemCount(&b_rx_can, TCP_RX_TAIL_CAN_TX) >= TCP_MAX_WRITE_COUNT ||
        (tick_ms - last_send_ms) >= UDP_MAX_WRITE_PERIOD_MS)
    {
        if (bGetTailForRead(&b_rx_can, TCP_RX_TAIL_CAN_TX, (void**) &buf, &consecutive_items) == 0)
        {
            if (consecutive_items > TCP_MAX_WRITE_COUNT) consecutive_items = TCP_MAX_WRITE_COUNT; // limit
            // Write time :D
            ret = sendto(eth_config.udp_bc_sock, (uint8_t *)buf,
                         consecutive_items * sizeof(*buf),
                         eth_config.udp_bc_addr,
                         eth_config.udp_bc_port);
            if (ret != consecutive_items * sizeof(*buf))
            {
                eth_handle_error(ETH_ERROR_UDP_SEND);
            }
            else
            {
                bCommitRead(&b_rx_can, TCP_RX_TAIL_CAN_TX, consecutive_items);
                last_send_ms = tick_ms;
            }
        }
    }
}
#endif

static void eth_rx_tcp_periodic(void)
{
    int32_t ret;
    timestamped_frame_t *frame;
    uint32_t cont;
    // When to definitely restart
    if (dh.eth_state != ETH_LINK_UP)
        dh.eth_tcp_state = ETH_TCP_IDLE;

    switch(dh.eth_tcp_state)
    {
        case ETH_TCP_IDLE:
            if (dh.eth_state == ETH_LINK_UP && dh.eth_enable_tcp_reception)
            {
                if (eth_init_tcp() != ETH_ERROR_NONE)
                {
                    dh.eth_tcp_state = ETH_TCP_FAIL;
                }
                else
                {
                    dh.eth_tcp_state = ETH_TCP_LISTEN;
                }
            }
            break;
        case ETH_TCP_LISTEN:
            if (eth_get_tcp_connected())
            {
                // Reset buffer state
                b_rx_tcp._head = 0;
                bActivateTail(&b_rx_tcp, TCP_RX_TAIL_CAN_TX);
                dh.eth_tcp_state = ETH_TCP_ESTABLISHED;
            }
            break;
        case ETH_TCP_ESTABLISHED:
            // Control rate of reception
            #if 1
            if (!dh.eth_tcp_last_rx_ms || tick_ms - dh.eth_tcp_last_rx_ms >= TCP_MIN_RX_PERIOD_MS)
            {
                dh.eth_tcp_last_rx_ms = tick_ms;
                bGetHeadForWrite(&b_rx_tcp, (void**) &frame, &cont);
                if ((cont / sizeof(*frame)) > 0)
                {
                    cont /= sizeof(*frame); // convert bytes -> frame count
                    if (cont > TCP_MAX_WRITE_COUNT) cont = TCP_MAX_WRITE_COUNT;
                    ret = recv(eth_config.tcp_sock, (uint8_t*)frame, cont * sizeof(*frame));
                    if (ret > 0)
                    {
                        // hooray
                        bCommitWrite(&b_rx_tcp, ret);
                    }
                    else if (ret == SOCK_BUSY)
                    {
                        // that's fine, check again later
                    }
                    else
                    {
                        // oof, assuming connection was killed
                        bDeactivateTail(&b_rx_tcp, TCP_RX_TAIL_CAN_TX);
                        dh.eth_tcp_state = ETH_TCP_IDLE;
                    }
                }
            }
            #endif
            break;
        case ETH_TCP_FAIL:
            // fall-through
        default:
            // staying here for now
            break;
    }

    eth_read_tcp_periodic();
}

static void conv_tcp_frame_to_can_msg(timestamped_frame_t *t, CanMsgTypeDef_t *c)
{
    c->Bus = t->bus_id == BUS_ID_CAN1 ? CAN1 : CAN2;

    if (t->msg_id & CAN_EFF_FLAG)
    {
        c->IDE = 1;
        c->ExtId = t->msg_id & CAN_EFF_MASK;
    }
    else
    {
        c->IDE = 0;
        c->StdId = t->msg_id & CAN_SFF_MASK;
    }

    c->DLC = t->dlc;
    for (uint8_t i = 0; i < 8; ++i) c->Data[i] = t->data[i];
}

static void conv_can_msg_to_tcp_frame(CanMsgTypeDef_t *c, timestamped_frame_t *t)
{
    t->cmd = MSG_TCP_TX_FRAME;
    t->tick_ms = tick_ms;
    t->bus_id = c->Bus == CAN1 ? BUS_ID_CAN1 : BUS_ID_CAN2;
    if (c->IDE)
    {
        t->msg_id = c->ExtId & ~CAN_EFF_MASK; // TODO is this right?
    }
    else
    {
        t->msg_id = c->StdId & ~CAN_SFF_MASK; // TODO
    }
    t->dlc = c->DLC;
    for (uint8_t i = 0; i < 8; ++i) t->data[i] = c->Data[i];
}

static void eth_tcp_send(uint8_t *buf, uint16_t size)
{
    int32_t ret;
    if (eth_get_tcp_connected())
    {
        ret = send(eth_config.tcp_sock, buf, size);
    }
}

// send CAN frame over TCP
static void eth_tcp_send_can_frame(CanMsgTypeDef_t *c)
{
    timestamped_frame_t frame;
    conv_can_msg_to_tcp_frame(c, &frame);
    eth_tcp_send((uint8_t *)&frame, sizeof(frame));
}

// Pull out of TCP queue and add it to CAN TX queue
static void eth_tcp_relay_can_frame(timestamped_frame_t *t)
{
    CanMsgTypeDef_t msg;
    conv_tcp_frame_to_can_msg(t, &msg);
    canTxSendToBack(&msg);
}

// Pull out of TCP queue and add it to UDS queue
static void eth_tcp_relay_uds_frame(timestamped_frame_t *t)
{
    t->cmd |= MSG_TCP_RX_FRAME; // Mask to store source of message
    qSendToBack(&q_rx_can_uds, t);
}

/**
 * Pull frames out of TCP RX queue (from PC) and process them by the prefixed command ID
 * If TCP_CMD_CAN_FRAME, put it on CAN for other nodes
 * If TCP_CMD_UDS_FRAME, it's a CAN message intended for DAQ (i.e. UDS) so add it to UDS queue
 */
static void eth_read_tcp_periodic(void)
{
    timestamped_frame_t *rx_msg_a;
    uint32_t cont;

    // Pull frame and send from tcp rx buffer
    if (bGetTailForRead(&b_rx_tcp, TCP_RX_TAIL_CAN_TX, (void**) &rx_msg_a, &cont) == 0)
    {
        if ((cont / sizeof(*rx_msg_a)) > 0)
        {
            // NOTE: if the buffer size is not multiple of frame size, or the tail gets shifted
            // by a frame, reception of frames will not work on the wrap-around.
            cont = MIN(cont / sizeof(*rx_msg_a), TCP_MAX_CAN_TX_COUNT); // flow control
            //cont = MIN(cont, TCP_MAX_CAN_TX_COUNT);
            for (uint32_t i = 0; i < cont; ++i)
            {
                switch(rx_msg_a->cmd)
                {
                    case TCP_CMD_HANDSHAKE:
                        PHAL_toggleGPIO(ERROR_LED_PORT, ERROR_LED_PIN); // TODO send something back?
                        break;
                    case TCP_CMD_CAN_FRAME:
                        eth_tcp_relay_can_frame(rx_msg_a);
                        break;
                    case TCP_CMD_UDS_FRAME:
                        eth_tcp_relay_uds_frame(rx_msg_a);
                        break;
                    default:
                        break;
                }
                ++rx_msg_a;
            }
            bCommitRead(&b_rx_tcp, TCP_RX_TAIL_CAN_TX, cont*sizeof(*rx_msg_a));
        }
    }
}

/**
 * @brief Returns if logging has been enabled.
 *        Sources for enabling are on-board switch
 *        and CAN message request from either
 *        dashboard or via wireless request.
 *        Logging will not be enabled unless
 *        the switch is flipped to on.
 *
 * @return Enabled
 */
static bool get_log_enable(void)
{
    // TODO: combine with CAN message from dash
    // return dh.log_enable_sw || dh.log_enable_tcp;
    // TMP: switch doesnt work
    return dh.log_enable_uds;
}

#define DAQ_BL_CMD_HANDSHAKE  0x00
#define DAQ_BL_CMD_RST        0x05

#define DAQ_BL_CMD_NTP_DATE   0x10
#define DAQ_BL_CMD_NTP_TIME   0x11
#define DAQ_BL_CMD_NTP_GET    0x12

#define DAQ_BL_CMD_LOG_ENABLE 0x20
#define DAQ_BL_CMD_LOG_STATUS 0x21

#define DAQ_BL_CMD_LED_DISCO  0x30

static RTC_timestamp_t start_time =
{
    .date = {.month_bcd=RTC_MONTH_FEBRUARY,
             .weekday=RTC_WEEKDAY_TUESDAY,
             .day_bcd=0x27,
             .year_bcd=0x24},
    .time = {.hours_bcd=0x18,
             .minutes_bcd=0x27,
             .seconds_bcd=0x00,
             .time_format=RTC_FORMAT_24_HOUR},
};

static void uds_process_cmd_ntp_get(void)
{
    RTC_timestamp_t time;
    if (PHAL_getTimeRTC(&time))
        debug_printf("DAQ time: 20%02d-%02d-%02d %02d:%02d:%02d\n",
            time.date.year_bcd, time.date.month_bcd,
            time.date.day_bcd,  time.time.hours_bcd,
            time.time.minutes_bcd, time.time.seconds_bcd);
}

#if 0
// send UDS response
// TODO grab source from rx_msg and determine TCP vs CAN
static void daq_uds_send_frame(uint8_t cmd, uint32_t data)
{
    CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_DAQ_RESPONSE_MAIN_MODULE, .DLC=DLC_DAQ_RESPONSE_MAIN_MODULE, .IDE=1};
    CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;
    data_a->daq_response_MAIN_MODULE.daq_response = daq_response_;
    canTxSendToBack(&msg);
    eth_tcp_send_can_frame(&msg);
}
#endif

static void eth_tx_tcp_periodic(void)
{
    ; // TODO pull out of TCP TX queue
}

/**
 * Process DAQ-specific UDS commands
 */
static void daq_uds_handle_cmd(uint8_t cmd, uint32_t data)
{
    switch (cmd)
    {
        case DAQ_BL_CMD_HANDSHAKE:
            PHAL_toggleGPIO(ERROR_LED_PORT, ERROR_LED_PIN);
            break;
        case DAQ_BL_CMD_RST:
            shutdown(); // NVIC reset / bootloader if loaded
            break;

        case DAQ_BL_CMD_NTP_DATE: // send date first
            start_time.date.day_bcd = data & 0xff;
            start_time.date.weekday = (data >> 8) & 0xf;
            start_time.date.month_bcd = (data >> 12) & 0xff;
            start_time.date.year_bcd = (data >> 20) & 0xff;
            break;
        case DAQ_BL_CMD_NTP_TIME: // then time
            start_time.time.seconds_bcd = data & 0xff;
            start_time.time.minutes_bcd = (data >> 8) & 0xff;
            start_time.time.hours_bcd = (data >> 16) & 0xff;
            PHAL_configureRTC(&start_time, true); // now sync
            break;
        case DAQ_BL_CMD_NTP_GET:
            uds_process_cmd_ntp_get();
            break;

        case DAQ_BL_CMD_LOG_ENABLE:
            dh.log_enable_uds = !!data;
            break;
        case DAQ_BL_CMD_LOG_STATUS:
            if (get_log_enable())
                debug_printf("Logging enabled\n");
            else
                debug_printf("Logging disabled\n");
            break;

        case DAQ_BL_CMD_LED_DISCO:
            //PHAL_writeGPIO(GPIO1_PORT, GPIO1_PIN, (frame.data >> 0) & 1);
            //PHAL_writeGPIO(GPIO2_PORT, GPIO2_PIN, (frame.data >> 1) & 1);
            PHAL_writeGPIO(ERROR_LED_PORT, ERROR_LED_PIN, (data >> 2) & 1);
            PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, (data >> 3) & 1);
            PHAL_writeGPIO(SD_ACTIVITY_LED_PORT, SD_ACTIVITY_LED_PIN, (data >> 4) & 1);
            PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, (data >> 5) & 1);
            break;
    }
}

/**
 * Pull UDS CAN frames out of UDS queue added during CAN1/CAN2 ISR
 * and process them in non-interrupt context
 */
static void uds_receive_periodic(void)
{
    timestamped_frame_t rx_msg;
    if (qReceive(&q_rx_can_uds, &rx_msg) == SUCCESS_G)
    {
        uint32_t data = rx_msg.data[4] << 24 | rx_msg.data[3] << 16 | rx_msg.data[2] << 8 | rx_msg.data[1];
        daq_uds_handle_cmd(rx_msg.data[0], data);
    }
}

/**
 * @brief Disables high power consumption devices
 *        If file open, flushes it to the sd card
 *        Then unmounts sd card
 */
static void shutdown(void)
{
    // First, turn off all power consuming devices to increase our write time to sd card
    GPIOD->BSRR |= (1 << ((1 << 4) | HEARTBEAT_LED_PIN)) | (1 << ((1 << 4) | CONNECTION_LED_PIN)) | (1 << ((1 << 4) | ERROR_LED_PIN)); // all LEDs go bye bye
    GPIOA->BSRR |= (1 << ((1 << 4) | SD_ACTIVITY_LED_PIN)) | (1 << ((1 << 4) | SD_ERROR_LED_PIN)) | (1 << ((1 << 4) | SD_DETECT_LED_PIN));

    PHAL_writeGPIO(ETH_RST_PORT, ETH_RST_PIN, 0);
    PHAL_deinitCAN(CAN1);
    PHAL_deinitCAN(CAN2);

    f_close(&dh.log_fp);   // Close file
    f_mount(0, "", 1);     // Unmount drive
    SD_DeInit();           // Shutdown SDIO peripheral

    // Hooray, we made it, blink an LED to show the world
    PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, 1);
    uint32_t start_tick = tick_ms;
    while (tick_ms - start_tick < 3000 || PHAL_readGPIO(PWR_LOSS_PORT, PWR_LOSS_PIN) == 0) // wait for power to fully turn off -> if it does not, restart
    {
        //if (tick_ms % 250 == 0) PHAL_toggleGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN);
    }
    NVIC_SystemReset(); // oof, we assumed wrong, restart and resume execution since the power is still on!
}

// Interrupt handler for power loss detection
// Note: this is set to lowest priority to allow preemption by other interrupts
void EXTI15_10_IRQHandler()
{
    if (EXTI->PR & EXTI_PR_PR15)
    {
        EXTI->PR |= EXTI_PR_PR15; // Clear interrupt
        shutdown();
    }
}
