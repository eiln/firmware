/**
 * @file can_parse.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief Parsing of CAN messages using auto-generated structures with bit-fields
 * @version 0.1
 * @date 2021-09-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef _CAN_PARSE_H_
#define _CAN_PARSE_H_

#include "common/queue/queue.h"
#include "common/psched/psched.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/daq/can_parse_base.h"

// Make this match the node name within the can_config.json
#define NODE_NAME "adbms"

// Message ID definitions
/* BEGIN AUTO ID DEFS */
#define ID_ADBMS_PRECHARGE_HB 0x9600000
#define ID_ADBMS_ELCON_CHARGER_COMMAND 0x1806e5f5
#define ID_ADBMS_NUM_THERM_BAD 0x9600400
#define ID_ADBMS_PACK_CHARGE_STATUS 0x9600600
#define ID_ADBMS_MAX_CELL_TEMP 0x9600800
#define ID_ADBMS_MOD_CELL_TEMP_AVG_A_B_C 0x9600a00
#define ID_ADBMS_MOD_CELL_TEMP_AVG_D_E 0x9600c00
#define ID_ADBMS_MOD_CELL_TEMP_MAX_A_B_C 0x9600e00
#define ID_ADBMS_MOD_CELL_TEMP_MAX_D_E 0x9601000
#define ID_ADBMS_MOD_CELL_TEMP_MIN_A_B_C 0x9601200
#define ID_ADBMS_MOD_CELL_TEMP_MIN_D_E 0x9601400
#define ID_ADBMS_RAW_CELL_TEMP_MODULE1 0x9601600
#define ID_ADBMS_RAW_CELL_TEMP_MODULE2 0x9601800
#define ID_ADBMS_RAW_CELL_TEMP_MODULE3 0x9601a00
#define ID_ADBMS_RAW_CELL_TEMP_MODULE4 0x9601c00
#define ID_ADBMS_RAW_CELL_TEMP_MODULE5 0x9601e00
#define ID_ADBMS_CAN_STATS 0x9602000
#define ID_ADBMS_I_SENSE 0x9602200
#define ID_A_BOX_BL_CMD 0x8000a00
/* END AUTO ID DEFS */

// Message DLC definitions
/* BEGIN AUTO DLC DEFS */
#define DLC_ADBMS_PRECHARGE_HB 2
#define DLC_ADBMS_ELCON_CHARGER_COMMAND 5
#define DLC_ADBMS_NUM_THERM_BAD 5
#define DLC_ADBMS_PACK_CHARGE_STATUS 7
#define DLC_ADBMS_MAX_CELL_TEMP 2
#define DLC_ADBMS_MOD_CELL_TEMP_AVG_A_B_C 6
#define DLC_ADBMS_MOD_CELL_TEMP_AVG_D_E 4
#define DLC_ADBMS_MOD_CELL_TEMP_MAX_A_B_C 6
#define DLC_ADBMS_MOD_CELL_TEMP_MAX_D_E 4
#define DLC_ADBMS_MOD_CELL_TEMP_MIN_A_B_C 6
#define DLC_ADBMS_MOD_CELL_TEMP_MIN_D_E 4
#define DLC_ADBMS_RAW_CELL_TEMP_MODULE1 5
#define DLC_ADBMS_RAW_CELL_TEMP_MODULE2 5
#define DLC_ADBMS_RAW_CELL_TEMP_MODULE3 5
#define DLC_ADBMS_RAW_CELL_TEMP_MODULE4 5
#define DLC_ADBMS_RAW_CELL_TEMP_MODULE5 5
#define DLC_ADBMS_CAN_STATS 7
#define DLC_ADBMS_I_SENSE 4
#define DLC_A_BOX_BL_CMD 5
/* END AUTO DLC DEFS */

// Message sending macros
/* BEGIN AUTO SEND MACROS */
#define SEND_ADBMS_PRECHARGE_HB(IMD_, BMS_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_PRECHARGE_HB, .DLC=DLC_ADBMS_PRECHARGE_HB, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_precharge_hb.IMD = IMD_;\
        data_a->adbms_precharge_hb.BMS = BMS_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_ELCON_CHARGER_COMMAND(voltage_limit_, current_limit_, charge_disable_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_ELCON_CHARGER_COMMAND, .DLC=DLC_ADBMS_ELCON_CHARGER_COMMAND, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_elcon_charger_command.voltage_limit = voltage_limit_;\
        data_a->adbms_elcon_charger_command.current_limit = current_limit_;\
        data_a->adbms_elcon_charger_command.charge_disable = charge_disable_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_NUM_THERM_BAD(A_left_, A_right_, B_left_, B_right_, C_left_, C_right_, D_left_, D_right_, E_left_, E_right_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_NUM_THERM_BAD, .DLC=DLC_ADBMS_NUM_THERM_BAD, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_num_therm_bad.A_left = A_left_;\
        data_a->adbms_num_therm_bad.A_right = A_right_;\
        data_a->adbms_num_therm_bad.B_left = B_left_;\
        data_a->adbms_num_therm_bad.B_right = B_right_;\
        data_a->adbms_num_therm_bad.C_left = C_left_;\
        data_a->adbms_num_therm_bad.C_right = C_right_;\
        data_a->adbms_num_therm_bad.D_left = D_left_;\
        data_a->adbms_num_therm_bad.D_right = D_right_;\
        data_a->adbms_num_therm_bad.E_left = E_left_;\
        data_a->adbms_num_therm_bad.E_right = E_right_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_PACK_CHARGE_STATUS(power_, charge_enable_, voltage_, current_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_PACK_CHARGE_STATUS, .DLC=DLC_ADBMS_PACK_CHARGE_STATUS, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_pack_charge_status.power = power_;\
        data_a->adbms_pack_charge_status.charge_enable = charge_enable_;\
        data_a->adbms_pack_charge_status.voltage = voltage_;\
        data_a->adbms_pack_charge_status.current = current_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_MAX_CELL_TEMP(max_temp_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_MAX_CELL_TEMP, .DLC=DLC_ADBMS_MAX_CELL_TEMP, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_max_cell_temp.max_temp = max_temp_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_MOD_CELL_TEMP_AVG_A_B_C(temp_A_, temp_B_, temp_C_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_MOD_CELL_TEMP_AVG_A_B_C, .DLC=DLC_ADBMS_MOD_CELL_TEMP_AVG_A_B_C, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_mod_cell_temp_avg_a_b_c.temp_A = temp_A_;\
        data_a->adbms_mod_cell_temp_avg_a_b_c.temp_B = temp_B_;\
        data_a->adbms_mod_cell_temp_avg_a_b_c.temp_C = temp_C_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_MOD_CELL_TEMP_AVG_D_E(temp_D_, temp_E_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_MOD_CELL_TEMP_AVG_D_E, .DLC=DLC_ADBMS_MOD_CELL_TEMP_AVG_D_E, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_mod_cell_temp_avg_d_e.temp_D = temp_D_;\
        data_a->adbms_mod_cell_temp_avg_d_e.temp_E = temp_E_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_MOD_CELL_TEMP_MAX_A_B_C(temp_A_, temp_B_, temp_C_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_MOD_CELL_TEMP_MAX_A_B_C, .DLC=DLC_ADBMS_MOD_CELL_TEMP_MAX_A_B_C, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_mod_cell_temp_max_a_b_c.temp_A = temp_A_;\
        data_a->adbms_mod_cell_temp_max_a_b_c.temp_B = temp_B_;\
        data_a->adbms_mod_cell_temp_max_a_b_c.temp_C = temp_C_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_MOD_CELL_TEMP_MAX_D_E(temp_D_, temp_E_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_MOD_CELL_TEMP_MAX_D_E, .DLC=DLC_ADBMS_MOD_CELL_TEMP_MAX_D_E, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_mod_cell_temp_max_d_e.temp_D = temp_D_;\
        data_a->adbms_mod_cell_temp_max_d_e.temp_E = temp_E_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_MOD_CELL_TEMP_MIN_A_B_C(temp_A_, temp_B_, temp_C_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_MOD_CELL_TEMP_MIN_A_B_C, .DLC=DLC_ADBMS_MOD_CELL_TEMP_MIN_A_B_C, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_mod_cell_temp_min_a_b_c.temp_A = temp_A_;\
        data_a->adbms_mod_cell_temp_min_a_b_c.temp_B = temp_B_;\
        data_a->adbms_mod_cell_temp_min_a_b_c.temp_C = temp_C_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_MOD_CELL_TEMP_MIN_D_E(temp_D_, temp_E_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_MOD_CELL_TEMP_MIN_D_E, .DLC=DLC_ADBMS_MOD_CELL_TEMP_MIN_D_E, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_mod_cell_temp_min_d_e.temp_D = temp_D_;\
        data_a->adbms_mod_cell_temp_min_d_e.temp_E = temp_E_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_RAW_CELL_TEMP_MODULE1(index_, temp_left_, temp_right_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_RAW_CELL_TEMP_MODULE1, .DLC=DLC_ADBMS_RAW_CELL_TEMP_MODULE1, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_raw_cell_temp_module1.index = index_;\
        data_a->adbms_raw_cell_temp_module1.temp_left = temp_left_;\
        data_a->adbms_raw_cell_temp_module1.temp_right = temp_right_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_RAW_CELL_TEMP_MODULE2(index_, temp_left_, temp_right_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_RAW_CELL_TEMP_MODULE2, .DLC=DLC_ADBMS_RAW_CELL_TEMP_MODULE2, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_raw_cell_temp_module2.index = index_;\
        data_a->adbms_raw_cell_temp_module2.temp_left = temp_left_;\
        data_a->adbms_raw_cell_temp_module2.temp_right = temp_right_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_RAW_CELL_TEMP_MODULE3(index_, temp_left_, temp_right_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_RAW_CELL_TEMP_MODULE3, .DLC=DLC_ADBMS_RAW_CELL_TEMP_MODULE3, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_raw_cell_temp_module3.index = index_;\
        data_a->adbms_raw_cell_temp_module3.temp_left = temp_left_;\
        data_a->adbms_raw_cell_temp_module3.temp_right = temp_right_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_RAW_CELL_TEMP_MODULE4(index_, temp_left_, temp_right_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_RAW_CELL_TEMP_MODULE4, .DLC=DLC_ADBMS_RAW_CELL_TEMP_MODULE4, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_raw_cell_temp_module4.index = index_;\
        data_a->adbms_raw_cell_temp_module4.temp_left = temp_left_;\
        data_a->adbms_raw_cell_temp_module4.temp_right = temp_right_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_RAW_CELL_TEMP_MODULE5(index_, temp_left_, temp_right_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_RAW_CELL_TEMP_MODULE5, .DLC=DLC_ADBMS_RAW_CELL_TEMP_MODULE5, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_raw_cell_temp_module5.index = index_;\
        data_a->adbms_raw_cell_temp_module5.temp_left = temp_left_;\
        data_a->adbms_raw_cell_temp_module5.temp_right = temp_right_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_CAN_STATS(can1_tx_queue_overflow_, can2_tx_queue_overflow_, can1_tx_fail_, can2_tx_fail_, can_rx_queue_overflow_, can1_rx_overrun_, can2_rx_overrun_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_CAN_STATS, .DLC=DLC_ADBMS_CAN_STATS, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_can_stats.can1_tx_queue_overflow = can1_tx_queue_overflow_;\
        data_a->adbms_can_stats.can2_tx_queue_overflow = can2_tx_queue_overflow_;\
        data_a->adbms_can_stats.can1_tx_fail = can1_tx_fail_;\
        data_a->adbms_can_stats.can2_tx_fail = can2_tx_fail_;\
        data_a->adbms_can_stats.can_rx_queue_overflow = can_rx_queue_overflow_;\
        data_a->adbms_can_stats.can1_rx_overrun = can1_rx_overrun_;\
        data_a->adbms_can_stats.can2_rx_overrun = can2_rx_overrun_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_ADBMS_I_SENSE(current_channel_1_, current_channel_2_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_ADBMS_I_SENSE, .DLC=DLC_ADBMS_I_SENSE, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->adbms_i_sense.current_channel_1 = current_channel_1_;\
        data_a->adbms_i_sense.current_channel_2 = current_channel_2_;\
        canTxSendToBack(&msg);\
    } while(0)
/* END AUTO SEND MACROS */

// Stale Checking
#define STALE_THRESH 5 / 2 // 5 / 2 would be 250% of period
/* BEGIN AUTO UP DEFS (Update Period)*/
/* END AUTO UP DEFS */

#define CHECK_STALE(stale, curr, last, period) if(!stale && \
                    (curr - last) > period * STALE_THRESH) stale = 1

/* BEGIN AUTO CAN ENUMERATIONS */
/* END AUTO CAN ENUMERATIONS */

// Message Raw Structures
/* BEGIN AUTO MESSAGE STRUCTURE */
typedef union { 
    struct {
        uint64_t IMD: 8;
        uint64_t BMS: 8;
    } adbms_precharge_hb;
    struct {
        uint64_t voltage_limit: 16;
        uint64_t current_limit: 16;
        uint64_t charge_disable: 1;
    } adbms_elcon_charger_command;
    struct {
        uint64_t A_left: 4;
        uint64_t A_right: 4;
        uint64_t B_left: 4;
        uint64_t B_right: 4;
        uint64_t C_left: 4;
        uint64_t C_right: 4;
        uint64_t D_left: 4;
        uint64_t D_right: 4;
        uint64_t E_left: 4;
        uint64_t E_right: 4;
    } adbms_num_therm_bad;
    struct {
        uint64_t power: 16;
        uint64_t charge_enable: 1;
        uint64_t voltage: 16;
        uint64_t current: 16;
    } adbms_pack_charge_status;
    struct {
        uint64_t max_temp: 16;
    } adbms_max_cell_temp;
    struct {
        uint64_t temp_A: 16;
        uint64_t temp_B: 16;
        uint64_t temp_C: 16;
    } adbms_mod_cell_temp_avg_a_b_c;
    struct {
        uint64_t temp_D: 16;
        uint64_t temp_E: 16;
    } adbms_mod_cell_temp_avg_d_e;
    struct {
        uint64_t temp_A: 16;
        uint64_t temp_B: 16;
        uint64_t temp_C: 16;
    } adbms_mod_cell_temp_max_a_b_c;
    struct {
        uint64_t temp_D: 16;
        uint64_t temp_E: 16;
    } adbms_mod_cell_temp_max_d_e;
    struct {
        uint64_t temp_A: 16;
        uint64_t temp_B: 16;
        uint64_t temp_C: 16;
    } adbms_mod_cell_temp_min_a_b_c;
    struct {
        uint64_t temp_D: 16;
        uint64_t temp_E: 16;
    } adbms_mod_cell_temp_min_d_e;
    struct {
        uint64_t index: 8;
        uint64_t temp_left: 16;
        uint64_t temp_right: 16;
    } adbms_raw_cell_temp_module1;
    struct {
        uint64_t index: 8;
        uint64_t temp_left: 16;
        uint64_t temp_right: 16;
    } adbms_raw_cell_temp_module2;
    struct {
        uint64_t index: 8;
        uint64_t temp_left: 16;
        uint64_t temp_right: 16;
    } adbms_raw_cell_temp_module3;
    struct {
        uint64_t index: 8;
        uint64_t temp_left: 16;
        uint64_t temp_right: 16;
    } adbms_raw_cell_temp_module4;
    struct {
        uint64_t index: 8;
        uint64_t temp_left: 16;
        uint64_t temp_right: 16;
    } adbms_raw_cell_temp_module5;
    struct {
        uint64_t can1_tx_queue_overflow: 8;
        uint64_t can2_tx_queue_overflow: 8;
        uint64_t can1_tx_fail: 8;
        uint64_t can2_tx_fail: 8;
        uint64_t can_rx_queue_overflow: 8;
        uint64_t can1_rx_overrun: 8;
        uint64_t can2_rx_overrun: 8;
    } adbms_can_stats;
    struct {
        uint64_t current_channel_1: 16;
        uint64_t current_channel_2: 16;
    } adbms_i_sense;
    struct {
        uint64_t cmd: 8;
        uint64_t data: 32;
    } a_box_bl_cmd;
    uint8_t raw_data[8];
} __attribute__((packed)) CanParsedData_t;
/* END AUTO MESSAGE STRUCTURE */

// contains most up to date received
// type for each variable matches that defined in JSON
/* BEGIN AUTO CAN DATA STRUCTURE */
typedef struct {
    struct {
        uint8_t cmd;
        uint32_t data;
    } a_box_bl_cmd;
} can_data_t;
/* END AUTO CAN DATA STRUCTURE */

extern can_data_t can_data;

/* BEGIN AUTO EXTERN CALLBACK */
extern void a_box_bl_cmd_CALLBACK(CanParsedData_t* msg_data_a);
extern void handleCallbacks(uint16_t id, bool latched);
extern void set_fault_daq(uint16_t id, bool value);
extern void return_fault_control(uint16_t id);
extern void send_fault(uint16_t id, bool latched);
/* END AUTO EXTERN CALLBACK */

/* BEGIN AUTO EXTERN RX IRQ */
/* END AUTO EXTERN RX IRQ */

/**
 * @brief Setup queue and message filtering
 *
 * @param q_rx_can RX buffer of CAN messages
 */
void initCANParse(void);

/**
 * @brief Pull message off of rx buffer,
 *        update can_data struct,
 *        check for stale messages
 */
void canRxUpdate();

/**
 * @brief Process any rx message callbacks from the CAN Rx IRQ
 *
 * @param rx rx data from message just recieved
 */
void canProcessRxIRQs(CanMsgTypeDef_t* rx);

extern volatile uint32_t last_can_rx_time_ms;

#endif
