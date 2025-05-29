#ifndef __ADBMS_MCU_H__
#define __ADBMS_MCU_H__

#include "adbms_conf.h"

#define DATA_LEN       (6)       // Data
#define DATAPKT_LEN    (6 + 2)   // Data + DPEC
#define CMD_LEN        (2)       // Cmd
#define CMDPKT_LEN     (2 + 2)   // Cmd + PEC

struct bms_data
{
    float cell_v_c[TOTAL_AD68][TOTAL_CELL]; // C-ADC cell voltage (V)
    float cell_v_s[TOTAL_AD68][TOTAL_CELL]; // S-ADC cell voltage (V)

    float aux_v[TOTAL_AD68][TOTAL_AUX]; // AUX GPIO (V)
    float aux_ow_v[TOTAL_AD68][TOTAL_AUX]; // AUX GPIO open-wire (V)

    float vmv[TOTAL_AD68]; // V- to S1N (V)
    float vpv[TOTAL_AD68]; // V+ to V- (V)

    float vd[TOTAL_AD68]; // Digital (V)
    float va[TOTAL_AD68]; // Analog (V)

    float vref2[TOTAL_AD68]; // Refernce voltage (V)
    float itmp[TOTAL_AD68]; // Internal die temperature (C)
};

typedef enum
{
    BMS_ERROR_SID = 0,  // Device ID
    BMS_ERROR_RXPEC,    // RX PEC mismatch
    BMS_ERROR_CONFIG,   // Config TX failed
    BMS_ERROR_POLL_TIMEOUT, // Poll Timeout
    BMS_ERROR_VPV,      // V+ to V−
    BMS_ERROR_VMV,      // S1N to V−
    BMS_ERROR_VA_UV,    // Analog power undervoltage
    BMS_ERROR_VA_OV,    // Analog power overvoltage
    BMS_ERROR_VD_UV,    // Digital power undervoltage
    BMS_ERROR_VD_OV,    // Digital power overvoltage
    BMS_ERROR_VREG,     // Regulated Power
    BMS_ERROR_VREF2,    // Vref2 for thermistors
    BMS_ERROR_ITMP_UT,  // Internal die temperature under temperature
    BMS_ERROR_ITMP_OT,  // Internal die temperature over temperature

    BMS_ERROR_CELL_OW,    // Cell open-wire
    BMS_ERROR_CELL_UV,    // Cell undervoltage
    BMS_ERROR_CELL_OV,    // Cell overvoltage
    BMS_ERROR_CELL_REDUN, // Cell redundant measurement

    BMS_ERROR_AUX_OW,     // AUX open-wire
    BMS_ERROR_AUX_UT,     // AUX under temperature
    BMS_ERROR_AUX_OT,     // AUX over temperature
    BMS_ERROR_AUX_REDUN,  // AUX over temperature

    BMS_ERROR_COUNT,
} bms_error_t;

#define BMS_GET_ERROR_MASK(field) (1 << (field))

extern struct bms_data bms;
#define BMS_POLL_TIMEOUT (50) // ms

uint32_t bms_getTick(void);

void adbms_transmit_cmd(uint8_t cmd[CMD_LEN]);
void adbms_transmit_data(uint8_t cmd[CMD_LEN], uint8_t txdata[TOTAL_AD68][DATA_LEN]);
uint32_t adbms_transmit_poll(uint8_t cmd[CMD_LEN]);
bool adbms_receive(uint8_t cmd[CMD_LEN], uint8_t data[TOTAL_AD68][DATA_LEN]);
void adbms_print_rxdata(uint8_t data[TOTAL_AD68][DATA_LEN]);

uint32_t bms_pack_faults(bms_error_t field);
void bms_set_fault(int ic, bms_error_t field, bool set);
void bms_set_fault_aux(int ic, int aux, bms_error_t field, bool set);
void bms_set_fault_all(bms_error_t field, bool set);
uint32_t bms_get_fault_duration(int ic, bms_error_t field);

#endif // __ADBMS_MCU_H__
