#ifndef __ADBMS_MCU_H__
#define __ADBMS_MCU_H__

#include "adbms_conf.h"
#include <stdint.h>

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

extern struct bms_data bms;
#define BMS_POLL_TIMEOUT (50) // ms

uint32_t bms_getTick(void);

void adbms_transmit_cmd(uint8_t cmd[CMD_LEN]);
void adbms_transmit_data(uint8_t cmd[CMD_LEN], uint8_t txdata[TOTAL_AD68][DATA_LEN]);
uint32_t adbms_transmit_poll(uint8_t cmd[CMD_LEN]);
bool adbms_receive(uint8_t cmd[CMD_LEN], uint8_t data[TOTAL_AD68][DATA_LEN]);
void adbms_print_rxdata(uint8_t data[TOTAL_AD68][DATA_LEN]);

#endif // __ADBMS_MCU_H__
