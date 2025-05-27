#ifndef __ADBMS_MCU_H__
#define __ADBMS_MCU_H__

#include "adbms_common.h"

#define DATA_LEN       (6)       // Data
#define DATAPKT_LEN    (6 + 2)   // Data + DPEC
#define CMD_LEN        (2)       // Cmd
#define CMDPKT_LEN     (2 + 2)   // Cmd + PEC

struct bms_data
{
    int16_t cell_voltages_raw[TOTAL_AD68][TOTAL_CELL]; // cell voltage (raw)
    float cell_voltages_parsed[TOTAL_AD68][TOTAL_CELL]; // cell voltage (V)

    // RDAUXA - RDAUXD
    int16_t aux_voltages_raw[TOTAL_AD68][TOTAL_AUX]; // therm/temps (raw)
    float aux_voltages_parsed[TOTAL_AD68][TOTAL_AUX]; // therm/temps (V)
    float aux_voltages_ow[TOTAL_AD68][TOTAL_AUX]; // therm/temps (V)
    float vmv[TOTAL_AD68]; // V- to S1N (V)
    float vpv[TOTAL_AD68]; // V+ to V- (V)

    // RDSTATB: internal supply voltages
    float vd[TOTAL_AD68]; // digital
    float va[TOTAL_AD68]; // analog

    // RDSTATA:
    float vref2[TOTAL_AD68]; // refernce voltage (V)
    float itmp[TOTAL_AD68]; // internal die temperature (C)
};

extern struct bms_data bms;

void adbms_transmit_cmd(uint8_t cmd[CMD_LEN]);
void adbms_transmit_data(uint8_t cmd[CMD_LEN], uint8_t txdata[TOTAL_AD68][DATA_LEN]);
uint32_t adbms_transmit_poll(uint8_t cmd[CMD_LEN]);
bool adbms_receive(uint8_t cmd[CMD_LEN], uint8_t data[TOTAL_AD68][DATA_LEN]);
void adbms_print_rxdata(uint8_t data[TOTAL_AD68][DATA_LEN]);

#endif // __ADBMS_MCU_H__
