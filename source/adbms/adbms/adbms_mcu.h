#ifndef __ADBMS_MCU_H__
#define __ADBMS_MCU_H__

#include "adbms_common.h"

#define DATA_LEN       (6)       // Data
#define DATAPKT_LEN    (6 + 2)   // Data + DPEC
#define CMD_LEN        (2)       // Cmd
#define CMDPKT_LEN     (2 + 2)   // Cmd + PEC

#define BMS_WAKEUP_DELAY 5       /// 1ms for 2950   /* BMS ic wakeup delay  */

struct bms_data
{
    int16_t cell_voltages_raw[TOTAL_AD68][TOTAL_CELL]; // cell voltage (raw)
    float cell_voltages_parsed[TOTAL_AD68][TOTAL_CELL]; // cell voltage (V)

    // RDAUXA - RDAUXD
    int16_t aux_voltages_raw[TOTAL_AD68][TOTAL_AUX]; // therm/temps (raw)
    float aux_voltages_parsed[TOTAL_AD68][TOTAL_AUX]; // therm/temps (V)
    float vmv[TOTAL_AD68]; // V+ to V- (V)
    float vpv[TOTAL_AD68]; // V+ to V- (V)

    // RDSTATB: internal supply voltages
    float vd[TOTAL_AD68]; // digital
    float va[TOTAL_AD68]; // analog

    // RDSTATA:
    float vref2[TOTAL_AD68]; // refernce voltage (V)
    float itmp[TOTAL_AD68]; // internal die temperature (C)
};

extern struct bms_data bms;

void bms_wakeupChain(void);
void bms_transmitCmd(uint8_t cmd[CMD_LEN]);
void bms_transmitData(uint8_t cmd[CMD_LEN], uint8_t txBuffer[TOTAL_AD68][DATA_LEN]);
void bms_transmitPoll(uint8_t cmd[CMD_LEN]);
void bms_receiveData(uint8_t cmd[CMD_LEN], uint8_t rxBuffer[TOTAL_AD68][DATA_LEN], uint16_t rxPec[TOTAL_AD68], uint8_t rxCc[TOTAL_AD68]);

bool adbms_receive(uint8_t cmd[CMD_LEN], uint8_t data[TOTAL_AD68][DATA_LEN]);
void adbms_print_rxdata(uint8_t data[TOTAL_AD68][DATA_LEN]);

#endif // __ADBMS_MCU_H__
