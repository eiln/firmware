
#include "main.h"
#include "adbms_mcu.h"
#include "adbms_regs.h"
#include "adbms6830.h"
#include "adbms_common.h"

#include "string.h"
#define printfDma debug_printf

ic_ad68_t ic_ad68[TOTAL_AD68];
struct bms_data bms;

uint8_t  txData[TOTAL_AD68][DATA_LEN];
uint8_t  rxData[TOTAL_AD68][DATA_LEN];
uint16_t rxPec[TOTAL_AD68];
uint8_t  rxCc[TOTAL_AD68];

#define ADBMS_RXPEC_NOERROR ((uint8_t)0)

static bool bms_checkRxFault(uint8_t data[TOTAL_AD68][DATA_LEN], uint16_t pec[TOTAL_AD68], uint8_t cc[TOTAL_AD68])
{
    uint8_t errorMask = bms_checkRxPec(data, pec, cc);
    if (errorMask) // DEBUG
    {
        printf("PEC ERROR - IC:");
        for(int ic = 0; ic < TOTAL_AD68; ic++)
        {
            if (errorMask & (1 << ic))
            {
                printf(" %d,", ic);
            }
        }
        printf("\n");
        // TODO send errormask over CAN
    } // END OF DEBUG

    return !!errorMask; // true if fault
}

bool adbms_receive(uint8_t cmd[CMD_LEN])
{
    // LOCK uses global
    bms_receiveData(cmd, rxData, rxPec, rxCc);
    if (bms_checkRxFault(rxData, rxPec, rxCc))
    {
        bmsmaster.error |= BMS_ERROR_RXPEC;
        return false;
    }
    else
    {
        bmsmaster.error &= ~BMS_ERROR_RXPEC; // TODO what about before
    }
    return true;
}

static inline uint8_t get_u8(uint8_t rxData[TOTAL_AD68][DATA_LEN], int ic, int index)
{
    return (uint8_t)(rxData[ic][index] & 0xff);
}

#define ADBMS_6830B_SID (0b000011)

uint32_t adbms_checkalive(void)
{
    uint32_t conn = 0; // bitmask

    #if 0
    bms_receiveData(RDSID, rxData, rxPec, rxCc);
    if (bms_checkRxFault(rxData, rxPec, rxCc))
    {
        return conn;
    }
    // bms_printRawData(rxData, rxCc);
    #if 0
    if (!adbms_receive(RDSID) == false)
    {
        return conn;
    }
    #endif
    #endif
    if (!adbms_receive(RDSID))
    {
        return conn;
    }

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        uint8_t sid = get_u8(rxData, ic, 1); // SID1 [1:6]
        sid = (sid >> 1) & 0x3f;
        if (sid == ADBMS_6830B_SID)
        {
            conn |= (1 << ic);
        }
    }

    return conn;
}

static inline uint16_t get_threshold_voltage(float voltage)
{
  uint16_t vov_value;
  uint8_t rbits = 12;
  voltage = (voltage - 1.5);
  voltage = voltage / (16 * 0.000150);
  vov_value = (uint16_t )(voltage + 2 * (1 << (rbits - 1)));
  vov_value &= 0xFFF;
  return vov_value;
}

void bms_init(void)
{
	uint8_t buff_6830_a[DATA_LEN] = {0};
	uint8_t buff_6830_b[DATA_LEN] = {0};

    /* 6830 CFGA */
    uint8_t refon = 0b1; // 1 = reference remains powered up until watchdog timeout. TODO determine
    uint8_t cth = 0b110; // C-ADC vs. S-ADC comparison voltage threshold
    // 110: 25.05 mV
	buff_6830_a[0] = (refon << 7) | (cth & 7);

    // Asserts various flags in Status Register C
    uint8_t flag_d = 0b00000000;
	buff_6830_a[1] = flag_d; // All flags = 0

    // Control reg for SOAK functions
	buff_6830_a[2] = 0x00;

    // All GPIO pull down off
	buff_6830_a[3] = 0xFF; // GPIOs [8:0] are all pulled down. (For ADC measurements)
	buff_6830_a[4] = 0x03; // GPIOs 10 and 9 pulled down.

	buff_6830_a[5] = (0x00<<3); // bits [2:0] is for filter.
	// We need to set bit 3 in the last byte for the last one in the daisy chain. We'll do it when we create the final buffer that is sent

    /* 6830 CFGB */
    //uint16_t vuv = get_threshold_voltage(2.5); // for testing
    uint16_t vuv = get_threshold_voltage(0.8);
    uint16_t vov = get_threshold_voltage(4.2);
    // Cell undervoltage threshold = VUV × 16 × 150 μV + 1.5 V.
    // Cell overvoltage threshold = VOV × 16 × 150 μV + 1.5 V.
	buff_6830_b[0] = vuv & 0xff;
	buff_6830_b[1] = ((vov & 0xf) << 4) | ((vuv >> 8) & 0xf); // Undervoltage and overvoltage
	buff_6830_b[2] = ((vov >> 4) & 0xff); // Over voltage. Both are set to 1.5V when these registers are set to 0x00

	buff_6830_b[3] = 0x00; // Enable discharge timer monitor if extended balancing (don't)
	buff_6830_b[4] = 0x00; // DCC to zero. Not sure what to set here
	buff_6830_b[5] = 0x00; // DCC to zero
	/* ======================= End of config definition =============================== */

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        if (ic == (TOTAL_AD68 - 1))
        {
            buff_6830_a[5] = (0x01 << 3); // last iter so it's fine
        }
        memcpy(&ic_ad68[ic].cfa_Tx, buff_6830_a, DATA_LEN);
        memcpy(&ic_ad68[ic].cfb_Tx, buff_6830_b, DATA_LEN);
    }

    bms_writeConfigA();
    bms_writeConfigB();
}

void bms_writeConfigA(void)
{
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        memcpy(txData[ic], &ic_ad68[ic].cfa_Tx, DATA_LEN);
    }
    // write config A
    bms_transmitData(WRCFGA, txData);
}

void bms_writeConfigB(void)
{
    // Fill buffer with the other ad6830 data
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        memcpy(txData[ic], &ic_ad68[ic].cfb_Tx, DATA_LEN);
    }

    // write config B
    bms_transmitData(WRCFGB, txData);
}

void bms_readStatus(void)
{
    printfDma("Status: \n");
    uint8_t *cmdList[5] = {RDSTATA, RDSTATB, RDSTATC, RDSTATD, RDSTATE};
    int cell = 0;
    for (int i = 0; i < 5; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        bms_checkRxFault(rxData, rxPec, rxCc);
        bms_printRawData(rxData, rxCc);
    }
    //printStatus(tIC, &ic[0], Status, ALL_GRP);
}

void adBms6830_Adcv(uint8_t rd, uint8_t cont, uint8_t dcp, uint8_t rstf, uint8_t owcs)
{
    uint8_t cmd[2];
    cmd[0] = 0x02 + rd;
    cmd[1] = (cont<<7)+(dcp<<4)+(rstf<<2)+(owcs & 0x03) + 0x60;
    bms_transmitCmd(cmd);
    //bms_transmitPoll(PLADC);
}

void adBms6830_Adsv(uint8_t cont, uint8_t dcp, uint8_t owcs)
{
    uint8_t cmd[2];
    cmd[0] = 0x01;
    cmd[1] = (cont<<7)+(dcp<<4)+(owcs &0x03) + 0x68;
    bms_transmitCmd(cmd);
}

void bms_startAdcvCell(void)
{
    adBms6830_Adcv(ADCV_RD_OFF, ADCV_CONT_CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
}

void adBms6830_Adax(uint8_t owaux, uint8_t pup, uint8_t ch)
{
    // 1 0 OW PUP CH[4] 0 1 CH[3] CH[2] CH[1] CH[0]
    uint8_t cmd[2];
    cmd[0] = 0x04 + owaux;
    cmd[1] = (pup << 7) + (((ch >>4)&0x01)<<6) + (ch & 0x0F) + 0x10;
    bms_transmitCmd(cmd);
    //bms_transmitPoll(PLAUX1);
}

void bms_startAdcvAux(void)
{
  adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
}

#if 0
6830: Cell voltages:
IC1: 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, CC: 47 |   IC2: 0x5E, 0xE6, 0x55, 0xE6, 0xAA, 0x7E, CC: 47 |

IC1: 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, CC: 47 |   IC2: 0xF4, 0xEE, 0x75, 0xE7, 0x81, 0xE7, CC: 47 |

IC1: 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, CC: 47 |   IC2: 0x75, 0xE7, 0x74, 0xE7, 0x6F, 0xE7, CC: 47 |

IC1: 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, CC: 47 |   IC2: 0x8F, 0xE7, 0x7C, 0xE7, 0x78, 0xE7, CC: 47 |

IC1: 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, CC: 47 |   IC2: 0x8A, 0xE7, 0x7B, 0xE7, 0x7F, 0xE7, CC: 47 |

IC1: 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, CC: 47 |   IC2: 0x76, 0xE7, 0xFF, 0xFF, 0xFF, 0xFF, CC: 47 |

16 bits * 16 cells = 256 bits
each group: 8 * 6 = 48 bits
256 ceildiv 48 = 5.33 = 6

#endif

static inline int16_t get_i16(uint8_t rxData[TOTAL_AD68][DATA_LEN], int ic, int index)
{
    return (int16_t)(rxData[ic][index * 2 + 0] & 0xff) | ((int16_t)(rxData[ic][index * 2 + 1] & 0xff) << 8);
}

float getVoltage(int data)
{
    float voltage_float; //voltage in Volts
    voltage_float = ((data + 10000) * 0.000150);
    return voltage_float;
}

void bms_checkCellVoltagesStatC(void)
{
    #if 1
    debug_printf("stat C:\n");
    bms_receiveData(RDSTATC, rxData, rxPec, rxCc);
    bms_checkRxFault(rxData, rxPec, rxCc);
    bms_printRawData(rxData, rxCc);
    #endif
    // statC is useless

    debug_printf("Stat D:\n");
    bms_receiveData(RDSTATD, rxData, rxPec, rxCc);
    bms_checkRxFault(rxData, rxPec, rxCc);
    bms_printRawData(rxData, rxCc);
    // TODO check uv/ov
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        //uint16_t csxflt = get_i16(rxData, ic, 0);
        //debug_printf("statC 0: 0x%04x\n", csxflt);
        //debug_printf("statC 1: 0x%04x\n", (uint16_t)get_i16(rxData, ic, 1));
        //debug_printf("statC 2: 0x%04x\n", (uint16_t)get_i16(rxData, ic, 2));
        // reference = VREF2 × 150 μV +1.5 V
        //bms.vref2[ic] = getVoltage(get_i16(rxData, ic, 0));
        //int16_t itmp = get_i16(rxData, ic, 1);
        // = (ITMP × 150 μV + 1.5 V)/7.5 mV/°C – 273°C.
        //bms.itmp[ic] = (itmp * 0.00015 + 1.5) / 0.0075 - 273;
        //debug_printf("itmp: %.2f\n", bms.itmp[ic]);
    }
}

void bms_readCellVoltages(void)
{
    uint8_t *cmdList[6] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};
    //bms_receiveData(RDCVALL, rxData, rxPec, rxCc);
    // TODO look into RDCVALL
    // need to change buffer size
    for (int i = 0; i < 6; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        bms_checkRxFault(rxData, rxPec, rxCc);
        //bms_printRawData(rxData, rxCc);
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            switch (i)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                    bms.cell_voltages_raw[ic][i * 3 + 0] = get_i16(rxData, ic, 0);
                    bms.cell_voltages_raw[ic][i * 3 + 1] = get_i16(rxData, ic, 1);
                    bms.cell_voltages_raw[ic][i * 3 + 2] = get_i16(rxData, ic, 2);
                break;
                case 5:
                    bms.cell_voltages_raw[ic][i * 3 + 0] = get_i16(rxData, ic, 0); // index 15
                break;
            }
        }
    }

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int i = 0; i < TOTAL_CELL; i++)
        {
            bms.cell_voltages_parsed[ic][i] = getVoltage((int16_t)bms.cell_voltages_raw[ic][i]);
            debug_printf("Cell %02d: %f ", i, bms.cell_voltages_parsed[ic][i]);
        }
    }
    debug_printf("\n");
}

void bms_readSVoltages(void)
{
    uint8_t *cmdList[6] = {RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF};
    //bms_receiveData(RDCVALL, rxData, rxPec, rxCc);
    // TODO look into RDCVALL
    // need to change buffer size
    for (int i = 0; i < 6; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        bms_checkRxFault(rxData, rxPec, rxCc);
        //bms_printRawData(rxData, rxCc);
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            switch (i)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                    bms.cell_voltages_raw[ic][i * 3 + 0] = get_i16(rxData, ic, 0);
                    bms.cell_voltages_raw[ic][i * 3 + 1] = get_i16(rxData, ic, 1);
                    bms.cell_voltages_raw[ic][i * 3 + 2] = get_i16(rxData, ic, 2);
                break;
                case 5:
                    bms.cell_voltages_raw[ic][i * 3 + 0] = get_i16(rxData, ic, 0); // index 15
                break;
            }
        }
    }

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int i = 0; i < TOTAL_CELL; i++)
        {
            bms.cell_voltages_parsed[ic][i] = getVoltage((int16_t)bms.cell_voltages_raw[ic][i]);
            debug_printf("Cell %02d: %f ", i, bms.cell_voltages_parsed[ic][i]);
        }
    }
    debug_printf("\n");
}

void bms_readAuxVoltages(void)
{
    debug_printf("6830: Aux Voltages:\n");
    uint8_t *cmdList[4] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD};
    int cell = 0;
    for (int i = 0; i < 4; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        bms_checkRxFault(rxData, rxPec, rxCc);
        //bms_printRawData(rxData, rxCc);
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            switch (i)
            {
                case 0:
                case 1:
                case 2:
                    // TODO parse aux voltage into C
                    bms.aux_voltages_raw[ic][i * 3 + 0] = get_i16(rxData, ic, 0);
                    bms.aux_voltages_raw[ic][i * 3 + 1] = get_i16(rxData, ic, 1);
                    bms.aux_voltages_raw[ic][i * 3 + 2] = get_i16(rxData, ic, 2);
                break;
                case 3:
                    // for group D: G10V then VMV, VPV
                    bms.aux_voltages_raw[ic][i * 3 + 0] = get_i16(rxData, ic, 0); // index 9
                    int16_t vmv = get_i16(rxData, ic, 1);
                    int16_t vpv = get_i16(rxData, ic, 2);
                    bms.vmv[ic] = getVoltage(vmv); // V
                    bms.vpv[ic] = 25 * (vpv * 0.00015 + 1.5); // V
                    debug_printf("vmv: %.2f vpv: %.2f\n", bms.vmv[ic], bms.vpv[ic]);
                break;
            }
        }
    }

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int i = 0; i < TOTAL_AUX; i++)
        {
            bms.aux_voltages_parsed[ic][i] = getVoltage(bms.aux_voltages_raw[ic][i]);
            debug_printf("Aux %02d: %f ", i, bms.aux_voltages_parsed[ic][i]);
        }
    }

    // The main AUX ADC measures the internal supply voltages (VD and VA),
    bms_receiveData(RDSTATB, rxData, rxPec, rxCc);
    bms_checkRxFault(rxData, rxPec, rxCc);
    //bms_printRawData(rxData, rxCc);
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        bms.vd[ic] = getVoltage(get_i16(rxData, ic, 0));
        bms.va[ic] = getVoltage(get_i16(rxData, ic, 1));
        debug_printf("vd: %.2f va: %.2f\n", bms.vd[ic], bms.va[ic]);
    }

    #if 0
    pg 72
    Analog power supply
    voltage = voltage at the
    VREG pin. VD is off in sleep
    16-bit ADC measurement value of analog power supply voltage. Analog power supply voltage = VA × 150 μV + 1.5
V. The value of VA is set by external components and must be in the range of 4.5 V to 5.5 V for normal operation.
Reset to 0x7FFF after power-up, sleep, and to 0x8000 after clear command (CLRAUX).
    #endif

    bms_receiveData(RDSTATA, rxData, rxPec, rxCc);
    bms_checkRxFault(rxData, rxPec, rxCc);
    //bms_printRawData(rxData, rxCc);
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        // reference = VREF2 × 150 μV +1.5 V
        bms.vref2[ic] = getVoltage(get_i16(rxData, ic, 0));
        int16_t itmp = get_i16(rxData, ic, 1);
        // = (ITMP × 150 μV + 1.5 V)/7.5 mV/°C – 273°C.
        bms.itmp[ic] = (itmp * 0.00015 + 1.5) / 0.0075 - 273;
        debug_printf("itmp: %.2f\n", bms.itmp[ic]);
    }
    // 16-bit ADC measurement value of Internal Die temperature. Temperature measurement voltage = (ITMP × 150 μV + 1.5 V)/7.5 mV/°C – 273°C. Reset to 0x7FFF after power-up, sleep, and to 0x8000 after clear command
}

void bms_checkAuxVoltages(void)
{
    // check temps under threshold
    // check va, vd, etc
}

static void bms_writePwmA(uint8_t pwm[TOTAL_AD68][TOTAL_CELL])
{
    memset(txData[0], 0x00, DATA_LEN);
    // memset(&ic_ad68[0].pwma, 0x00, sizeof(ad68_pwma_t));

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        ic_ad68[ic].pwma.pwm1 = pwm[ic][0];
        ic_ad68[ic].pwma.pwm2 = pwm[ic][1];
        ic_ad68[ic].pwma.pwm3 = pwm[ic][2];
        ic_ad68[ic].pwma.pwm4 = pwm[ic][3];
        ic_ad68[ic].pwma.pwm5 = pwm[ic][4];
        ic_ad68[ic].pwma.pwm6 = pwm[ic][5];
        ic_ad68[ic].pwma.pwm7 = pwm[ic][6];
        ic_ad68[ic].pwma.pwm8 = pwm[ic][7];
        ic_ad68[ic].pwma.pwm9 = pwm[ic][8];
        ic_ad68[ic].pwma.pwm10 = pwm[ic][9];
        ic_ad68[ic].pwma.pwm11 = pwm[ic][10];
        ic_ad68[ic].pwma.pwm12 = pwm[ic][11];

        memcpy(txData[ic], &ic_ad68[ic].pwma, DATA_LEN);
    }

    // write config A
    bms_transmitData(WRPWM1, txData);
}

static void bms_writePwmB(uint8_t pwm[TOTAL_AD68][TOTAL_CELL])
{
    memset(txData[0], 0x00, DATA_LEN);
    // memset(&ic_ad68[0].pwmb, 0x00, sizeof(ad68_pwma_t));

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        ic_ad68[ic].pwmb.pwm13 = pwm[ic][12];
        ic_ad68[ic].pwmb.pwm14 = pwm[ic][13];
        ic_ad68[ic].pwmb.pwm15 = pwm[ic][14];
        ic_ad68[ic].pwmb.pwm16 = pwm[ic][15];

        memcpy(txData[ic], &ic_ad68[ic].pwmb, DATA_LEN);
    }

    // write config A
    bms_transmitData(WRPWM2, txData);
}

void bms_writePwm(uint8_t pwm[TOTAL_AD68][TOTAL_CELL])
{
    bms_writePwmA(pwm);
    bms_writePwmB(pwm);
    //(pwm2 & 0xf) << 4 | (pwm1 & 0xf)
}

void bms_startDischarge(uint8_t pwm[TOTAL_AD68][TOTAL_CELL])
{
    #if 0
    ic_ad68[0].cfb_Tx.dcto = 1;     // DC Timer in minutes (DTRNG = 0)
    ic_ad68[0].cfb_Tx.dtmen = 0;    // Disables Discharge Timer Monitor (DTM)
    ic_ad68[0].cfb_Tx.dcc = 0b1; // --- High priority discharge (bypasses PWM)
    bms_writeConfigB();             // Send the DCTO Timer config
    #endif

    bms_writePwmA(pwm); // Send the PWM configs
    bms_writePwmB(pwm); // Send the PWM configs
    #if 0
    ic_ad68[0].pwma.pwm1 = 0b0111;  // 4 bit pwm at 937 ms (for testing -> enables discharge for cell 1)

    // The PWM discharge functionality is possible in the standby, REF-UP, extended balancing and in the measure states
    // AND while the discharge timeout has not expired (DCTO ≠ 0)

    ic_ad68[0].cfb_Tx.dcto = 1;     // DC Timer in minutes (DTRNG = 0)
    ic_ad68[0].cfb_Tx.dtmen = 0;    // Disables Discharge Timer Monitor (DTM)
//    ic_ad68[0].cfb_Tx.dcc = 0b1; // --- High priority discharge (bypasses PWM)

    bms_writeConfigB();             // Send the DCTO Timer config
    bms_writePwmA();                // Send the PWM configs
    bms_writePwmB();                // Send the PWM configs
    #endif
}

void bms_stopDischarge(void)
{
    bms_wakeupChain();
    bms_transmitCmd(SRST);      // Put all devices to sleep
    printfDma("--- SOFT RESET --- \n");
}
