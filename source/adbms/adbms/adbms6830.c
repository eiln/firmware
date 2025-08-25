
#include "main.h"
#include "adbms6830.h"
#include "adbms_mcu.h"
#include "adbms_regs.h"

#include "string.h"

ic_ad68_t ic_ad68[TOTAL_AD68];
struct bms_data data;

uint8_t  txData[TOTAL_AD68][DATA_LEN];
uint8_t  rxData[TOTAL_AD68][DATA_LEN];

#define ADBMS_6830B_SID (0b000011)

bool adbms_checkalive(void)
{
    uint8_t rxdata[TOTAL_AD68][DATA_LEN];
    if (!adbms_receive(RDSID, rxdata))
    {
        return false;
    }

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        uint8_t sid = rxdata[ic][1]; // SID1 [1:6]
        sid = (sid >> 1) & 0x3f;
        bms_set_fault(ic, BMS_ERROR_SID, sid != ADBMS_6830B_SID);
    }

    return true;
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

#if 0
Default
IC0: 0x01, 0x00, 0x00, 0xFF, 0x03, 0x00, CC: 0 |
IC0: 0x00, 0xF8, 0x7F, 0x00, 0x00, 0x00, CC: 0 |

Init
IC0: 0x86, 0x00, 0x00, 0xFF, 0x03, 0x08, CC: 2 |
IC0: 0xDC, 0x5E, 0x46, 0x00, 0x00, 0x00, CC: 2 |
#endif

bool bms_init(void)
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
    // If OWRNG = 0, soak time = 2^(6 + OWA[2:0]) clocks (32 us to 4.1 ms).
    // If OWRNG = 1, soak time = 2^(13 + OWA[2:0]) clocks (4.1 ms to 524 ms).
    uint8_t soakon = 0b1; // soak time enable
    uint8_t owrng = 0b0; // short soak time
    uint8_t owa = 0b000; // short soak time
	buff_6830_a[2] = (soakon << 7) | (owrng << 6) | (owa << 3);

    // All GPIO pull down off
	buff_6830_a[3] = 0b11111111; // GPIOs [8:0] pull down OFF
	buff_6830_a[4] = 0b00000011; // GPIOs 9/10 pull down OFF

    uint8_t snap_st = 0b0;
    uint8_t mute_st = 0b0;
    uint8_t comm_bk = 0b0;
    uint8_t fc = 0b000;
	buff_6830_a[5] = (snap_st << 5) | (mute_st << 4) | (comm_bk << 3) | (fc);

	// We need to set bit 3 in the last byte for the last one in the daisy chain. We'll do it when we create the final buffer that is sent

    /* 6830 CFGB */
    uint16_t vuv = get_threshold_voltage(0.2);
    uint16_t vov = get_threshold_voltage(4.2);
    // Cell undervoltage threshold = VUV × 16 × 150 μV + 1.5 V.
    // Cell overvoltage threshold = VOV × 16 × 150 μV + 1.5 V.
	buff_6830_b[0] = vuv & 0xff;
	buff_6830_b[1] = ((vov & 0xf) << 4) | ((vuv >> 8) & 0xf); // Undervoltage and overvoltage
	buff_6830_b[2] = ((vov >> 4) & 0xff); // Over voltage. Both are set to 1.5V when these registers are set to 0x00
    // discharge settings
	buff_6830_b[3] = 0x00; // Enable discharge timer monitor if extended balancing (don't)
	buff_6830_b[4] = 0x00; // DCC to zero. Not sure what to set here
	buff_6830_b[5] = 0x00; // DCC to zero

    uint8_t txData_a[TOTAL_AD68][DATA_LEN];
    uint8_t txData_b[TOTAL_AD68][DATA_LEN];
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        if (ic == (TOTAL_AD68 - 1))
        {
            // Set termination bit
            // Last iter so it's fine to modify
            buff_6830_a[5] = (0x01 << 3);
        }
        memcpy(txData_a[ic], buff_6830_a, DATA_LEN);
        memcpy(txData_b[ic], buff_6830_b, DATA_LEN);
    }

    adbms_transmit_data(WRCFGA, txData_a);
    adbms_transmit_data(WRCFGB, txData_b);

    // Check if config has been sent
    if (!adbms_receive(RDCFGA, rxData)) return false;
    if (memcmp(txData_a, rxData, sizeof(txData_a) != 0))
    {
        // TODO for now just set it on all of them
        bms_set_fault_all(BMS_ERROR_CONFIG, true);
        return false;
    }

    if (!adbms_receive(RDCFGB, rxData)) return false;
    if (memcmp(txData_b, rxData, sizeof(txData_b) != 0))
    {
        bms_set_fault_all(BMS_ERROR_CONFIG, true);
        return false;
    }

    bms_set_fault_all(BMS_ERROR_CONFIG, false);
    return true;
}

void adBms6830_Adcv(uint8_t rd, uint8_t cont, uint8_t dcp, uint8_t rstf, uint8_t owcs)
{
    uint8_t cmd[2];
    cmd[0] = 0x02 + rd;
    cmd[1] = (cont << 7) + (dcp << 4) + (rstf << 2) + (owcs & 0x03) + 0x60;
    adbms_transmit_cmd(cmd);
}

void adBms6830_Adsv(uint8_t cont, uint8_t dcp, uint8_t owcs)
{
    uint8_t cmd[2];
    cmd[0] = 0x01;
    cmd[1] = (cont << 7) + (dcp << 4) + (owcs & 0x03) + 0x68;
    adbms_transmit_cmd(cmd);
}

void adBms6830_Adax(uint8_t owaux, uint8_t pup, uint8_t ch)
{
    // 1 0 OW PUP CH[4] 0 1 CH[3] CH[2] CH[1] CH[0]
    uint8_t cmd[2];
    cmd[0] = 0x04 + owaux;
    cmd[1] = (pup << 7) + (((ch >> 4) & 0x01) << 6) + (ch & 0x0F) + 0x10;
    adbms_transmit_cmd(cmd);
}

/* CELL */

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

static inline float getVoltage(int data)
{
    float voltage_float; // V
    voltage_float = ((data + 10000) * 0.000150f);
    return voltage_float;
}

static inline void bms_read_cell_v_c(int ic, int group, int idx)
{
    int16_t raw = get_i16(rxData, ic, idx);
    data.cell_v_c[ic][group * 3 + idx] = getVoltage(raw);
}

void bms_readCellVoltages(void)
{
    uint8_t *cmdList[6] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};
    //bms_receiveData(RDCVALL, rxData, rxPec, rxCc);
    // TODO look into RDCVALL
    // need to change buffer size

    #define BMS_GET_C_V(ic, idx) ()
    for (int group = 0; group < 6; group++)
    {
        if (!adbms_receive(cmdList[group], rxData))
        {
            return;
        }
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            switch (group)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                    bms_read_cell_v_c(ic, group, 0);
                    bms_read_cell_v_c(ic, group, 1);
                    bms_read_cell_v_c(ic, group, 2);
                break;
                case 5:
                    bms_read_cell_v_c(ic, group, 0);
                break;
            }
        }
    }
}

static inline void bms_read_cell_v_s(int ic, int group, int idx)
{
    int16_t raw = get_i16(rxData, ic, idx);
    data.cell_v_s[ic][group * 3 + idx] = getVoltage(raw);
}

void bms_readSVoltages(void)
{
    uint8_t *cmdList[6] = {RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF};
    //bms_receiveData(RDCVALL, rxData, rxPec, rxCc);
    // TODO look into RDCVALL
    // need to change buffer size
    for (int group = 0; group < 6; group++)
    {
        if (!adbms_receive(cmdList[group], rxData))
        {
            return;
        }
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            switch (group)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                    bms_read_cell_v_s(ic, group, 0);
                    bms_read_cell_v_s(ic, group, 1);
                    bms_read_cell_v_s(ic, group, 2);
                break;
                case 5:
                    bms_read_cell_v_s(ic, group, 0);
                break;
            }
        }
    }
}

/* STAT */

#if 0
Stat C:
IC0: 0xFF, 0xFF, 0x00, 0x20, 0xFF, 0xFB, CC: 4 |

Stat D:
IC0: 0x45, 0x55, 0x55, 0x55, 0xFF, 0x00, CC: 4 |
#endif

void bms_checkCellVoltagesStatC(void)
{
    #if 0
    // statC is useless
    debug_printf("Stat C:\n");
    if (!adbms_receive(RDSTATC, rxData))
    {
        return; // TODO exit
    }
    adbms_print_rxdata(rxData);
    #endif

    debug_printf("Stat D:\n");
    if (!adbms_receive(RDSTATD, rxData))
    {
        return; // TODO exit
    }
    adbms_print_rxdata(rxData);
    // TODO check uv/ov
}

/* AUX */
static inline void bms_read_aux_v(int ic, int group, int idx, bool ow)
{
    int16_t raw = get_i16(rxData, ic, idx);
    if (!ow)
        data.aux_v[ic][group * 3 + idx] = getVoltage(raw);
    else
        data.aux_ow_v[ic][group * 3 + idx] = getVoltage(raw);
}

void bms_readAuxVoltages(bool ow)
{
    uint8_t *cmdList[4] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD};
    for (int group = 0; group < 4; group++)
    {
        if (!adbms_receive(cmdList[group], rxData))
        {
            return;
        }
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            switch (group)
            {
                case 0:
                case 1:
                case 2:
                    bms_read_aux_v(ic, group, 0, ow);
                    bms_read_aux_v(ic, group, 1, ow);
                    bms_read_aux_v(ic, group, 2, ow);
                    // TODO parse aux voltage into C for thermistors
                break;
                case 3:
                    // for group D: G10V then VMV, VPV
                    bms_read_aux_v(ic, group, 0, ow);
                    // Do not overwrite vmv values with values obtained during ow check
                    if (!ow)
                    {
                        int16_t vmv = get_i16(rxData, ic, 1);
                        int16_t vpv = get_i16(rxData, ic, 2);
                        data.vmv[ic] = getVoltage(vmv);
                        data.vpv[ic] = 25 * (vpv * 0.00015 + 1.5);
                    }
                break;
            }
        }
    }
}

void bms_readAuxVoltagesAll(void)
{
    bms_readAuxVoltages(false); // Should not read all with ow check

    if (!adbms_receive(RDSTATB, rxData))
    {
        return; // TODO exit
    }
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        data.vd[ic] = getVoltage(get_i16(rxData, ic, 0));
        data.va[ic] = getVoltage(get_i16(rxData, ic, 1));
    }

    if (!adbms_receive(RDSTATA, rxData))
    {
        return; // TODO exit
    }
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        // reference = VREF2 × 150 μV +1.5 V
        data.vref2[ic] = getVoltage(get_i16(rxData, ic, 0));
        int16_t itmp = get_i16(rxData, ic, 1);
        // = (ITMP × 150 μV + 1.5 V)/7.5 mV/°C – 273°C.
        data.itmp[ic] = (itmp * 0.00015 + 1.5) / 0.0075 - 273;
    }
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
    adbms_transmit_data(WRPWM1, txData);
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

    // Write config A
    adbms_transmit_data(WRPWM2, txData);
}

void bms_writePwm(uint8_t pwm[TOTAL_AD68][TOTAL_CELL])
{
    bms_writePwmA(pwm);
    bms_writePwmB(pwm);
}

void bms_startDischarge(uint8_t pwm[TOTAL_AD68][TOTAL_CELL])
{
    bms_writePwmA(pwm); // Send the PWM configs
    bms_writePwmB(pwm); // Send the PWM configs
}

void bms_stopDischarge(void)
{
    adbms_transmit_cmd(SRST);      // Put all devices to sleep
}
