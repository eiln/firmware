
#include "main.h"
#include "adbms/adbms.h"
#include "math.h"

static void bms_read_cells(void);
static void bms_check_cells(void);
static void bms_send_cells(void);

void bms_monitor_cells(void)
{
    bms_read_cells();
    bms_check_cells();
    bms_send_cells();
}

static void bms_print_cell_voltages(void)
{
    debug_printf("C-ADC Voltages:\n");
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int i = 0; i < TOTAL_CELL; i++)
        {
            debug_printf("Cell %02d: %.4f ", i, data.cell_v_c[ic][i]);
            if (i % 4 == 3)
                debug_printf("\n");
        }
    }
    debug_printf("\n");
}

static void bms_read_cells(void)
{
    #if 0
    The direct method involves setting the redundancy bit (RD) in an
    ADCV command. In this case, the C-ADCs and S-ADCs are both
    triggered to provide redundancy. After 8 ms, the average results
    of the C-ADCs are compared to the results of the S-ADCs

    If the results do not match within the threshold set by the CTH[2:0] in
    Configuration Register A, the CSxFLT flag is set in the Status
    Register Group C.

    ADSV with DCP = 0, CONT = 1, OW = 0 (redundant check)
    ► ADSV with DCP = 0, CONT = 0, OW = 1 (even open wire
    check)
    ► ADSV with DCP = 0, CONT = 0, OW = 2 (odd open wire
    check)

    C-ADC conversions are usually started once during initialization:
    C-ADCs run in continuous mode, deliver measurement re-
    sults, and feed the IIR filter. No comparison between C-ADC
    and S-ADC results is performed. PWM discharge is ongoing
    and is not affected

    Thus, the whole redundant and open wire diagnostic takes 24
    ms to 32 ms and limits the maximum discharge duty cycle. In
    average, the discharge is inhibited for 0.5 × (32 ms + 24 ms) =
    28 ms. Assuming an FTTI of 100 ms, the maximum discharge duty
    cycle of the ADBMS6830B is limited to 72% (even if the PWM
    was configured to 100%, it is limited to 72% by the diagnostic
    measurements).
    #endif

    // 16 additional ADCs are dedicated to measure the 16 differential
    // inputs (SxP and SxN) synchronously with an input range of 0 V
    // to 5. 5 V and a sampling frequency of ~4 MHz, giving out results
    // every 8 ms.

    adBms6830_Adcv(ADCV_RD_OFF, ADCV_CONT_SINGLE, DCP_OFF, RSTF_OFF, OW_ON_EVEN_CH);
    bms_mDelay(1);
    bms_readCellVoltages(); // TODO store in ow slot and compare

    adBms6830_Adcv(ADCV_RD_OFF, ADCV_CONT_SINGLE, DCP_OFF, RSTF_OFF, OW_ON_ODD_CH);
    bms_mDelay(1);
    bms_readCellVoltages();

    adBms6830_Adsv(ADCV_CONT_SINGLE, DCP_OFF, OW_OFF_ALL_CH);
    bms_mDelay(8);
    bms_readSVoltages();

    adBms6830_Adcv(ADCV_RD_OFF, ADCV_CONT_SINGLE, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
    bms_mDelay(1);
    bms_checkCellVoltagesStatC();
    bms_readCellVoltages();
    bms_print_cell_voltages();
}

static void bms_check_cells(void)
{
    // TODO SIMD lol
    bool set;

    #define CELL_REDUN_DELTA_MAX (0.05) // V

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int i = 0; i < TOTAL_CELL; i++)
        {
            float delta = fabsf(data.cell_v_c[ic][i] - data.cell_v_s[ic][i]);
            set = delta > CELL_REDUN_DELTA_MAX;
            BMS_SET_FAULT_CELL_DEBUG(i, BMS_ERROR_CELL_REDUN, delta);
        }
    }
}

static void bms_send_cells(void)
{
    ;
}
