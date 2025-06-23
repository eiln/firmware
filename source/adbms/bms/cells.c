
#include "main.h"
#include "adbms/adbms.h"
#include "bms/bms.h"
#include "math.h"
#include "bms_common.h"

static void bms_read_cells(bms_t *bms);
static void bms_check_cells(bms_t *bms);
static void process_cell_readings(bms_t *bms);

void bms_cells_start(void)
{
    adBms6830_Adcv(ADCV_RD_ON, ADCV_CONT_CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
    bms_mDelay(1);
}

void bms_cells_update(void)
{
    printf("\n");
    bms_read_cells(&bms);
}

static void print_c_voltages(void)
{
    printf("C-ADC Voltages:\n");
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int i = 0; i < TOTAL_CELL; i++)
        {
            printf("Cell %02d: %.4f ", i, data.cell_v_c[ic][i]);
            if (i % 4 == 3)
            {
                printf("\n");
            }
        }
    }
}

static void print_s_voltages(void)
{
    printf("S-ADC Voltages:\n");
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int i = 0; i < TOTAL_CELL; i++)
        {
            printf("Cell %02d: %.4f ", i, data.cell_v_s[ic][i]);
            if (i % 4 == 3)
            {
                printf("\n");
            }
        }
    }
}

static void bms_read_cells(bms_t *bms)
{
    #if 0
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

    // TODO open wire switches
#if 0
    adBms6830_Adsv(ADCV_CONT_SINGLE, DCP_OFF, OW_ON_EVEN_CH);
    bms_mDelay(8);
    bms_readSVoltages();

    adBms6830_Adsv(ADCV_CONT_SINGLE, DCP_OFF, OW_ON_ODD_CH);
    bms_mDelay(8);
    bms_readSVoltages();
#endif

    adBms6830_Adsv(ADCV_CONT_SINGLE, DCP_OFF, OW_OFF_ALL_CH);
    bms_mDelay(8);
    bms_readSVoltages();
    // print_s_voltages();
    bms_readCellVoltages();

    bms->cells_ok = !bms_any_fault(BMS_ERROR_RXPEC);
    bms_check_cells(bms);
    if (bms->cells_ok)
    {
        print_c_voltages();
        process_cell_readings(bms);
    }
    else
    {
        bms_error("Bad cell readings! Discarding\n");
    }
}

enum
{
    MOD_VOLT_MIN = 0,
    MOD_VOLT_MAX,
    MOD_VOLT_AVG,
    MOD_VOLT_NUM,
};

static void bms_check_cells(bms_t *bms)
{
    bool set;

    #define CELL_REDUN_DELTA_MAX (0.05f) // V

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int cell = 0; cell < TOTAL_CELL; cell++)
        {
            if (bms->cells_ok)
            {
                float delta = fabsf(data.cell_v_c[ic][cell] - data.cell_v_s[ic][cell]);
                set = delta > CELL_REDUN_DELTA_MAX;
                BMS_SET_FAULT_CELL_DEBUG(cell, BMS_ERROR_CELL_REDUN, delta);
            }
            else
            {
                bms_set_fault_cell(ic, cell, BMS_ERROR_CELL_REDUN, false);
            }
        }
    }
}

static void print_pack_readings(bms_t *bms, float pack_vstat[MOD_VOLT_NUM])
{
    printf("pack: total: %.3f min: %.3f max: %.3f avg: %.3f delta: %.3f \n",
        bms->pack_voltage, 
        pack_vstat[MOD_VOLT_MIN], pack_vstat[MOD_VOLT_MAX], pack_vstat[MOD_VOLT_AVG],
        pack_vstat[MOD_VOLT_MAX] - pack_vstat[MOD_VOLT_MIN]);
}

static void process_cell_readings(bms_t *bms)
{
    float pack_volts;
    float volts;
    bool set;

    float pack_vstat[MOD_VOLT_NUM]; // min, max, avg
    float mod_vstat[TOTAL_AD68][MOD_VOLT_NUM];

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        float min = data.cell_v_c[ic][0];
        float max = data.cell_v_c[ic][0];
        float avg = 0.0f;
        for (int cell = 0; cell < TOTAL_CELL; cell++)
        {
            volts = data.cell_v_c[ic][cell];
            min = MIN(min, volts);
            max = MAX(max, volts);
            avg += volts;
            pack_volts += volts;
        }
        avg /= TOTAL_CELL;

        // Average is fine since delta (shouldn't be) that large
        #define WEAK_CELL_DELTA (0.10f) // V
        for (int cell = 0; cell < TOTAL_CELL; cell++)
        {
            volts = data.cell_v_c[ic][cell];
            set = volts < avg && (avg - volts) >= WEAK_CELL_DELTA;
            set |= volts > avg && (volts - avg) >= WEAK_CELL_DELTA;
            bms_set_fault_cell(ic, cell, BMS_ERROR_CELL_WEAK, set);
            
        }

        mod_vstat[ic][MOD_VOLT_MIN] = min;
        mod_vstat[ic][MOD_VOLT_MAX] = max;
        mod_vstat[ic][MOD_VOLT_AVG] = avg;

        if (!ic)
        {
            pack_vstat[MOD_VOLT_MIN] = min;
            pack_vstat[MOD_VOLT_MAX] = max;
            pack_vstat[MOD_VOLT_AVG] = avg;
        }
        else
        {
            pack_vstat[MOD_VOLT_MIN] = MIN(min, pack_vstat[MOD_VOLT_MIN]);
            pack_vstat[MOD_VOLT_MAX] = MAX(max, pack_vstat[MOD_VOLT_MAX]);
            pack_vstat[MOD_VOLT_AVG] += avg;
        }
    }
    pack_vstat[MOD_VOLT_AVG] /= TOTAL_AD68;

    #define WEAK_PACK_DELTA (10.0f) // V
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        float avg = pack_vstat[MOD_VOLT_AVG];
        set = volts < avg && (avg - volts) >= WEAK_PACK_DELTA;
        set |= volts > avg && (volts - avg) >= WEAK_PACK_DELTA;
        bms_set_fault(ic, BMS_ERROR_PACK_WEAK, set);
    }

    bms->cell_v_max = ema_filter(pack_vstat[MOD_VOLT_MAX], bms->cell_v_max, 0.50f);
    bms->pack_voltage = ema_filter(pack_volts, bms->pack_voltage, 0.50f);
    print_pack_readings(bms, pack_vstat);
}
