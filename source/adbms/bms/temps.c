
#include "main.h"
#include "adbms/adbms.h"

#include <math.h>

static void bms_read_temps(void);
static void bms_send_temps(void);

#define TEMP_MONITOR_MS (1000) // 1000 ms

void bms_temps_update(void)
{
    uint32_t now = getTick();
    if (!bms.temp_last_tick || (now - bms.temp_last_tick) >= TEMP_MONITOR_MS)
    {
        bms.temp_last_tick = now;
        bms_read_temps();
        bms_send_temps();
    }
}

static void bms_print_aux_voltages(bool ow)
{
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        printf("IC[%d]: ", ic);
        for (int aux = 0; aux < TOTAL_AUX; aux++)
        {
            float voltage;
            if (!ow)
                voltage = data.aux_v[ic][aux];
            else
                voltage = data.aux_ow_v[ic][aux];
            printf("%.2f ", voltage);
        }
        printf("\n");
    }
}

static void bms_print_aux_all(bool ow)
{
    bms_print_aux_voltages(ow);

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        printf("IC[%d]: vmv: %.2f vpv: %.2f vd: %.2f va: %.2f vref2: %.2f itmp: %.2f\n", ic, data.vmv[ic], data.vpv[ic], data.vd[ic], data.va[ic], data.vref2[ic], data.itmp[ic]);
    }
}

static void bms_aux_ow_check(void)
{
    bool set;

    // open-wire check on 10 GPIOs
    // GPIOs assumed pull-up, so should be no difference between pull-up and pull-down
    // AUX_ALL includes 10 GPIOS + various temps (VD, VA, ITEMP, VPV, VMV, VRES)

    // Run internal pull-down vs pull-up to see if there's open wire
    // printf("open wire: \n");
    adBms6830_Adax(AUX_OW_ON, PUP_UP, AUX_ALL);
    bms_mDelay(5); // adbms_transmit_poll(PLAUX1);
    bms_readAuxVoltages(true);

    adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
    bms_mDelay(5); // adbms_transmit_poll(PLAUX1); TODO this doesn't settle when it's hot
    bms_readAuxVoltagesAll();
    bms_print_aux_all(false);

    // TODO compare values
    #if 0
    #define BMS_AUX_OW_DELTA (1.0f) // TODO calcs
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int aux = 0; aux < TOTAL_AUX; aux++)
        {
            bool set = fabsf(data.aux_voltages_parsed[ic][aux] - data.aux_voltages_ow[ic][aux]) >= BMS_AUX_OW_DELTA;
            bms_set_fault_aux(ic, aux, BMS_ERROR_AUX_OW, set);
        }
    }
    // TODO NULL values in case of open-wire and exit state
    // if open-wire is okay, use values read from bms_readAuxVoltagesAll()
    // if else, null it and do shit
    #endif

#if 0
    // TODO convert volts to C and check temp min/max
    #define BMS_TEMP_MIN (-40.0f)
    #define BMS_TEMP_MAX (60.0f)
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int aux = 0; aux < TOTAL_AUX; aux++)
        {
            set = data.aux_v[ic][aux] < BMS_TEMP_MIN;
            if (set) bms_error("[FAULT]: [IC%d]: BMS_ERROR_AUX_UNDERTEMP: %.3f\n", ic, data.vref2[ic]);
            bms_set_fault_aux(ic, aux, BMS_ERROR_AUX_UNDERTEMP, set);
            set = data.aux_v[ic][aux] > BMS_TEMP_MAX;
            bms_set_fault_aux(ic, aux, BMS_ERROR_AUX_OVERTEMP, set);
        }
    }
#endif
}

static void bms_aux_voltages_check(void)
{
    // Sanity check va, vd, etc
    // IC[0]: vmv: -0.00 vpv: 11.70 vd: 3.03 va: 5.09 vref2: 3.00 itmp: 26.08
    bool set;

    // TODO check VMV and VPV

    // Va
    // Analog power supply voltage = voltage at the VREG pin.
    // Analog power supply voltage = VA × 150 μV + 1.5 V.
    // The value of VA is set by external components and must be in the range of 4.5 V to 5.5 V for normal operation.
    #define BMS_VA_MIN (4.5f)
    #define BMS_VA_MAX (5.5f)
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        set = data.va[ic] < BMS_VA_MIN;
        BMS_SET_FAULT_DEBUG(BMS_ERROR_VA_UV, data.va[ic]);

        set = data.va[ic] > BMS_VA_MAX;
        BMS_SET_FAULT_DEBUG(BMS_ERROR_VA_OV, data.va[ic]);
    }

    // Vd
    // digital power supply voltage
    // must be within 2.7 V to 3.6 V.
    #define BMS_VD_MIN (2.7f)
    #define BMS_VD_MAX (3.6f)
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        set = data.vd[ic] < BMS_VD_MIN;
        BMS_SET_FAULT_DEBUG(BMS_ERROR_VD_UV, data.vd[ic]);

        set = data.vd[ic] > BMS_VD_MAX;
        BMS_SET_FAULT_DEBUG(BMS_ERROR_VD_OV, data.vd[ic]);
    }

    // VREF2
    // Normal range is within 2.988 V to 3.012 V considering data sheet limits, thermal hysteresis, and long-term drift
    #define BMS_VREF2_MIN (2.988f)
    #define BMS_VREF2_MAX (3.012f)
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        set = data.vref2[ic] < BMS_VREF2_MIN || data.vref2[ic] > BMS_VREF2_MAX;
        BMS_SET_FAULT_DEBUG(BMS_ERROR_VREF2, data.vref2[ic]);
    }

    // ITMP: Internal Die temperature
    #define BMS_ITMP_MIN  (0.0f) // 32F
    #define BMS_ITMP_MAX (40.0f) // 104F
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        set = data.itmp[ic] < BMS_ITMP_MIN;
        BMS_SET_FAULT_DEBUG(BMS_ERROR_ITMP_UT, data.itmp[ic]);

        set = data.itmp[ic] > BMS_ITMP_MAX;
        BMS_SET_FAULT_DEBUG(BMS_ERROR_ITMP_OT, data.itmp[ic]);
    }
}

static void bms_aux_temps_check(void)
{
    bool set;

    // Absolute min/max for discharge
    #define BMS_AUX_TEMP_MIN (-40.0f) // TODO calcs
    #define BMS_AUX_TEMP_MAX (60.0f) // TODO calcs
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int aux = 0; aux < TOTAL_AUX; aux++)
        {
            // TODO convert to C and state uv/ov
            // set = data.aux_voltages_parsed[ic][aux] < BMS_AUX_TEMP_MIN;
            // bms_set_fault_aux(ic, aux, BMS_ERROR_AUX_UV, set);
        }
    }
}

static void bms_read_temps(void)
{
    // 1. Check open wire
    // 2. Check operating voltages
    // 3. Check temperatures
    bms_aux_ow_check();
    // Run voltage checks after open-wire in case results were trash due to ow
    bms_aux_voltages_check();
    bms_aux_temps_check();
}

static void bms_send_temps(void)
{    
    #if 0
    // send raw, module min/max
    uint16_t max_temps[TOTAL_AD68] = {0};
    uint16_t min_temps[TOTAL_AD68] = {0};
    uint16_t avg_temps[TOTAL_AD68] = {0};
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        uint16_t max_temp = data.aux_voltages_raw[ic][0];
        uint16_t min_temp = data.aux_voltages_raw[ic][0];
        uint16_t avg_temp = 0;
        for (int therm = 0; therm < TOTAL_AUX; therm++)
        {
            uint16_t temp = data.aux_voltages_raw[ic][therm]; // TODO convert to C ?
            max_temp = MAX(temp, max_temp);
            min_temp = MIN(temp, min_temp);
            avg_temp += temp;
        }
        avg_temp /= TOTAL_AUX;
        min_temps[ic] = min_temp;
        max_temps[ic] = max_temp;
        avg_temps[ic] = avg_temp;

        // 10 thermistors each,
        // SEND_MOD_CELL_TEMP_RAW(cell_temps[ic][0], min_temp, avg_temp);
        // SEND_MOD_CELL_TEMP_MAX(max_temp, min_temp, avg_temp);
    }
    // SEND_MOD_CELL_TEMP_AVG(max_temps[0], max_temps[0], max_temps[0], max_temps[0]);
    #endif
}
