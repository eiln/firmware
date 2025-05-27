
#include "main.h"
#include "adbms/adbms.h"

// AUX GPIO / TEMPS

#if 0

Cell Temps
max           -  60 C - open SDC/full shutdown
min charge    -   0 C - prohibit charging
min discharge - -40 C - prohibit discharging

#endif

#define CELL_TEMP_MAX_C 60.0

static void bms_read_temps(void);
static void bms_check_temps(void);
static void bms_send_temps(void);

void bms_monitor_temps(void)
{
    bms_read_temps();
    bms_send_temps();
    bms_check_temps();
}

static void bms_print_temps(void)
{
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        printf("IC[%d]: vmv: %.2f vpv: %.2f vd: %.2f va: %.2f vref2: %.2f itmp: %.2f\n", ic, bms.vmv[ic], bms.vpv[ic], bms.vd[ic], bms.va[ic], bms.vref2[ic], bms.itmp[ic]);
    }
}

static void bms_read_temps(void)
{
    // AUX_ALL includes 10 GPIOS + various temps (VD, VA, ITEMP, VPV, VMV, VRES)
    adBms6830_Adax(AUX_OW_OFF, PUP_DOWN, AUX_ALL);
    adbms_transmit_poll(PLAUX1);
    bms_readAuxVoltages();
    bms_print_temps();
    // read temps and status at the same time
    // since it's done by the same GPIO
    //bms_delayMsActive(10);
}

static void bms_check_temps(void)
{
    #if 0
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int therm = 0; therm < TOTAL_AUX; therm++)
        {
            uint16_t temp = bms.aux_temps_parsed[ic][therm]; // TODO convert to C ?
            // check unreasonable values & threshold values (too low or too high)
            // determine thresholds
            // 60C or datasheet (EV 3.1)
            if (temp < BMS_MOD_TEMP_MIN || temp > BMS_MOD_TEMP_MAX)
            {
                // send emergency CAN (high priority)
                printf("Module %d therm %d exceeded temp: %d\n", ic, therm, cell_temps[ic][therm]);
                // send temps before
                // pull SDC
                // setFault(ID_HEATSINK_THERMISTOR_FAULT, can_data.orion_errors.heatsink_thermistor);
                // setFault(ID_THERMISTOR_FAULT, can_data.orion_errors.thermistor);
            }
        }
    }
    // call cooling?
    // send warning for medium threshold
    #endif
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
        uint16_t max_temp = bms.aux_voltages_raw[ic][0];
        uint16_t min_temp = bms.aux_voltages_raw[ic][0];
        uint16_t avg_temp = 0;
        for (int therm = 0; therm < TOTAL_AUX; therm++)
        {
            uint16_t temp = bms.aux_voltages_raw[ic][therm]; // TODO convert to C ?
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
