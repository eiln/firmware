
#include "main.h"
#include "adbms_mcu.h"
#include "adbms6830.h"
#include "adbms_cmdlist.h"

#define MAX_DELTA       0.10 // V
#define BALANCING_MIN_V 3.99 // V

void bms_balance_cells(void)
{
    uint8_t pwm[TOTAL_AD68][TOTAL_CELL] = {0};

    int16_t min_volts = bms.cell_voltages_raw[0][0];
    int16_t max_volts = bms.cell_voltages_raw[0][0];
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int cell = 0; cell < TOTAL_CELL; cell++)
        {
            int16_t volts = bms.cell_voltages_raw[ic][cell];
            min_volts = MIN(volts, min_volts);
            max_volts = MIN(volts, max_volts);
        }
    }

    // TODO avoid float comparisons
    // output is offsetted negative so have to use float I believe
    float max_v = getVoltage(max_volts);
    float min_v = getVoltage(max_volts);
    if (min_v >= BALANCING_MIN_V)
    {
        for (int ic = 0; ic < TOTAL_AD68; ic++)
        {
            for (int cell = 0; cell < TOTAL_CELL; cell++)
            {
                float v = getVoltage(bms.cell_voltages_raw[ic][cell]);
                if (v >= BALANCING_MIN_V && (v - min_v) >= MAX_DELTA)
                {
                    pwm[ic][cell] = 0b0011; // TODO calculate duty
                }
            }
        }
    }

    bms_startDischarge(pwm);
}

void bms_monitor_charge(void)
{
    bms_monitor_cells();
    bms_monitor_temps();
    bms_balance_cells();
}
