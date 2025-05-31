
/* Faults */

#include "faults.h"
#include "adbms_conf.h"
#include "adbms_mcu.h"
#include "main.h"

void bms_set_fault_global(bms_global_error_t field, bool set)
{
    uint32_t mask = BMS_GET_ERROR_MASK(field);
    if (set)
    {
        bmsmaster.fault_global |= mask;
    }
    else
    {
        bmsmaster.fault_global &= ~mask;
    }
}

uint32_t bms_get_fault_duration(int ic, bms_error_t field)
{
    uint32_t now = bms_getTick();
    if (bmsmaster.fault[ic] & BMS_GET_ERROR_MASK(field))
    {
        bmsmaster.last_fault_time[ic][field] = now;
        return bmsmaster.last_fault_time[ic][field] - bmsmaster.first_fault_time[ic][field];
    }
    return 0;
}

void bms_set_fault(int ic, bms_error_t field, bool set)
{
    uint32_t now = bms_getTick();
    uint32_t mask = BMS_GET_ERROR_MASK(field);
    if (set)
    {
        if (!(bmsmaster.fault[ic] & mask))
        {
            bmsmaster.first_fault_time[ic][field] = now;
        }
        else
        {
            // Fault already set, so it's been ongoing
            if (!bmsmaster.first_fault_time[ic][field])
                bmsmaster.first_fault_time[ic][field] = now;
        }
        bmsmaster.fault[ic] |= mask;
        bmsmaster.last_fault_time[ic][field] = now;
    }
    else
    {
        bmsmaster.fault[ic] &= ~mask;
        bmsmaster.first_fault_time[ic][field] = 0;
        bmsmaster.last_fault_time[ic][field] = 0;
    }
}

void bms_set_fault_all(bms_error_t field, bool set)
{
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        bms_set_fault(ic, field, set);
    }
}

uint32_t bms_pack_faults(bms_error_t field)
{
    uint32_t mask = 0;
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        mask |= !!(bmsmaster.fault[ic] & BMS_GET_ERROR_MASK(field)) << ic;
    }
    return mask;
}

void bms_set_fault_aux(int ic, int aux, bms_error_t field, bool set)
{
    uint32_t mask = BMS_GET_ERROR_MASK(field);
    if (set)
    {
        bmsmaster.fault_aux[ic][aux] |= mask;
    }
    else
    {
        bmsmaster.fault_aux[ic][aux] &= ~mask;
    }
}
