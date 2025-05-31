
/* Faults */

#include "faults.h"
#include "adbms_conf.h"
#include "adbms_mcu.h"
#include "main.h"

uint32_t bms_get_fault_duration(int ic, bms_error_t field)
{
    uint32_t now = bms_getTick();
    if (bms.fault[ic] & BMS_GET_ERROR_MASK(field))
    {
        bms.last_fault_time[ic][field] = now;
        return bms.last_fault_time[ic][field] - bms.first_fault_time[ic][field];
    }
    return 0;
}

void bms_set_fault(int ic, bms_error_t field, bool set)
{
    uint32_t now = bms_getTick();
    uint32_t mask = BMS_GET_ERROR_MASK(field);
    if (set)
    {
        if (!(bms.fault[ic] & mask))
        {
            bms.first_fault_time[ic][field] = now;
        }
        else
        {
            // Fault already set, so it's been ongoing
            if (!bms.first_fault_time[ic][field])
                bms.first_fault_time[ic][field] = now;
        }
        bms.fault[ic] |= mask;
        bms.last_fault_time[ic][field] = now;
    }
    else
    {
        bms.fault[ic] &= ~mask;
        bms.first_fault_time[ic][field] = 0;
        bms.last_fault_time[ic][field] = 0;
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
        mask |= !!(bms.fault[ic] & BMS_GET_ERROR_MASK(field)) << ic;
    }
    return mask;
}

void bms_set_fault_cell(int ic, int cell, bms_error_t field, bool set)
{
    uint32_t mask = BMS_GET_ERROR_MASK(field);
    if (set)
    {
        bms.fault_cell[ic][cell] |= mask;
    }
    else
    {
        bms.fault_cell[ic][cell] &= ~mask;
    }
}

void bms_set_fault_aux(int ic, int aux, bms_error_t field, bool set)
{
    uint32_t mask = BMS_GET_ERROR_MASK(field);
    if (set)
    {
        bms.fault_aux[ic][aux] |= mask;
    }
    else
    {
        bms.fault_aux[ic][aux] &= ~mask;
    }
}


void bms_set_fault_global(bms_global_error_t field, bool set)
{
    uint32_t mask = BMS_GET_ERROR_MASK(field);
    if (set)
    {
        bms.fault_global |= mask;
    }
    else
    {
        bms.fault_global &= ~mask;
    }
}

bool bms_global_fault(bms_global_error_t field)
{
    return bms.fault_global & BMS_GET_ERROR_MASK(field);
}
