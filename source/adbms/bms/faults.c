
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

static inline bool bms_any_fault(bms_error_t field)
{
    uint32_t mask = BMS_GET_ERROR_MASK(field);
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        if (bms.fault[ic] & mask)
        {
            return true;
        }
    }
    return false;
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

bool bms_any_cell_fault(bms_error_t field)
{
    uint32_t mask = BMS_GET_ERROR_MASK(field);
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int cell = 0; cell < TOTAL_CELL; cell++)
        {
            if (bms.fault_cell[ic][cell] & mask)
            {
                return true;
            }
        }
    }

    return false;
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

bool bms_any_aux_fault(bms_error_t field)
{
    uint32_t mask = BMS_GET_ERROR_MASK(field);
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int i = 0; i < TOTAL_AUX; i++)
        {
            if (bms.fault_aux[ic][i] & mask)
            {
                return true;
            }
        }
    }

    return false;
}

void bms_set_fault_global(bms_error_t field, bool set)
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

bool bms_global_fault(bms_error_t field)
{
    return bms.fault_global & BMS_GET_ERROR_MASK(field);
}

static bool is_any_error(void)
{
    if (bms.fault_global) return true;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        if (bms.fault[ic]) return true;
        for (int i = 0; i < TOTAL_CELL; i++)
        {
            if (bms.fault_cell[ic][i]) return true;
        }
        for (int i = 0; i < TOTAL_AUX; i++)
        {
            if (bms.fault_aux[ic][i]) return true;
        }
    }
    return false;
}

#define print_bms_fault(ic, field) do {\
    if (bms.fault[ic] & BMS_GET_ERROR_MASK(field))\
        printf("\t " #field " time: %d last: %d\n", bms_get_fault_duration(ic, field), bms.last_fault_time[ic][field]);\
} while (0);

#define print_bms_cell_fault(field) do {\
    if (bms.fault_cell[ic][cell] & BMS_GET_ERROR_MASK(field))\
        printf("\t [CELL%2d]" #field "\n", cell);\
} while (0);

#define print_bms_aux_fault(field) do {\
    if (bms.fault_aux[ic][i] & BMS_GET_ERROR_MASK(field))\
        printf("\t [AUX%2d]" #field "\n", i);\
} while (0);

static void print_faults(void)
{
    #ifdef ADBMS_DEBUG_PRINT
    if (!is_any_error()) return;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        printf("BMS Error IC[%d]: 0x%08x\n", ic, bms.fault[ic]);
        print_bms_fault(ic, BMS_ERROR_SID);
        print_bms_fault(ic, BMS_ERROR_RXPEC);
        print_bms_fault(ic, BMS_ERROR_CONFIG);
        print_bms_fault(ic, BMS_ERROR_POLL_TIMEOUT);
        print_bms_fault(ic, BMS_ERROR_VPV);
        print_bms_fault(ic, BMS_ERROR_VMV);
        print_bms_fault(ic, BMS_ERROR_VA_UV);
        print_bms_fault(ic, BMS_ERROR_VA_OV);
        print_bms_fault(ic, BMS_ERROR_VD_UV);
        print_bms_fault(ic, BMS_ERROR_VD_OV);
        print_bms_fault(ic, BMS_ERROR_VREG);
        print_bms_fault(ic, BMS_ERROR_VREF2);
        print_bms_fault(ic, BMS_ERROR_ITMP_UT);
        print_bms_fault(ic, BMS_ERROR_ITMP_OT);

        for (int cell = 0; cell < TOTAL_CELL; cell++)
        {
            print_bms_cell_fault(BMS_ERROR_CELL_OW);
            print_bms_cell_fault(BMS_ERROR_CELL_UV);
            print_bms_cell_fault(BMS_ERROR_CELL_OV);
            print_bms_cell_fault(BMS_ERROR_CELL_REDUN);
        }

        for (int i = 0; i < TOTAL_AUX; i++)
        {
            print_bms_aux_fault(BMS_ERROR_AUX_OW);
            print_bms_aux_fault(BMS_ERROR_AUX_UT);
            print_bms_aux_fault(BMS_ERROR_AUX_OT);
            print_bms_aux_fault(BMS_ERROR_AUX_REDUN);
        }
    }
    #endif // ADBMS_DEBUG_PRINT
}

void bms_error_handler(void)
{
    // Pack faults nicely into a single error message
    bms.errors.internal_comms = bms_any_fault(BMS_ERROR_SID) || bms_any_fault(BMS_ERROR_RXPEC) || bms_any_fault(BMS_ERROR_CONFIG);
    bms.errors.external_comms = bms_global_fault(BMS_ERROR_CAN); // TODO CAN error count
    bms.errors.internal_hardware = bms_any_cell_fault(BMS_ERROR_CELL_REDUN) || bms_any_aux_fault(BMS_ERROR_AUX_REDUN) || bms_any_fault(BMS_ERROR_VREG) || bms_any_fault(BMS_ERROR_VREF2);
    bms.errors.hv_isolation = 0; // TODO IMD GPIO
    bms.errors.input_psu = 0; // TODO ADC powerrail readings

    bms.errors.cell_open_wire = bms_any_cell_fault(BMS_ERROR_CELL_OW);
    bms.errors.cell_uv = bms_any_cell_fault(BMS_ERROR_CELL_UV);
    bms.errors.cell_ov = bms_any_cell_fault(BMS_ERROR_CELL_OV);

    bms.errors.thermistor_open_wire = bms_any_aux_fault(BMS_ERROR_AUX_OW);
    bms.errors.thermistor_bad = bms_any_cell_fault(BMS_ERROR_AUX_UT) || bms_any_cell_fault(BMS_ERROR_AUX_OT);
    bms.errors.pack_overheat = bms_any_cell_fault(BMS_ERROR_AUX_OT) || bms_any_fault(BMS_ERROR_ITMP_OT);

    print_faults();

    if (is_any_error())
    {
        PHAL_toggleGPIO(LED_PORT_RED, LED_PIN_RED);
    }
    else
    {
        PHAL_writeGPIO(LED_PORT_RED, LED_PIN_RED, 0);
    }
}
