
#ifndef __BMS_FAULTS_H__
#define __BMS_FAULTS_H__

#include <assert.h>
#include "stdint.h"
#include "stdbool.h"

typedef enum
{
    BMS_ERROR_SID = 0,  // Device ID
    BMS_ERROR_RXPEC,    // RX PEC mismatch
    BMS_ERROR_CONFIG,   // Config TX failed
    BMS_ERROR_POLL_TIMEOUT, // Poll Timeout

    BMS_ERROR_VPV,      // V+ to V−
    BMS_ERROR_VMV,      // S1N to V−
    BMS_ERROR_VA_UV,    // Analog power undervoltage
    BMS_ERROR_VA_OV,    // Analog power overvoltage
    BMS_ERROR_VD_UV,    // Digital power undervoltage
    BMS_ERROR_VD_OV,    // Digital power overvoltage
    BMS_ERROR_VREG,     // Regulated Power
    BMS_ERROR_VREF2,    // Vref2 for thermistors
    BMS_ERROR_ITMP_UT,  // Internal die temperature under temperature
    BMS_ERROR_ITMP_OT,  // Internal die temperature over temperature

    BMS_ERROR_CELL_OW,    // Cell open-wire
    BMS_ERROR_CELL_UV,    // Cell undervoltage
    BMS_ERROR_CELL_OV,    // Cell overvoltage
    BMS_ERROR_CELL_REDUN, // Cell redundant measurement

    BMS_ERROR_AUX_OW,     // AUX open-wire
    BMS_ERROR_AUX_UT,     // AUX under temperature
    BMS_ERROR_AUX_OT,     // AUX over temperature
    BMS_ERROR_AUX_REDUN,  // AUX over temperature

    BMS_ERROR_COUNT,
} bms_error_t;
static_assert(BMS_ERROR_COUNT < 32); // since packing in u32

typedef enum
{
    BMS_GLOBAL_ERROR_CAN = 0,
    BMS_GLOBAL_ERROR_CHARGER,
    BMS_GLOBAL_ERROR_COUNT,
} bms_global_error_t;
static_assert(BMS_GLOBAL_ERROR_COUNT < 32); // since packing in u32

#define BMS_GET_ERROR_MASK(field) (1 << (field))

void bms_set_fault_global(bms_global_error_t field, bool set);
bool bms_global_fault(bms_global_error_t field);

void bms_set_fault(int ic, bms_error_t field, bool set);
void bms_set_fault_all(bms_error_t field, bool set);
uint32_t bms_pack_faults(bms_error_t field);
uint32_t bms_get_fault_duration(int ic, bms_error_t field);
void bms_set_fault_aux(int ic, int aux, bms_error_t field, bool set);

#endif // __BMS_FAULTS_H__
