
#ifndef __BMS_FAULTS_H__
#define __BMS_FAULTS_H__

#include <assert.h>
#include "stdint.h"
#include "stdbool.h"

typedef enum
{
    BMS_ERROR_CAN = 0,  // CAN RX/TX
    BMS_ERROR_ELCON,
    BMS_ERROR_CHARGER_PORT,

    BMS_ERROR_SID,  // Device ID
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
    BMS_ERROR_PACK_WEAK, // Under/Overvoltage module

    BMS_ERROR_CELL_OW,    // Cell open-wire
    BMS_ERROR_CELL_UV,    // Cell undervoltage
    BMS_ERROR_CELL_OV,    // Cell overvoltage
    BMS_ERROR_CELL_REDUN, // Cell redundant measurement
    BMS_ERROR_CELL_WEAK,  // Under/Overvoltage cell in module

    BMS_ERROR_AUX_OW,     // AUX open-wire
    BMS_ERROR_AUX_UT,     // AUX under temperature
    BMS_ERROR_AUX_OT,     // AUX over temperature
    BMS_ERROR_AUX_REDUN,  // AUX over temperature

    BMS_ERROR_COUNT,
} bms_error_t;
static_assert(BMS_ERROR_COUNT < 32); // since packing in u32

#define BMS_GET_ERROR_MASK(field) (1 << (field))

void bms_set_fault(int ic, bms_error_t field, bool set);
void bms_set_fault_all(bms_error_t field, bool set);
uint32_t bms_pack_faults(bms_error_t field);
uint32_t bms_get_fault_duration(int ic, bms_error_t field);
bool bms_any_fault(bms_error_t field);

void bms_set_fault_cell(int ic, int cell, bms_error_t field, bool set);
void bms_set_fault_aux(int ic, int aux, bms_error_t field, bool set);
bool bms_any_cell_fault(bms_error_t field);
bool bms_any_aux_fault(bms_error_t field);

void bms_set_fault_global(bms_error_t field, bool set);
bool bms_global_fault(bms_error_t field);

void bms_error_handler(void);

#define BMS_SET_FAULT_DEBUG(field, var)\
    if (set) bms_error("[FAULT]: [IC%d]: " #field ": %.3f\n", ic, var);\
    bms_set_fault(ic, field, set);

#define BMS_SET_FAULT_CELL_DEBUG(cell, field, var)\
    if (set) bms_error("[FAULT]: [IC%d]: [CELL%2d]: " #field ": %.3f\n", ic, cell, var);\
    bms_set_fault_cell(ic, cell, field, set);

typedef struct {
    uint64_t internal_comms: 1;    // ADBMS ISOSPI
    uint64_t external_comms: 1;    // Vehicle CAN/Elcon CAN
    uint64_t internal_hardware: 1; // Undervoltage/Overvoltage/etc
    uint64_t hv_isolation: 1;      // IMD fault
    uint64_t input_psu: 1;         // Power rails

    uint64_t cell_open_wire: 1;
    uint64_t cell_uv: 1;
    uint64_t cell_ov: 1;
    uint64_t weak_cell: 1;
    uint64_t weak_pack: 1;

    uint64_t thermistor_open_wire: 1;
    uint64_t thermistor_bad: 1;
    uint64_t pack_overheat: 1;
} bms_errors_t;
static_assert(sizeof(bms_errors_t) == sizeof(uint64_t));

#endif // __BMS_FAULTS_H__
