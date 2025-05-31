#include "main.h"
#include "adbms/adbms.h"

static bool bms_can_charge(void)
{
    // Bare minimum checks to see if charging can continue
    // TODO add some debouncing to e.g. voltage

    // 0. Check if charger is connected
    bool charge = PHAL_readGPIO(CHARGE_ENABLED_PORT, CHARGE_ENABLED_PIN);
    if (!charge) return false;

    // 1. Check BMS connection
    if (bms_pack_faults(BMS_ERROR_SID) || bms_pack_faults(BMS_ERROR_RXPEC) || bms_pack_faults(BMS_ERROR_CONFIG) || bms_pack_faults(BMS_ERROR_POLL_TIMEOUT))
    {
        bms_error("[ERROR]: Connection fault! Cannot charge!\n");
        return false;
    }

    // 2. Check temperatures
    if (bms_pack_faults(BMS_ERROR_AUX_OW) || bms_pack_faults(BMS_ERROR_AUX_UT) || bms_pack_faults(BMS_ERROR_AUX_OT) || bms_pack_faults(BMS_ERROR_AUX_REDUN))
    {
        bms_error("[ERROR]: Temperature fault! Cannot charge!\n");
        return false;
    }

    // 3. Check voltages
    if (bms_pack_faults(BMS_ERROR_CELL_OW) || bms_pack_faults(BMS_ERROR_CELL_UV) || bms_pack_faults(BMS_ERROR_CELL_OV) || bms_pack_faults(BMS_ERROR_CELL_REDUN))
    {
        bms_error("[ERROR]: Cell fault! Cannot charge!\n");
        return false;
    }

    // 4. Check CAN communication
    if (bms_global_fault(BMS_GLOBAL_ERROR_CAN))
    {
        bms_error("[ERROR]: CAN fault! Cannot charge!\n");
        return false;
    }

    // 5. Check Elcon
    if (bms.state == BMS_STATE_CHARGING && bms_global_fault(BMS_GLOBAL_ERROR_CHARGER))
    {
        bms_error("[ERROR]: Charger fault! Cannot charge!\n");
        return false;
    }

    return true;
}



