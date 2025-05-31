#include "main.h"
#include "adbms/adbms.h"

bool bms_charge_requested(void)
{
    bool charger_connected = PHAL_readGPIO(CHARGE_ENABLED_PORT, CHARGE_ENABLED_PIN);
    bool daq_connected = false; // TODO daqapp
    return charger_connected && daq_connected;
}

static bool bms_can_charge(void)
{
    // Bare minimum checks to see if charging can continue

    // 1. Check BMS connection
    if (bms_pack_faults(BMS_ERROR_SID) || bms_pack_faults(BMS_ERROR_RXPEC) || bms_pack_faults(BMS_ERROR_CONFIG) || bms_pack_faults(BMS_ERROR_POLL_TIMEOUT))
    {
        bms_error("[ERROR]: BMS Connection fault! Cannot charge!\n");
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
    if (bmsmaster.state == BMS_STATE_CHARGING && bms_global_fault(BMS_GLOBAL_ERROR_CHARGER))
    {
        bms_error("[ERROR]: Charger fault! Cannot charge!\n");
        return false;
    }

    return true;
}

static int charger_fail_count = 0;

static void bms_pull_sdc(void)
{
    // PHAL_writeGPIO(SPI_CS_PORT, SPI_CS_PIN, 0);
    // TODO enter fatal error state
}

static void elcon_charger_stop(void)
{
    ;
}

static void elcon_charger_start(void)
{
    ;
}

void bms_cell_balance_task(void);

void bms_charge_task(void)
{
    if (!bms_can_charge())
    {
        charger_fail_count++;
    }
    else
    {
        charger_fail_count = 0;
    }

    // Assumes task is called in charge requested state
    if (charger_fail_count >= 5)
    {
        bms_error("[ERROR]: Charger fault unresolved! Disconnecting from charger\n");
        elcon_charger_stop();
        bms_pull_sdc();
        return;
    }
    if (charger_fail_count)
    {
        bms_error("[ERROR]: Retrying charger fault...\n");
        elcon_charger_stop();
        return;
    }

    bmsmaster.state = BMS_STATE_CHARGING;

    bms_cell_balance_task();
    elcon_charger_start();
    // elcon_send_charge_request(CHARGER_CVL_MAX, CHARGER_CCL_MAX, true);
    // discharge rate: voltage / (30 ohm) = 3.0 / (30 ohm) = 0.1A
}
