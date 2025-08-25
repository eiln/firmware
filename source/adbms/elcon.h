
#define CELL_C_RATE (4.5f) // 1C, Amps
#define CHARGER_CCL_MAX (13.5f)   // 3C (4.5*3)
#define CHARGER_CVL_MAX (596.4f) // Pack voltage

static void elcon_send_charge_request(float voltage_req, float current_req, bool charge)
{
    current_req = MIN(current_req, CHARGER_CCL_MAX);
    voltage_req = MIN(voltage_req, CHARGER_CVL_MAX);

    current_req *= 10.0f; // Elcon expects 3201 for 320.1 V
    voltage_req *= 10.0f;

    // Swap endianess
    voltage_req = ((voltage_req & 0x00FF) << 8) | ((voltage_req >> 8) & 0xFF);
    current_req = ((current_req & 0x00FF) << 8) | ((current_req >> 8) & 0xFF);

    // Every 1s, minimum every 5s
    // 0 = charge, 1 = dont charge
    SEND_ELCON_CHARGER_COMMAND(voltage_req, current_req, !charge);
}

static void elcon_send_stop_request(float voltage_req)
{
    elcon_send_charge_request(voltage_req, 0.0f, false);
}

volatile uint32_t elcon_last_status;

static bool elcon_charger_fault_status(CAN_FRAME &frame)
{
    float charge_voltage = frame.charge_voltage * 0.1f;
    float charge_current = frame.charge_current * 0.1f;

    uint32_t now = bms_getTick();
    // set elcon_last_status as start
    // 2. The charger send broadcast message (Message 2) at intervals of 1s
    if (now - elcon_last_status > 5) // 5 second timeout
    {
        elcon_last_status = now;
        return true;
    }
    elcon_last_status = now;

    if (frame.hw_fail || frame.temp_fail || frame.input_v_fail || frame.startup_fail || frame.communication_fail)
    {
        return true;
    }

    if (charge_voltage > CHARGER_CVL_MAX || CHARGER_CCL_MAX > CHARGER_CCL_MAX)
    {
        return true;
    }

    return false;
}

static void elcon_charger_status(CAN_FRAME &frame)
{
    bool set = elcon_process_charger_status(frame);
    bms_set_fault_global(BMS_GLOBAL_ERROR_CHARGER, set);
}

void charger_elcon_stop(void)
{
    // stop immediately
    // send elcon CAN stop command
    // if can fails -> pull SDC
}

#define CELL_BALANCE_THRESHOLD (4.0f) // V

void charge(void)
{
    elcon_send_charge_request(CHARGER_CVL_MAX, CHARGER_CCL_MAX, true);

    // discharge rate: voltage / (30 ohm) = 3.0 / (30 ohm) = 0.1A

    // loop:
    // monitor_task()
    // if one cell charges at faster rate
        // discharge with calcauted pwm rate
        // ask about cell discharge rate

    // if cell is near upper charge threshold (4.0)
        // charger_elcon_stop();
        // discharge until cell is below upper charge threshold
        // monitor_task()
        // charger_elcon_start();

    // if cell is not discharging for some reason, and it's close to 4.2:
        // charger_elcon_stop();
        // pull SDC

    // send elcon CAN heartbeat
}
