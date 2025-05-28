
void charger_elcon_start(void)
{
    monitor_task();
    // discharge all to some threshold
    // start elcon charger CAN command
    // if can fails -> pull SDC

    //user_charge_current_request = 10;
    charge_current_req = MIN(can_data.orion_info.pack_ccl, user_charge_current_request);

    //user_charge_voltage_request = 314;
    charge_voltage_req = MIN(user_charge_voltage_request, MAX_VOLT); // Hard limit, don't overcharge
    charge_voltage_req *= 10;
    charge_current_req *= 10;

    // Swap endianess
    charge_voltage_req = ((charge_voltage_req & 0x00FF) << 8) | ((charge_voltage_req >> 8) & 0xFF);
    charge_current_req = ((charge_current_req & 0x00FF) << 8) | ((charge_current_req >> 8) & 0xFF);

    // every 1s, minimum every 5s
    SEND_ELCON_CHARGER_COMMAND(charge_voltage_req, charge_current_req, 1);
    bms->state = BMS_STATE_CHARGING;
}

void charger_elcon_stop(void)
{
    // stop immediately
    // send elcon CAN stop command
    // if can fails -> pull SDC
}

void charge(void)
{
    charger_elcon_start();

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

#if 0

Another charging strategy with lower charging time
is the constant-current constant-voltage (CC-CV) method. In
this method, in the first stages, a higher charging current is
used and when the battery terminal voltages hit the threshold
value, the current is reduced and a constant voltage is used to
do the final charging of the battery [8].

balancing: over 4.0


current required for a 1-hour discharge is described as 1C
look at cell C rate
determine approprate resistor

resistor = 15 ohm
cell v = variable (e.g 3.0 V)
current = v / r = 0.2 A
c rate = 1C = 1 C

estimate time for remaining cells to reach full charge
assume rate of other cells is constant
time remaining = (v max - v current) / rate
time remaining = x seconds

c time remaining = (v max - v current) / c rate
c time remaining < time remaining
percent = ct / t
pwm at percent

discharge emergency loop:

#endif
