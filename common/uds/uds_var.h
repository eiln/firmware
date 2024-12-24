

struct daq_variable {
    uint64_t (*read_fn)();
    void (*send_fn)();
    uint32_t rate;
    uint32_t minimum_post;
    uint64_t last;
    uint32_t last_tick;
};

void post_daq_variable(struct daq_variable *dvar)
{
    uint64_t value = dvar->read_fn();
    if ((value != dvar->last) ||
       (dvar->minimum_post && (getTick() - dvar->last_tick > dvar->minimum_post)))
    {
        dvar->send_fn();
        dvar->last_tick = getTick();
    }
    dvar->last = value;
}

SEND_REAR_WHEEL_SPEEDS(left_speed_mc_, right_speed_mc_, left_speed_sensor_, right_speed_sensor_)

void send_rear_wheel_speeds(void)
{
    CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_REAR_WHEEL_SPEEDS, .DLC=DLC_REAR_WHEEL_SPEEDS, .IDE=1};
    CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;
    data_a->rear_wheel_speeds.left_speed_mc = car.motor_l.rpm;
    data_a->rear_wheel_speeds.right_speed_mc = car.motor_r.rpm;
    data_a->rear_wheel_speeds.left_speed_sensor = car.motor_l.rpm;
    data_a->rear_wheel_speeds.right_speed_sensor = car.motor_r.rpm;
    canTxSendToBack(&msg);
}

uint64_t load_sensor_read(void)
{
    CanParsedData_t data_a;
    data_a.rear_wheel_speeds.left_speed_mc = car.motor_l.rpm;
    data_a.rear_wheel_speeds.right_speed_mc = car.motor_r.rpm;
    data_a.rear_wheel_speeds.left_speed_sensor = car.motor_l.rpm;
    data_a.rear_wheel_speeds.right_speed_sensor = car.motor_r.rpm;
    return data_a.uds_command_main_module.payload;
}

struct daq_variable dvar_load_sensor = {
    .send_fn = &load_sensor_send,
    .read_fn = &load_sensor_read,
    .minimum_post = 2000,
    .last_tick = 0,
};

void post_daq_variable_load_sensor(void)
{
    post_daq_variable(&dvar_load_sensor);
}
