
void update_thermal_model(float I, float T_measured, float T_ambient, float dt) {
    static float T_pred = T_measured;  // init on boot
    const float R_internal = 0.015f;   // ohms
    const float R_th = 2.5f;           // °C/W
    const float C_th = 50.0f;          // J/°C

    float P_heat = I * I * R_internal;
    float dT = dt / C_th * (P_heat - (T_pred - T_ambient) / R_th);

    T_pred += dT;

    float error = T_measured - T_pred;

    if (error > 5.0f) {
        log_fault("Thermal deviation anomaly");
        trigger_alert();
    }
}

static void drop_therm(void)
{

}

static void therm_monitor(void)
{
    // One sensor anomaly (spike or drift) may cause a false positive
    float delta_T = fabs(T[i] - T[i+1]);
    if (T[i] > T_crit && delta_T < 3.0) {
        // Likely false alarm: isolated hot reading, no neighbor confirms it
    }

    // 1. Rate of Temperature Rise (dT/dt)
    // Trigger warning at:
    // dT/dt > 1.5 °C/s (early warning)
    // dT/dt > 5.0 °C/s (shutdown)

    // 2. Cell-to-Cell Thermal Gradient (ΔT)
    // Detects localized runaway BEFORE it propagates
    // |T_cell[i] - T_cell[i+1]| > 8–10 °C

    // 3. High Absolute Temperature Threshold
    // T > 60–70 °C: warning
    // T > 80 °C: hard shutdown
}

T_pred[i] = thermal_model(T_prev, I, Q, env);
error = T_measured[i] - T_pred[i];

if (error > threshold) flag_hotspot();
