
#include "bms_common.h"
#include "main.h"

#if 0
1. High SoC (>90-95%) → Limit or disable regen
Regen is tapered down linearly above 90% SoC
At 100%, regen is fully disabled
If cell voltage is near 4.20V, regen stops even before 100% SoC

2. Cold Battery (<15°C) → Heavily derated regen
Below 0°C: regen can be entirely blocked
Above 10-15°C: regen returns gradually
Battery preconditioning will actively heat the pack before fast driving or Supercharging to restore regen

3. Variable Regen Tuning Based on Drive Mode
In Sport or Track Mode:
    Regen limit is less conservative (until thermal constraints hit)
In Normal Mode:
    Regen is smoother and tapers early


Torque request is ramped to avoid sudden drop-off
Dashboard shows regen limit bar or faded regen icon
Battery heater used to enable regen in cold

🔋 At high SoC (≥98%), regen is almost entirely disabled regardless of temperature.
🧊 At cold temperatures (<0°C), regen is significantly limited or blocked to protect the battery.
✅ The maximum regen (100 A) is available in the optimal window: SoC < 80% and temperature between 20–30°C.

#endif

#include <math.h>

// SoC (%) breakpoints
static const float soc_table[] = {0.f, 80.f, 90.f, 95.f, 98.f, 100.f};  // 0-100%
static const float soc_limit[] = {100.f, 100.f, 80.f, 50.f, 10.f, 0.f}; // A

// Temperature (°C) breakpoints
static const float temp_table[] = {-20.f, -10.f, 0.f, 10.f, 25.f, 40.f}; // C
static const float temp_limit[] = {0.f, 10.f, 30.f, 60.f, 100.f, 90.f};  // A

static float interpolate(const float *x_table, const float *y_table, size_t len, float x)
{
    if (x <= x_table[0]) return y_table[0];
    if (x >= x_table[len - 1]) return y_table[len - 1];

    for (size_t i = 0; i < len - 1; ++i) {
        if (x >= x_table[i] && x <= x_table[i + 1]) {
            float x0 = x_table[i], x1 = x_table[i + 1];
            float y0 = y_table[i], y1 = y_table[i + 1];
            float slope = (y1 - y0) / (x1 - x0);
            return y0 + slope * (x - x0);
        }
    }
    return y_table[len - 1]; // Fallback
}

// Compute final regen limit (A)
static float compute_regen_limit(float soc, float temperature)
{
    float soc_limit_a  = interpolate(soc_table, soc_limit, sizeof(soc_table)/sizeof(float), soc);
    float temp_limit_a = interpolate(temp_table, temp_limit, sizeof(temp_table)/sizeof(float), temperature);
    return fminf(soc_limit_a, temp_limit_a); // A
}

void regen_update_ccl(bms_t *bms)
{
    float32_t ccl = compute_regen_limit(bms->ekf.soc, bms->pack_temperature);
    printf("regen ccl: %.3f\n", ccl);
}
