
#include "main.h"
#include "adbms/adbms.h"
#include "bms_common.h"

#define BALANCE_DELTA     0.02f    // 20 mV threshold to trigger balancing
#define BALANCING_MIN_V   3.80f    // Do not balance below this voltage
#define RAMP_ALPHA        80.0f    // Steepness of exponential ramp
// 60–80 for moderate curve
// 100+ for aggressive balancing
// TODO tune RAMP_ALPHA dynamically based on temperature or SoH

#if 0

| Parameter            | Example Safety Window                          |       |                     |
| -------------------- | ---------------------------------------------- | ----- | ------------------- |
| **Cell voltage**     | 3.6 V ≤ Vcell ≤ 4.1 V                          |       |                     |
| **Pack current**     |                                                | Ipack | < 2–5 A (near idle) |
| **Cell temperature** | 15 °C ≤ T ≤ 45 °C                              |       |                     |
| **Vehicle state**    | Not driving, not charging (idle or soft sleep) |       |                     |
| **Cell delta-V**     | ≥ 10–30 mV between max and min                 |       |                     |
| **SoC**              | Often limited to mid-range SoC (e.g. 20–80%)   |       |                     |

#endif

static float32_t fast_expf(float x) {
    // Approximate exp(x) for x in small range
    x = 1.0f + x / 256.0f;
    x *= x; x *= x; x *= x; x *= x;
    x *= x; x *= x; x *= x; x *= x;
    return x;
}

static float32_t calc_cell_pwm(float32_t cell_v, float32_t max_volts)
{
    float32_t vdelta = max_volts - cell_v;
    float32_t duty = 0.0f;

    // Safety check: don't balance under-voltage cells or too-small delta
    if (vdelta < BALANCE_DELTA || cell_v < BALANCING_MIN_V) {
        duty = 0.0f;
    }
    else if (vdelta >= 0.05f) {
        duty = 1.0f;
    }
    else {
        float32_t effective_delta = vdelta - BALANCE_DELTA;
        duty = 1.0f - fast_expf(-RAMP_ALPHA * effective_delta);
        duty = clampf(duty, 0.0f, 1.0f);
    }

    return duty;
}

void bms_cell_balance_task(void)
{
    float32_t duty;
    uint8_t pwm[TOTAL_AD68][TOTAL_CELL] = {0};
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int cell = 0; cell < TOTAL_CELL; cell++)
        {
            duty = calc_cell_pwm(data.cell_v_c[ic][cell], bms.mod_vstats[ic].max);
            pwm[ic][cell] = (0b1111 * duty); // TODO calculate duty
        }
    }

    // bms_startDischarge(pwm);
}
