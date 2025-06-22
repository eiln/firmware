
#pragma once
#include "main.h"

#define SOC_EKF_STEP_DT (10E-3f) // EKF runs at 100Hz or 10ms
#define SOC_INIT_DELAY_MS (500)	 // 500 ms, wait for N sampling cycles

void soc_ekf_update(bms_t *bms);
