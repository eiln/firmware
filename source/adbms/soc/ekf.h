
#pragma once

typedef float float32_t;

typedef struct {
	float soc; // Estimated SoC [0.0 - 1.0]
	float P;   // Covariance
} ekf_state_t;

void ekf_predict(ekf_state_t *ekf, float current_ma, float dt_sec);
void ekf_update(ekf_state_t *ekf, float voltage_meas, float current_ma);
void ekf_estimate_init(ekf_state_t *ekf, float voltage_meas, float current_ma);
