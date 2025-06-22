
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "soc/ekf.h"

#define Q_CAPACITY_MAH 2000.0f // Battery capacity
#define R_INTERNAL_OHM 0.05f   // Internal resistance
#define Qp 1e-5f			   // Process noise covariance
#define Rm 1e-3f			   // Measurement noise covariance

// ---- OCV-SOC Lookup Table ----
#define OCV_TABLE_SIZE 19
static const float soc_table[OCV_TABLE_SIZE] = {0.000, 0.015, 0.042, 0.077, 0.133, 0.194, 0.254, 0.321, 0.402, 0.493,
												0.601, 0.700, 0.785, 0.869, 0.911, 0.941, 0.967, 0.988, 1.000};
static const float voltage_table[OCV_TABLE_SIZE] = {1.07, 1.30, 1.58, 1.85, 2.05, 2.20, 2.36, 2.49, 2.62, 2.81,
													2.97, 3.09, 3.19, 3.30, 3.36, 3.40, 3.43, 3.48, 3.61};

// ---- Interpolation for OCV ----
static inline float interpolate_ocv(float soc)
{
	if (soc <= soc_table[0])
		return voltage_table[0];
	if (soc >= soc_table[OCV_TABLE_SIZE - 1])
		return voltage_table[OCV_TABLE_SIZE - 1];

	for (int i = 0; i < OCV_TABLE_SIZE - 1; ++i) {
		if (soc >= soc_table[i] && soc <= soc_table[i + 1]) {
			float t = (soc - soc_table[i]) / (soc_table[i + 1] - soc_table[i]);
			return voltage_table[i] + t * (voltage_table[i + 1] - voltage_table[i]);
		}
	}
	return 0.0f; // fallback
}

// ---- Numerical Derivative for dV/dSoC ----
static inline float df_dSoC(float soc)
{
	float delta = 0.001f;
	float v_plus = interpolate_ocv(fminf(soc + delta, 1.0f));
	float v_minus = interpolate_ocv(fmaxf(soc - delta, 0.0f));
	return (v_plus - v_minus) / (2.0f * delta);
}

// ---- EKF Predict Step ----
void ekf_predict(ekf_state_t *ekf, float current_ma, float dt_sec)
{
	float delta_soc = -(current_ma * dt_sec) / (3600.0f * Q_CAPACITY_MAH);
	ekf->soc += delta_soc;
	ekf->P += Qp;
}

// ---- EKF Update Step ----
void ekf_update(ekf_state_t *ekf, float voltage_meas, float current_ma)
{
	float V_pred = interpolate_ocv(ekf->soc) + R_INTERNAL_OHM * current_ma;
	float H = df_dSoC(ekf->soc);

	float K = ekf->P * H / (H * H * ekf->P + Rm);
	ekf->soc = ekf->soc + K * (voltage_meas - V_pred);
	ekf->P = (1.0f - K * H) * ekf->P;

	// Clamp SoC
	if (ekf->soc > 1.0f)
		ekf->soc = 1.0f;
	if (ekf->soc < 0.0f)
		ekf->soc = 0.0f;
}

#define SOC_PACK_CURRENT_IDLE (50E-3f) // 50mA

void ekf_estimate_init(ekf_state_t *ekf, float voltage_meas, float current_ma)
{
	float soc;
	if (voltage_meas > 4.18f && current_ma < SOC_PACK_CURRENT_IDLE) {
		soc = 1.0f;
	} else if (voltage_meas < 2.90f && current_ma < SOC_PACK_CURRENT_IDLE) {
		soc = 0.0f;
	} else {
		// measure OCV voltage at rest (no current flow)
		// TODO read from eeprom
		// soc = ocv_lookup(V_idle, T);
		// soc = interpolate_ocv(voltage_meas);
		soc = 1.0f;
	}

	ekf->soc = soc;
	ekf->P = 1e-3; // Set high uncertainty initially
}
