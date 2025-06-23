
#include "soc/ekf.h"
#include "soc/soc.h"
#include "common/freertos/freertos.h"
#include "bms_common.h"

#define PACK_INT_RESISTANCE (1.0f) // Ohms (TODO)

static bool can_i_trust_pack_current(bms_t *bms)
{
	float pack_current = 0.0f; // ADC raw
	bool set = (pack_current <= -1000.0f || pack_current >= 1000.0f);
	bms_set_fault_global(BMS_ERROR_ISENSE, set);
	if (!set) {
		// Filter after sanity checking
		bms->pack_current = ema_filter(pack_current, bms->pack_current, 0.50f);
		return true;
	}
	return false;
}

static bool can_i_trust_pack_voltage(bms_t *bms)
{
	return (bms->state == BMS_STATE_CONNECTED) && ((getTick() - bms->connect_time) >= SOC_INIT_DELAY_MS) && bms->cells_ok;
}

static bool collect_pack_data(bms_t *bms)
{
	bool current_ok = can_i_trust_pack_current(bms);
	bool voltage_ok = can_i_trust_pack_voltage(bms);

	if (!current_ok) {
		// Estimate current from requested torque
		// TODO get ecu torque CAN message
		// torque = kt * current
		// TODO check ecu CAN stale
		float requested_torque = 9.8; // N-m
		float kt = 5.0f;			  // TODO get
		float I_est = requested_torque / kt;
		bms->pack_current = I_est;
		current_ok = true;
	}
	if (!voltage_ok && current_ok && bms->ekf_initialized) {
		// Lost adbms connection, but try our best to estimate voltage using current draw
		float V_ocv = bms->pack_voltage; // Last voltage
		float V_est = V_ocv - bms->pack_current * PACK_INT_RESISTANCE;
		bms->pack_voltage = V_est;
		voltage_ok = true;
	}

	// TODO send current/voltage over CAN (current: f32, voltage: f32)

	return current_ok && voltage_ok;
}

static void print_soc_status(bms_t *bms)
{
	printf("soc available: %d", bms->soc_available);
}

void soc_ekf_update(bms_t *bms)
{
	bool soc_available = collect_pack_data(bms);
	if (soc_available) {
		float voltage_meas = bms->pack_voltage;
		float current_ma = bms->pack_current * 1E-3f;
		if (!bms->ekf_initialized) {
			ekf_estimate_init(&bms->ekf, voltage_meas, current_ma);
			bms->ekf_initialized = true;
		} else {
			ekf_predict(&bms->ekf, current_ma, SOC_EKF_STEP_DT);
			ekf_update(&bms->ekf, voltage_meas, current_ma);
		}
	}
	bms->soc_available = soc_available;
	// print_soc_status(bms);
	// TODO: send SOC data (1 bit available, 1 bit: 32-bit: float 0.0-1.0)
}
