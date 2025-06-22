#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# --------------------------
# Battery Parameters
# --------------------------
Q_CAPACITY = 2000.0  # Battery capacity in mAh
R_INTERNAL = 0.05E-3    # Internal resistance in ohms
Qp = 1e-5            # Process noise covariance
Rm = 1e-3            # Measurement noise covariance

# --------------------------
# Custom OCV-SOC Curve
# --------------------------
n_capacity = np.array([-0.00, 0.06, 0.17, 0.31, 0.54, 0.78, 1.02, 1.29, 1.62, 1.99, 
                       2.43, 2.83, 3.17, 3.51, 3.68, 3.80, 3.90, 3.97, 4.02])
n_voltage = np.array([3.61, 3.48, 3.43, 3.40, 3.36, 3.30, 3.19, 3.09, 2.97, 2.81, 
                      2.62, 2.49, 2.36, 2.20, 2.05, 1.85, 1.58, 1.30, 1.07])[::-1]
ocv_interp = interp1d(n_capacity / max(n_capacity), n_voltage, kind='cubic', fill_value="extrapolate")

# --------------------------
# EKF SoC Estimator Class
# --------------------------
class SoCEKF:
    def __init__(self, initial_soc, ocv_model_func):
        self.soc = initial_soc
        self.P = 1e-3  # Initial error covariance
        self.ocv_model = ocv_model_func

    def predict(self, current_ma, dt_sec):
        delta_soc = -(current_ma * dt_sec) / (3600.0 * Q_CAPACITY)
        self.soc += delta_soc
        self.P += Qp

    def update(self, voltage_measured, current_ma):
        V_pred = self.ocv_model(self.soc) + R_INTERNAL * current_ma
        H = self.df_dSoC(self.soc)

        K = self.P * H / (H**2 * self.P + Rm)
        self.soc = self.soc + K * (voltage_measured - V_pred)
        self.P = (1 - K * H) * self.P
        self.soc = max(0.0, min(1.0, self.soc))

    def df_dSoC(self, soc):
        delta = 1e-4
        return (self.ocv_model(soc + delta) - self.ocv_model(soc - delta)) / (2 * delta)

# --------------------------
# Simulation Function
# --------------------------
def simulate_ekf_soc():
    dt = 1.0  # seconds
    steps = 3600  # simulate for 1 hour
    load_current = 500.0 # mA

    ekf = SoCEKF(initial_soc=1.0, ocv_model_func=ocv_interp)
    soc_log = []
    v_log = []
    true_soc = 1.0
    soc_true_log = []

    for i in range(steps):
        true_soc -= (load_current * dt) / (3600.0 * Q_CAPACITY)
        true_soc = max(0.0, true_soc)

        v_true = ocv_interp(true_soc) + R_INTERNAL * load_current
        v_measured = v_true + np.random.normal(-0.01, 0.01)
        v_log.append(v_measured)

        ekf.predict(load_current, dt)
        ekf.update(v_measured, load_current)

        soc_log.append(ekf.soc)
        soc_true_log.append(true_soc)

    return soc_log, soc_true_log, v_log

if 0:
    # Generate a dense range of SoC values between 0 and 1
    soc_range = np.linspace(0, 1, 200)
    voltage_range = ocv_interp(soc_range)

    # Plot the OCV vs SoC curve
    plt.figure(figsize=(8, 5))
    plt.plot(soc_range, voltage_range, label='OCV vs SoC', color='blue')
    plt.xlabel('State of Charge (SoC)')
    plt.ylabel('Open Circuit Voltage (V)')
    plt.title('OCV-SOC Curve from Provided Data')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if 1:
    soc_estimated, soc_true, v_log = simulate_ekf_soc()
    plt.figure(figsize=(10, 6))
    #plt.plot(v_log, label='voltage', linestyle='--')
    plt.scatter(soc_true, v_log, label='True SoC', color="blue")
    plt.scatter(soc_estimated, v_log, label='Estimated SoC (EKF)', alpha=0.8, linestyle='--', color="orange")
    plt.xlabel('State of Charge (SoC)')
    plt.ylabel('Measured Voltage (V)')
    plt.title('EKF-Based SoC Estimation')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
