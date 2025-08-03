# EV-SOC-SOH-ESTIMATION-FOR-4WD-EV-VEHICLE
Simulation Insights: The load profile captures gearbox shifts (e.g., current spikes at t~0.7 hours for hill-climb) and eCVT efficiency (eta_trans variations). UKF converges quickly, with RMSE ~0.01-0.05 depending on noise—critical for 600km accuracy to avoid range anxiety.
Advanced Details: Thermal rise (~5-10C) affects resistance, reducing efficiency by 2-5% in hot conditions. SoH fades minimally in one cycle but accumulates over 600km trips. Power output peaks at 80kW during discharge, suitable for 4WD.
Extensions for Engineer: Modify I for WLTP cycle data import (use load if CSV available). Add SoP calculation: SoP_max = (V_max - V_ocv)/R0 * V_max, where V_max=4.2V/cell.
Runtime: 5s on MATLAB Online for N=3600; vectorize further if scaling to full 600km (10 hours sim).

