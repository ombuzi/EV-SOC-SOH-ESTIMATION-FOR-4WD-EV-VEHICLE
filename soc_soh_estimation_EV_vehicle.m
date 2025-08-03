% Advanced Battery Pack Simulation with UKF State Estimation
% For 600km Range EV with 4WD, Gearbox, and eCVT Integration
% Author: Frank Otieno
% Date: August 03, 2025
% Assumptions: Li-ion NMC811, 100 kWh pack, 96s4p configuration
% Load Profile: Custom drive cycle incorporating gearbox shifts and eCVT ratios

clear; clc; close all;

%% Section 1: Parameter Initialization
% Battery Parameters (Advanced: Temperature-dependent, Aging factors)
NomCap = 100; % Nominal capacity in Ah (for entire pack)
NomV = 400; % Nominal voltage (V)
R0 = 0.01; % Ohmic resistance (Ohm)
R1 = 0.005; % RC1 resistance (Ohm)
C1 = 1000; % RC1 capacitance (F)
R2 = 0.01; % RC2 resistance (Ohm)
C2 = 5000; % RC2 capacitance (F)
Tau1 = R1 * C1; % Time constant 1
Tau2 = R2 * C2; % Time constant 2

% OCV-SoC Curve (Polynomial fit for advanced modeling)
SoC_vec = 0:0.1:1;
OCV_vec = 3.2 + 0.8*SoC_vec + 0.2*SoC_vec.^2 - 0.1*SoC_vec.^3; % Per cell, scale to pack later
OCV_func = @(SoC) interp1(SoC_vec, OCV_vec, SoC, 'spline') * (NomV / 3.7); % Scale to pack voltage

% Temperature Effects (Advanced: Arrhenius-based resistance scaling)
T_amb = 25; % Ambient temp (C)
T = T_amb; % Initial battery temp
Alpha_T = 0.005; % Temp coefficient for resistance

% Aging Parameters (SoH estimation)
SoH_init = 1; % Initial SoH (1 = new)
Aging_factor = 0.0001; % Capacity fade per cycle (simplified)

% Simulation Settings
dt = 1; % Time step (s)
t_end = 3600; % Simulation time (s, ~1 hour for subset of 600km cycle)
t = 0:dt:t_end;
N = length(t);

%% Section 2: Generate Dynamic Load Profile
% Advanced: Mimic EV drive cycle with gearbox (2-speed), eCVT (variable ratio), 4WD
% Current profile: Positive = discharge, Negative = charge (regen)
% Urban: Low current, frequent stops
% Highway: Steady high current
% Hill-climb: Peaks with regen

% Gearbox/eCVT emulation: Efficiency eta = 0.95 - 0.98 based on ratio
gear_ratio = ones(N,1); % 1 = low gear/eCVT low, 2 = high
eta_trans = 0.95 * ones(N,1); % Transmission efficiency

% Current Demand (A): Scaled for 100 kWh pack
I_base = 50 * sin(2*pi*t/(t_end/10)); % Sinusoidal base for dynamics
I_urban = 100 * (t < t_end/3) .* (rand(N,1) - 0.5)*20; % Noise for urban
I_highway = 150 * (t >= t_end/3 & t < 2*t_end/3); % Steady discharge
I_hill = 200 * exp(-((t - 2.5*t_end/3)/ (t_end/10)).^2) - 100 * exp(-((t - 2.8*t_end/3)/ (t_end/10)).^2); % Peak discharge + regen
I = I_base + I_urban + I_highway + I_hill;

% Adjust for transmission: Effective battery current = I_motor / eta_trans / gear_ratio
% Simplified: Assume motor current = I, adjust for losses
I = I ./ eta_trans; % Increase current to account for losses

% Error Handling: Clip current to safe limits (e.g., 3C max discharge)
C_rate_max = 3;
I = max(min(I, C_rate_max * NomCap), -C_rate_max * NomCap);

%% Section 3: ECM Simulation (ODE for RC branches)
% States: SoC, V_RC1, V_RC2, SoH, T
x = zeros(5, N);
x(:,1) = [1; 0; 0; SoH_init; T]; % Initial: Full SoC, zero voltages, new SoH, ambient T

% Preallocate outputs
V_t = zeros(N,1); % Terminal voltage
P_out = zeros(N,1); % Power output

for k = 2:N
    SoC_k = x(1,k-1);
    V_RC1_k = x(2,k-1);
    V_RC2_k = x(3,k-1);
    SoH_k = x(4,k-1);
    T_k = x(5,k-1);
    
    % Update resistances with temp and SoH
    R0_adj = R0 * (1 + Alpha_T*(T_k - 25)) / SoH_k;
    R1_adj = R1 * (1 + Alpha_T*(T_k - 25)) / SoH_k;
    R2_adj = R2 * (1 + Alpha_T*(T_k - 25)) / SoH_k;
    
    % OCV
    V_ocv = OCV_func(SoC_k);
    
    % RC dynamics (discretized Euler method for speed)
    dV_RC1 = -V_RC1_k / Tau1 + I(k-1) / C1;
    dV_RC2 = -V_RC2_k / Tau2 + I(k-1) / C2;
    V_RC1_new = V_RC1_k + dt * dV_RC1;
    V_RC2_new = V_RC2_k + dt * dV_RC2;
    
    % SoC update (Coulomb counting with efficiency)
    eta_coul = 0.98; % Coulombic efficiency
    dSoC = -I(k-1) * dt / (3600 * NomCap * SoH_k);
    SoC_new = SoC_k + eta_coul * dSoC;
    
    % SoH update (simplified cycle aging)
    dSoH = -Aging_factor * abs(dSoC); % Fade proportional to throughput
    SoH_new = SoH_k + dSoH;
    
    % Thermal model (simple heat balance)
    Q_gen = I(k-1)^2 * R0_adj + (V_RC1_k^2 / R1_adj) + (V_RC2_k^2 / R2_adj); % Joule heating
    h_conv = 10; % Convection coefficient (W/m^2K), assume pack surface
    A_pack = 2; % Effective area (m^2)
    dT = (Q_gen - h_conv * A_pack * (T_k - T_amb)) / (1000 * 0.5); % mC = 1000 J/K (specific heat * mass approx)
    T_new = T_k + dt * dT;
    
    % Update states
    x(:,k) = [SoC_new; V_RC1_new; V_RC2_new; SoH_new; T_new];
    
    % Terminal voltage
    V_t(k) = V_ocv - I(k) * R0_adj - V_RC1_new - V_RC2_new;
    
    % Power (for SoP later)
    P_out(k) = V_t(k) * I(k);
end

%% Section 4: UKF for State Estimation
% Advanced: Unscented Kalman Filter for SoC, V_RC1, V_RC2 estimation
% Measurement: Voltage + Current (noisy)
sigma_V = 0.01; % Voltage noise std
sigma_I = 1; % Current noise std
Y_meas = V_t + sigma_V * randn(N,1); % Noisy voltage
I_meas = I + sigma_I * randn(N,1); % Noisy current

% UKF Parameters
n_states = 3; % SoC, V_RC1, V_RC2
alpha = 1e-3;
kappa = 0;
beta = 2;
lambda = alpha^2 * (n_states + kappa) - n_states;

Q = diag([1e-6, 1e-4, 1e-4]); % Process noise cov
R = sigma_V^2; % Measurement noise cov

% Initial state and cov
x_est = [0.95; 0; 0]; % Slightly off initial guess
P = eye(n_states) * 0.01;

x_ukf = zeros(n_states, N);
x_ukf(:,1) = x_est;

for k = 2:N
    % Sigma points
    sqrtP = chol((n_states + lambda) * P, 'lower');
    X_sig = [x_est, x_est + sqrtP, x_est - sqrtP];
    
    % Time update (propagate sigma points)
    X_pred = zeros(n_states, 2*n_states+1);
    for i = 1:2*n_states+1
        SoC_sig = X_sig(1,i);
        V_RC1_sig = X_sig(2,i);
        V_RC2_sig = X_sig(3,i);
        
        % Process model (same as ECM)
        dSoC_sig = -I_meas(k-1) * dt / (3600 * NomCap * SoH_init); % Assume known SoH for simplicity
        SoC_pred = SoC_sig + dSoC_sig;
        
        dV_RC1_sig = -V_RC1_sig / Tau1 + I_meas(k-1) / C1;
        V_RC1_pred = V_RC1_sig + dt * dV_RC1_sig;
        
        dV_RC2_sig = -V_RC2_sig / Tau2 + I_meas(k-1) / C2;
        V_RC2_pred = V_RC2_sig + dt * dV_RC2_sig;
        
        X_pred(:,i) = [SoC_pred; V_RC1_pred; V_RC2_pred];
    end
    
    % Mean and cov prediction
    Wm = [lambda/(n_states+lambda), ones(1,2*n_states)/(2*(n_states+lambda))];
    Wc = Wm; Wc(1) = Wc(1) + (1 - alpha^2 + beta);
    x_pred = X_pred * Wm';
    P_pred = (X_pred - x_pred) * diag(Wc) * (X_pred - x_pred)' + Q;
    
    % Measurement update
    Y_sig = zeros(1, 2*n_states+1);
    for i = 1:2*n_states+1
        V_ocv_sig = OCV_func(X_pred(1,i));
        V_t_sig = V_ocv_sig - I_meas(k) * R0 - X_pred(2,i) - X_pred(3,i); % Note: Use current at k for measurement
        Y_sig(i) = V_t_sig;
    end
    
    y_pred = Y_sig * Wm';
    Py = (Y_sig - y_pred) * diag(Wc) * (Y_sig - y_pred)' + R;
    Pxy = (X_pred - x_pred) * diag(Wc) * (Y_sig - y_pred)';
    
    K = Pxy / Py; % Kalman gain
    x_est = x_pred + K * (Y_meas(k) - y_pred);
    P = P_pred - K * Py * K';
    
    x_ukf(:,k) = x_est;
end

%% Section 5: Results Visualization and Analysis
% Plot True vs Estimated SoC
figure(1);
plot(t/3600, x(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t/3600, x_ukf(1,:), 'r--', 'LineWidth', 2);
xlabel('Time (hours)'); ylabel('SoC');
legend('True SoC', 'UKF Estimated SoC');
title('SoC Estimation During EV Drive Cycle');
grid on;

% Terminal Voltage
figure(2);
plot(t/3600, V_t, 'k-', 'LineWidth', 2);
xlabel('Time (hours)'); ylabel('Terminal Voltage (V)');
title('Battery Terminal Voltage Under Load');
grid on;

% Temperature and SoH
figure(3);
yyaxis left; plot(t/3600, x(5,:), 'g-', 'LineWidth', 2); ylabel('Temperature (C)');
yyaxis right; plot(t/3600, x(4,:), 'm-', 'LineWidth', 2); ylabel('SoH');
xlabel('Time (hours)');
title('Thermal and Aging Dynamics');
grid on;

% Detailed Analysis
SoC_RMSE = sqrt(mean((x(1,:) - x_ukf(1,:)).^2));
fprintf('SoC Estimation RMSE: %.4f\n', SoC_RMSE);

% Sensitivity Sweep: Vary NomCap for 600km range impact
Cap_sweep = NomCap * [0.9, 1, 1.1];
Range_est = zeros(3,1);
for i = 1:3
    Energy_used = trapz(t, abs(P_out)) / 3600; % Wh used in sim
    Range_est(i) = (Cap_sweep(i) * 1000 * mean(x(1,:))) / (Energy_used / (t_end/3600)) * (600 / NomCap); % Scaled to 600km
end
disp('Capacity Sweep for Estimated Range (km):');
disp(table(Cap_sweep', Range_est));

% Fault Simulation: Inject cell imbalance (e.g., 10% SoC deviation in one module)
% Advanced extension: Rerun with modified SoC_init for one 'cell', observe voltage drop
% (Omitted for brevity, but add: SoC_fault = x(1,:)*0.9; recalculate V_t_fault)

% Integration Note: In full EV, feed SoC to eCVT controller for ratio adjustment (e.g., lower ratio if SoC low)
% Gearbox shift threshold: If P_out > 100kW, shift to high gear (code extension possible)