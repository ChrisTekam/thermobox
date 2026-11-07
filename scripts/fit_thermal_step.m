%% fit_thermal_step.m
% Fits a delayed two-lag (overdamped 2nd-order) model to the open-loop run:
%   heater OFF 0..t_on, heater ON at 100 % from t_on until T reached 40 C,
%   then supply switched off (free cooling).
%
%   T(t) = Ta + K * y(t),   y = response of  e^(-theta*s) / ((tau1*s+1)(tau2*s+1))
%   to a pulse input (1 between t_on and t_off, 0 elsewhere)
%
% Base MATLAB only (fminsearch). Needs R2016b+ (local function at the end).

clear; clc; close all;

%% ---- Settings ----
csvFile  = 'OpenloopTest1_29_10_2026';
T_target = 40;     % degC: supply was switched off when this was reached
t_on     = 30;     % s:    heater switched on (BASELINE_S in open_loop_test.ino)

%% ---- Load data ----
d  = readtable(csvFile);
t  = d.time_s;
T  = d.temperature;
ok = isfinite(t) & isfinite(T);
t  = t(ok);  T = T(ok);

idx   = find(T >= T_target, 1, 'first');   % first sample at/above 40 C
t_off = t(idx);                            % power-off time
dt    = median(diff(t));                   % sample time (~1 s)
Ta0   = mean(T(t < t_on));                 % initial ambient estimate

%% ---- Fit: p = [Ta  K  theta  tau1  tau2] ----
p0   = [Ta0, 40, 10, 300, 100];
cost = @(p) sum((simModel(p, t, t_on, t_off, dt) - T).^2);
opts = optimset('MaxFunEvals', 1e5, 'MaxIter', 1e5, 'TolX', 1e-9, 'TolFun', 1e-12);

p = fminsearch(cost, p0, opts);
for k = 1:3                                % restart a few times to settle
    p = fminsearch(cost, p, opts);
end

Ta    = p(1);
K     = abs(p(2));
theta = abs(p(3));
tau   = sort(abs(p(4:5)), 'descend');

%% ---- Fit quality ----
Tfit = simModel(p, t, t_on, t_off, dt);
res  = T - Tfit;
rmse = sqrt(mean(res.^2));
R2   = 1 - sum(res.^2) / sum((T - mean(T)).^2);
[Tpk, ipk] = max(Tfit);

%% ---- Command window output ----
fprintf('\n================ THERMAL FIT RESULTS ================\n');
fprintf('Data file          : %s  (%d samples, dt = %.2f s)\n', csvFile, numel(t), dt);
fprintf('Heater ON          : t = %.1f s\n', t_on);
fprintf('Power OFF (T>=%.0fC): t = %.1f s  (T = %.3f C)\n', T_target, t_off, T(idx));
fprintf('-----------------------------------------------------\n');
fprintf('Model: T = Ta + K * e^(-theta*s) / ((tau1*s+1)(tau2*s+1)) * u\n');
fprintf('  Ta    (ambient)           = %.3f C\n',  Ta);
fprintf('  K     (gain @ 100%% power) = %.3f C   -> steady state ~ %.2f C\n', K, Ta + K);
fprintf('  theta (dead time)         = %.2f s\n',  theta);
fprintf('  tau1                      = %.2f s\n',  tau(1));
fprintf('  tau2                      = %.2f s\n',  tau(2));
fprintf('-----------------------------------------------------\n');
fprintf('RMSE = %.4f C   R^2 = %.5f   max |err| = %.3f C\n', rmse, R2, max(abs(res)));
fprintf('Model peak: %.2f C at t = %.1f s (%.1f s after power off)\n', ...
        Tpk, t(ipk), t(ipk) - t_off);
fprintf('Measured peak: %.2f C at t = %.1f s\n', max(T), t(find(T == max(T), 1)));
fprintf('=====================================================\n\n');

%% ---- Plot ----
figure('Name', 'Thermal fit');
subplot(2,1,1);
plot(t, T, '.', 'Color', [0.6 0.6 0.6]); hold on;
plot(t, Tfit, 'r', 'LineWidth', 1.5);
xline(t_on,  '--g', 'Heater ON');
xline(t_off, '--k', 'Power OFF');
grid on; xlabel('Time (s)'); ylabel('Temperature (\circC)');
legend('Measured', 'Fit', 'Location', 'southeast');
title(sprintf('Fit: K=%.1f, \\theta=%.1f s, \\tau_1=%.0f s, \\tau_2=%.0f s (RMSE %.3f \\circC)', ...
      K, theta, tau(1), tau(2), rmse));
subplot(2,1,2);
plot(t, res, 'b'); grid on;
xlabel('Time (s)'); ylabel('Residual (\circC)');

%% ---- Local function: simulate model on the sample grid ----
function y = simModel(p, t, t_on, t_off, dt)
    Ta  = p(1);
    K   = abs(p(2));
    th  = abs(p(3));
    tau = max(abs(p(4:5)), 1e-3);

    % Delayed pulse input with soft (sub-sample) edges
    u = min(max((t - (t_on  + th))/dt + 0.5, 0), 1) ...
      - min(max((t - (t_off + th))/dt + 0.5, 0), 1);

    % Two cascaded first-order lags (exact discretisation)
    x = u;
    for k = 1:numel(tau)
        a = exp(-dt / tau(k));
        x = filter(1 - a, [1 -a], x);
    end
    y = Ta + K * x;
end
