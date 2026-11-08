clear; clc; close all;

%% variables
csv_file    = 'OpenloopTest1_29_10_2026.csv';  % columns: time_s, setpoint, temperature, duty, pwm
Tref        = 40;          % setpoint
toff_mode   = 'crossing';  
toff_manual = 490;         
ton_manual  = 30;          
rmse_tol    = 1.10;        

%% load and clean data
d = readtable(csv_file);
t = d.time_s;          y = d.temperature;
if iscell(t) || isstring(t), t = str2double(t); end
if iscell(y) || isstring(y), y = str2double(y); end
t = double(t(:));  y = double(y(:));
ok = isfinite(t) & isfinite(y);            n_bad = sum(~ok);
t = t(ok);  y = y(ok);
if ismember('duty', d.Properties.VariableNames)
    duty = double(d.duty(:));  duty = duty(ok);
else
    duty = [];
end
[t, iu] = unique(t, 'stable');   y = y(iu);   if ~isempty(duty), duty = duty(iu); end
[t, is] = sort(t);               y = y(is);   if ~isempty(duty), duty = duty(is); end

% test input: start of heating and heating level (duty may be logged in % or 0-1)
if ~isempty(duty) && any(duty > 0)
    ton  = t(find(duty > 0, 1));
    u_on = max(duty);
    if u_on > 1.5, u_on = u_on/100; end     % logged in percent
else
    ton = ton_manual;  u_on = 1;
end
Tamb = mean(y(t < ton - 2));               % ambient = temperature before heating
dy   = y - Tamb;                           % temperature rise above ambient

% switch-off time
iref = find(y >= Tref, 1);
if isempty(iref), error('Temperature never reached Tref = %g degC in the log.', Tref); end
t_cross = t(iref);
[ypk, ipk] = max(y);

ts = (0:1:t(end))';

%% model structs
models = struct( ...
  'name', {'1st order + delay', '2 equal lags + delay', '2 lags + delay'}, ...
  'type', {'P1D', 'P2D_eq', 'P2D'}, ...
  'p0',   {[40 400 20], [40 350 25], [40 500 250 20]}, ...
  'lb',   {[1 10 0],    [1 10 0],    [1 10 10 0]}, ...
  'ub',   {[1000 5000 150], [1000 3000 150], [1000 5000 5000 150]});
opts = optimoptions('lsqnonlin', 'Display', 'off', 'MaxFunctionEvaluations', 5000, ...
                    'FunctionTolerance', 1e-10, 'StepTolerance', 1e-10);

%% switch-off time
switch toff_mode
    case 'crossing',  toff = t_cross;
    case 'manual',    toff = toff_manual;
    case 'best_fit'
        offs = -30:2:40;  rm = zeros(size(offs));
        for k = 1:numel(offs)
            r = fit_one(models(2), t, dy, ts, ton, t_cross + offs(k), u_on, opts);
            rm(k) = r.rmse;
        end
        [~, kb] = min(rm);  toff = t_cross + offs(kb);
    otherwise, error('toff_mode must be crossing, manual or best_fit.');
end

fprintf('=============== DATA ===============\n');
fprintf('File: %s  (%d valid rows, %d bad rows removed)\n', csv_file, numel(t), n_bad);
fprintf('Sample time           : %.2f s,  duration %.0f s\n', median(diff(t)), t(end));
fprintf('Ambient (before heat) : %.2f degC\n', Tamb);
fprintf('Heating starts t_on   : %.1f s at duty %.2f\n', ton, u_on);
fprintf('Reached %.1f degC at   : %.1f s (first sample >= Tref)\n', Tref, t_cross);
fprintf('Supply off t_off used : %.1f s  (mode: %s)\n', toff, toff_mode);
fprintf('Peak after cut-off    : %.2f degC at %.0f s (%.0f s after t_off)\n', ypk, t(ipk), t(ipk) - toff);
fprintf('Temperature at end    : %.2f degC (still cooling toward ambient)\n\n', y(end));

%% fit all structs
fits = cell(1, numel(models));
for i = 1:numel(models)
    fits{i} = fit_one(models(i), t, dy, ts, ton, toff, u_on, opts);
end
rmse_all = cellfun(@(f) f.rmse, fits);
best = min(rmse_all);
sel = find(rmse_all <= rmse_tol*best, 1);   % simplest model within tolerance

fprintf('=============== MODEL COMPARISON ===============\n');
fprintf('%-22s %8s %8s   %s\n', 'Model', 'Fit [%]', 'RMSE[C]', 'Parameters');
for i = 1:numel(models)
    fprintf('%-22s %8.1f %8.3f   %s\n', models(i).name, fits{i}.fitpct, fits{i}.rmse, ...
            param_str(models(i).type, fits{i}.p));
end
fprintf('Selected (simplest within %.0f %% of best RMSE): %s\n\n', (rmse_tol-1)*100, models(sel).name);

%% selected plant
f = fits{sel};  p = f.p;
switch models(sel).type
    case 'P1D',    K = p(1); taus = p(2);        Ld = p(3);
    case 'P2D_eq', K = p(1); taus = [p(2) p(2)]; Ld = p(3);
    case 'P2D',    K = p(1); taus = [p(2) p(3)]; Ld = p(4);
end
den = 1;  for k = 1:numel(taus), den = conv(den, [taus(k) 1]); end
G   = tf(K, den, 'InputDelay', Ld);

fprintf('=============== IDENTIFIED PLANT (duty 0..1 -> temperature rise) ===============\n');
fprintf('K (DC gain)         = %.2f degC per unit duty\n', K);
for k = 1:numel(taus), fprintf('tau%d                = %.1f s\n', k, taus(k)); end
fprintf('Dead time L         = %.1f s\n', Ld);
fprintf('Fit / RMSE          = %.1f %% / %.3f degC\n', f.fitpct, f.rmse);
fprintf('Duty needed for %.0f degC (steady state) = %.3f\n', Tref, (Tref - Tamb)/K);
fprintf('Transfer function:\n'); G %#ok<NOPTS>

% parameter confidence intervals (optional, needs Statistics Toolbox)
try
    ci = nlparci(f.p, f.resid, 'jacobian', f.J);
    fprintf('95 %% confidence intervals (parameters in the order shown above):\n');
    disp(ci);
catch
    fprintf('(nlparci not available -> no confidence intervals)\n');
end

%% ---------------- Sensitivity to the switch-off time ----------------
fprintf('\n=============== SENSITIVITY TO t_off (%s) ===============\n', models(sel).name);
fprintf('%10s %8s %8s %8s %9s\n', 't_off [s]', 'K', 'tau1', 'L', 'RMSE');
offs = [-20 -10 0 10 20 30];  sens = zeros(numel(offs), 4);
for k = 1:numel(offs)
    r = fit_one(models(sel), t, dy, ts, ton, toff + offs(k), u_on, opts);
    switch models(sel).type
        case 'P1D',    Kk = r.p(1); Tk = r.p(2); Lk = r.p(3);
        case 'P2D_eq', Kk = r.p(1); Tk = r.p(2); Lk = r.p(3);
        case 'P2D',    Kk = r.p(1); Tk = r.p(2); Lk = r.p(4);
    end
    sens(k,:) = [toff + offs(k), Kk, Tk, Lk];
    fprintf('%10.1f %8.2f %8.1f %8.1f %9.3f\n', toff + offs(k), Kk, Tk, Lk, r.rmse);
end

%% ---------------- Parameters for Simulink / design scripts ----------------
fprintf('\n=============== FOR SIMULINK ===============\n');
fprintf('Transport Delay      : %.1f\n', Ld);
fprintf('Transfer Fcn numer.  : %.2f\n', K);
fprintf('Transfer Fcn denom.  : %s\n', mat2str(den, 6));
fprintf('Ambient offset block : %.2f\n', Tamb);
fprintf('MATLAB: G = tf(%.2f, %s, ''InputDelay'', %.1f);\n', K, mat2str(den,6), Ld);
save('plant_model.mat', 'G', 'K', 'taus', 'Ld', 'Tamb', 'toff', 'ton', 'u_on');
fprintf('Saved G, K, taus, Ld, Tamb, toff, ton to plant_model.mat\n');

%% ---------------- Plots ----------------
u = u_on*double(ts >= ton & ts < toff);
figure('Name', 'Open-loop fit', 'Color', 'w');
subplot(3,1,1); hold on; grid on;
plot(t, y, 'k', 'LineWidth', 1.6);
col = lines(numel(models));
for i = 1:numel(models)
    yh = model_out(fits{i}.p, models(i).type, ts, ton, toff, u_on, t);
    plot(t, Tamb + yh, 'Color', col(i,:), 'LineWidth', 1.1);
end
plot(toff*[1 1], ylim, 'r--');
ylabel('Temperature (°C)');  title('Open-loop pulse response: measured vs models');
legend(['Measured', {models.name}, 't_{off}'], 'Location', 'northwest');
subplot(3,1,2); hold on; grid on;
yh = model_out(fits{sel}.p, models(sel).type, ts, ton, toff, u_on, t);
plot(t, y - (Tamb + yh), 'LineWidth', 1.2);
ylabel('Residual (°C)'); title(sprintf('Residual of selected model (%s), RMSE = %.3f °C', ...
                                       models(sel).name, f.rmse));
subplot(3,1,3); stairs(ts, u, 'LineWidth', 1.4); grid on; ylim([-0.1 1.1]);
xlabel('t (s)'); ylabel('Input duty'); title('Input used in the fit (supply cut by hand)');

figure('Name', 't_off sensitivity', 'Color', 'w');
subplot(3,1,1); plot(sens(:,1), sens(:,2), 'o-'); grid on; ylabel('K'); title('Sensitivity to t_off');
subplot(3,1,2); plot(sens(:,1), sens(:,3), 'o-'); grid on; ylabel('\tau_1 (s)');
subplot(3,1,3); plot(sens(:,1), sens(:,4), 'o-'); grid on; ylabel('L (s)'); xlabel('t_{off} (s)');

figure('Name', 'Step response of identified plant', 'Color', 'w');
step(G, 3000); grid on; title('Unit duty step of the identified plant');

%% =====================================================================
%                              LOCAL FUNCTIONS
%% =====================================================================
function yh = model_out(p, type, ts, ton, toff, u_on, t)
% Simulates the pulse response and returns it at the log times t.
    switch type
        case 'P1D',    G = tf(p(1), [p(2) 1]);                    L = p(3);
        case 'P2D_eq', G = tf(p(1), conv([p(2) 1], [p(2) 1]));    L = p(3);
        case 'P2D',    G = tf(p(1), conv([p(2) 1], [p(3) 1]));    L = p(4);
    end
    u  = u_on*double(ts >= ton & ts < toff);
    yl = lsim(G, u, ts);
    yh = interp1(ts + L, yl, t, 'linear', 0);      % apply dead time
end

function r = fit_one(m, t, dy, ts, ton, toff, u_on, opts)
% Least-squares fit of one model structure (two starting points, best kept).
    cost   = @(p) model_out(p, m.type, ts, ton, toff, u_on, t) - dy;
    starts = {m.p0, m.p0 .* [1 2*ones(1, numel(m.p0)-2) 1]};
    r.rmse = Inf;
    for k = 1:numel(starts)
        [p, ~, res, ~, ~, ~, J] = lsqnonlin(cost, starts{k}, m.lb, m.ub, opts);
        rm = sqrt(mean(res.^2));
        if rm < r.rmse
            r.p = p; r.rmse = rm; r.resid = res; r.J = full(J);
        end
    end
    r.fitpct = 100*(1 - norm(r.resid)/norm(dy - mean(dy)));
end

function s = param_str(type, p)
    switch type
        case 'P1D',    s = sprintf('K=%.1f  tau=%.0f s  L=%.1f s', p(1), p(2), p(3));
        case 'P2D_eq', s = sprintf('K=%.1f  tau=%.0f s (x2)  L=%.1f s', p(1), p(2), p(3));
        case 'P2D',    s = sprintf('K=%.1f  tau1=%.0f s  tau2=%.0f s  L=%.1f s', p(1), p(2), p(3), p(4));
    end
end
