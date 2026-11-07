%% PI design for the thermal box: course method vs. grid-search loop shaping
%  Plant (identified from open-loop test): G(s) = K*exp(-Ld*s)/(tau*s+1)^2
%  Controller on the Arduino: discrete PI, Ts = 1 s, duty limited to [0,1],
%  conditional-integration anti-windup (same as pi_controller.cpp).
%  Needs: Control System Toolbox. Local functions at the bottom (R2016b+).
clear; clc; close all;

%% ---------------- Inputs ----------------
K      = 38.7;      % degC per unit duty
tau    = 354;       % s, two equal lags
Ld     = 27;        % s, dead time
Ts     = 1;         % s, Arduino sample time
Tamb   = 27;        % degC (same offset as in Simulink)
Tref   = 40;        % degC target
r      = Tref - Tamb;   % step in temperature rise [K]
OS_req = 2.5;       % % overshoot requirement
Ts_req = 1500;      % s, 2 % settling-time requirement
PM_min = 60;        % deg, minimum phase margin accepted in the grid search

s = tf('s');
G = tf(K, conv([tau 1],[tau 1]), 'InputDelay', Ld);   % continuous plant incl. delay

fprintf('=============== PLANT AND SPECS ===============\n');
fprintf('G(s) = %.1f*exp(-%gs)/(%gs+1)^2,  Ts = %g s\n', K, Ld, tau, Ts);
fprintf('Step: %.1f -> %.1f degC (rise r = %.1f K), steady duty needed = %.3f\n', ...
        Tamb, Tref, r, r/K);
fprintf('Spec: OS <= %.1f %%, Ts(2%%) <= %g s\n\n', OS_req, Ts_req);

%% ======================================================================
%  METHOD A: course method (steps 1-6 of the slide)
%% ======================================================================
% Step 1: zeta from %OS
zeta = -log(OS_req/100) / sqrt(pi^2 + log(OS_req/100)^2);

% Step 2: phase margin for zeta (Eq. 10.73) and rule of thumb 100*zeta
PM_exact = atand( 2*zeta / sqrt(-2*zeta^2 + sqrt(1 + 4*zeta^4)) );
PM_rule  = 100*zeta;
PM_des   = PM_exact;           % rule of thumb is only valid for zeta <= 0.7

% Step 3: bandwidth from settling time (Eq. 10.55); crossover wc ~ wBW
wBW = 4/(Ts_req*zeta) * sqrt( (1 - 2*zeta^2) + sqrt(4*zeta^4 - 4*zeta^2 + 2) );
wc  = wBW;

% Step 4: loop with the integrator alone, L0 = G/s, evaluated at wc
[magG, phG] = bode(G, wc);  magG = squeeze(magG);  phG = squeeze(phG);
phL0 = phG - 90;                              % phase of L0 at wc [deg]
magL0 = magG/wc;

% Step 5: place the PI zero z = Ki/Kp so that PM = PM_des at wc
%   angle(C) = -90 + atan(wc/z)  ->  PM = 180 + angle(C) + phG
phC_needed = PM_des - 180 - phG;              % required controller phase [deg]
a = phC_needed + 90;                          % = atand(wc/z)
if a <= 0 || a >= 90
    error('Course method: no PI zero gives PM = %.1f deg at wc = %.3g rad/s.', PM_des, wc);
end
zc = wc / tand(a);                            % zero frequency [rad/s]

% Step 6: Kp so that |L(j wc)| = 1
Kp_A = 1 / (magG * sqrt(1 + (zc/wc)^2));
Ki_A = Kp_A * zc;
Ti_A = Kp_A / Ki_A;

fprintf('=============== METHOD A: COURSE METHOD ===============\n');
fprintf('Step 1  zeta (from OS = %.1f %%)            = %.4f\n', OS_req, zeta);
fprintf('Step 2  PM exact (Eq. 10.73)                = %.2f deg\n', PM_exact);
fprintf('        PM rule of thumb 100*zeta           = %.2f deg  (valid only for zeta <= 0.7 -> not used)\n', PM_rule);
fprintf('Step 3  wBW (Eq. 10.55) = wc                = %.4e rad/s\n', wBW);
fprintf('Step 4  integrator-only loop at wc: |L0|   = %.3f (%.1f dB), phase = %.1f deg\n', ...
        magL0, 20*log10(magL0), phL0);
fprintf('        plant at wc: |G| = %.3f, phase = %.1f deg\n', magG, phG);
fprintf('Step 5  required controller phase           = %.2f deg\n', phC_needed);
fprintf('        PI zero z = Ki/Kp                   = %.4e rad/s  (Ti = %.0f s)\n', zc, 1/zc);
fprintf('Step 6  Kp                                  = %.5f\n', Kp_A);
fprintf('        Ki = Kp*z                           = %.4e\n\n', Ki_A);

%% ======================================================================
%  METHOD B: my method = loop shaping + grid search on the real discrete loop
%  Pick Kp, Ti from a grid; keep the pairs that meet OS and Ts on the discrete
%  closed-loop simulation (delay, 0-1 duty limit, anti-windup) and PM >= PM_min;
%  choose the fastest settling one.
%% ======================================================================
Ti_grid = 200:50:2000;
Kp_grid = 0.005:0.0025:0.06;
Tend_search = 3000;

fprintf('=============== METHOD B: GRID SEARCH ===============\n');
fprintf('Searching %d x %d = %d candidates ...\n', numel(Ti_grid), numel(Kp_grid), ...
        numel(Ti_grid)*numel(Kp_grid));

cand = [];            % columns: Kp Ki Ti PM OS Ts
for Ti = Ti_grid
    for Kp = Kp_grid
        Ki = Kp/Ti;
        pm = pm_cont(Kp, Ki, K, tau, Ld);
        if isnan(pm) || pm < PM_min, continue; end
        [t, y, u] = sim_pi(Kp, Ki, K, tau, Ld, Ts, r, Tend_search);
        m = step_metrics(t, y, u, r);
        if m.OS <= OS_req && m.Ts <= Ts_req
            cand(end+1,:) = [Kp Ki Ti pm m.OS m.Ts]; %#ok<SAGROW>
        end
    end
end
if isempty(cand)
    error('No (Kp,Ti) in the grid meets OS <= %.1f%%, Ts <= %g s, PM >= %g deg.', ...
          OS_req, Ts_req, PM_min);
end
[~, ib] = min(cand(:,6));                     % fastest settling among feasible
Kp_B = cand(ib,1);  Ki_B = cand(ib,2);  Ti_B = cand(ib,3);
fprintf('%d feasible pairs. Selected: Kp = %.5f, Ki = %.4e (Ti = %.0f s)\n', ...
        size(cand,1), Kp_B, Ki_B, Ti_B);
fprintf('Feasible set (Kp, Ki, Ti, PM, OS, Ts):\n');
disp(array2table(sortrows(cand,6), 'VariableNames', {'Kp','Ki','Ti','PM_deg','OS_pct','Ts_s'}));

%% ======================================================================
%  ANALYSIS OF BOTH DESIGNS (continuous loop, discrete loop, simulation)
%% ======================================================================
designs = struct('name', {'A: course method','B: grid search'}, ...
                 'Kp', {Kp_A, Kp_B}, 'Ki', {Ki_A, Ki_B});
Tend = 8000;                                   % long enough to see slow tails
z = tf('z', Ts);
res = struct();
for d = 1:2
    Kp = designs(d).Kp;  Ki = designs(d).Ki;

    % continuous loop (used for the 1e-6 ... 1e6 rad/s Bode plot)
    Cc = Kp + Ki/s;
    Lc = Cc*G;
    [Gm_c, Pm_c, Wcg_c, Wcp_c] = margin(Lc);

    % discrete loop: ZOH plant (27 s = 27 samples of delay) and the Arduino PI
    %   I(k) = I(k-1) + Ki*Ts*e(k);  u(k) = Kp*e(k) + I(k)
    Gz = c2d(G, Ts, 'zoh');
    Cz = Kp + Ki*Ts*z/(z-1);
    Lz = Cz*Gz;
    [Gm_z, Pm_z, Wcg_z, Wcp_z] = margin(Lz);
    Tcl = feedback(absorbDelay(Lz), 1);        % delay -> poles at z = 0
    pcl = pole(Tcl);                           % closed-loop poles (linear, no saturation)
    tau_slow = -Ts/log(max(abs(pcl)));         % decay time constant of the slowest mode [s]

    % nonlinear discrete simulation (like the real Arduino loop)
    [t, y, u] = sim_pi(Kp, Ki, K, tau, Ld, Ts, r, Tend);
    m = step_metrics(t, y, u, r);

    res(d).Cc = Cc; res(d).Lc = Lc; res(d).Lz = Lz;
    res(d).Gm_c = Gm_c; res(d).Pm_c = Pm_c; res(d).Wcg_c = Wcg_c; res(d).Wcp_c = Wcp_c;
    res(d).Gm_z = Gm_z; res(d).Pm_z = Pm_z; res(d).Wcg_z = Wcg_z; res(d).Wcp_z = Wcp_z;
    res(d).tau_slow = tau_slow;
    res(d).t = t; res(d).y = y; res(d).u = u; res(d).m = m;
end

%% ---------------- Comparison table in the command window ----------------
fprintf('\n=============== COMPARISON ===============\n');
fprintf('%-34s %16s %16s\n', '', 'A: course', 'B: grid search');
row = @(lbl, f, a, b) fprintf('%-34s %16s %16s\n', lbl, sprintf(f,a), sprintf(f,b));
row('Kp',                         '%.5f',  designs(1).Kp, designs(2).Kp);
row('Ki',                         '%.4e',  designs(1).Ki, designs(2).Ki);
row('Ti = Kp/Ki [s]',             '%.0f',  designs(1).Kp/designs(1).Ki, designs(2).Kp/designs(2).Ki);
row('PI zero Ki/Kp [rad/s]',      '%.3e',  designs(1).Ki/designs(1).Kp, designs(2).Ki/designs(2).Kp);
fprintf('--- continuous loop L = (Kp+Ki/s)*G ---\n');
row('crossover wc [rad/s]',       '%.3e',  res(1).Wcp_c, res(2).Wcp_c);
row('phase margin [deg]',         '%.1f',  res(1).Pm_c,  res(2).Pm_c);
row('gain margin [dB]',           '%.1f',  20*log10(res(1).Gm_c), 20*log10(res(2).Gm_c));
row('phase crossover [rad/s]',    '%.3e',  res(1).Wcg_c, res(2).Wcg_c);
fprintf('--- discrete loop (Ts = %g s, ZOH, Arduino PI) ---\n', Ts);
row('crossover [rad/s]',          '%.3e',  res(1).Wcp_z, res(2).Wcp_z);
row('phase margin [deg]',         '%.1f',  res(1).Pm_z,  res(2).Pm_z);
row('gain margin [dB]',           '%.1f',  20*log10(res(1).Gm_z), 20*log10(res(2).Gm_z));
row('slowest closed-loop pole tau [s]', '%.0f', res(1).tau_slow, res(2).tau_slow);
fprintf('--- step %.0f -> %.0f degC (discrete sim, saturation + anti-windup) ---\n', Tamb, Tref);
row('overshoot [%]',              '%.2f',  res(1).m.OS,  res(2).m.OS);
row('rise time 10-90 % [s]',      '%.0f',  res(1).m.t90 - res(1).m.t10, res(2).m.t90 - res(2).m.t10);
row('settling time 2 % [s]',      '%.0f',  res(1).m.Ts,  res(2).m.Ts);
row('peak duty',                  '%.3f',  res(1).m.umax, res(2).m.umax);
row('temperature at t = Ts_req [C]', '%.2f', Tamb + res(1).y(round(Ts_req/Ts)), Tamb + res(2).y(round(Ts_req/Ts)));
meets = @(m) (m.OS <= OS_req) && (m.Ts <= Ts_req);
fprintf('%-34s %16s %16s\n', 'meets OS and Ts spec?', ternary(meets(res(1).m)), ternary(meets(res(2).m)));

fprintf('\nGains to put in thermobox.ino / PID block:\n');
fprintf('  A: KP = %.5ff, KI = %.4ef\n', designs(1).Kp, designs(1).Ki);
fprintf('  B: KP = %.5ff, KI = %.4ef\n', designs(2).Kp, designs(2).Ki);

%% ---------------- Bode plots, 1e-6 ... 1e6 rad/s, with margins ----------------
w = logspace(-6, 6, 6000);
plot_bode_margins(res(1).Lc, w, sprintf('Method A (course), continuous loop: Kp=%.4f, Ki=%.2e', designs(1).Kp, designs(1).Ki));
plot_bode_margins(res(2).Lc, w, sprintf('Method B (grid search), continuous loop: Kp=%.4f, Ki=%.2e', designs(2).Kp, designs(2).Ki));

% discrete loops: defined only up to the Nyquist frequency pi/Ts
figure('Name','Discrete loop A','Color','w'); margin(res(1).Lz); grid on;
title(sprintf('Method A, discrete loop (Ts = %g s, Nyquist = %.2f rad/s)', Ts, pi/Ts));
figure('Name','Discrete loop B','Color','w'); margin(res(2).Lz); grid on;
title(sprintf('Method B, discrete loop (Ts = %g s, Nyquist = %.2f rad/s)', Ts, pi/Ts));

%% ---------------- Step responses ----------------
figure('Name','Step responses','Color','w');
subplot(2,1,1); hold on; grid on;
plot(res(1).t, Tamb + res(1).y, 'LineWidth', 1.5);
plot(res(2).t, Tamb + res(2).y, 'LineWidth', 1.5);
plot([0 Tend], [Tref Tref], 'k--');
plot([0 Tend], Tamb + 1.02*r*[1 1], ':', 'Color', [.4 .4 .4]);
plot([0 Tend], Tamb + 0.98*r*[1 1], ':', 'Color', [.4 .4 .4]);
plot(Ts_req*[1 1], [Tamb Tamb + 1.4*r], 'r--');
xlim([0 Tend]); ylabel('Temperature (°C)'); xlabel('t (s)');
legend('A: course', 'B: grid search', 'Setpoint', '±2 % band', '', 'Ts requirement', 'Location', 'southeast');
title('Discrete closed-loop step response (nonlinear: delay + duty limit + anti-windup)');
subplot(2,1,2); hold on; grid on;
plot(res(1).t, res(1).u, 'LineWidth', 1.5);
plot(res(2).t, res(2).u, 'LineWidth', 1.5);
xlim([0 Tend]); ylabel('Duty'); xlabel('t (s)'); legend('A', 'B');

%% =====================================================================
%                              LOCAL FUNCTIONS
%% =====================================================================
function [pm, wc] = pm_cont(Kp, Ki, K, tau, Ld)
% Phase margin and crossover of the continuous loop (Kp+Ki/s)*K*e^(-Ld s)/(tau s+1)^2
    w  = logspace(-6, 0, 3000);
    sj = 1i*w;
    L  = (Kp + Ki./sj) .* K ./ (tau*sj + 1).^2 .* exp(-sj*Ld);
    m  = log(abs(L));
    ph = unwrap(angle(L))*180/pi;
    i  = find(m < 0, 1);
    if isempty(i) || i == 1, pm = NaN; wc = NaN; return; end
    a  = m(i-1)/(m(i-1) - m(i));
    wc = exp(log(w(i-1)) + a*(log(w(i)) - log(w(i-1))));
    pm = 180 + ph(i-1) + a*(ph(i) - ph(i-1));
end

function [t, y, u] = sim_pi(Kp, Ki, K, tau, Ld, Ts, r, Tend)
% Discrete closed loop, mimics the Arduino code (pi_controller.cpp):
% conditional-integration anti-windup, duty limited to [0,1], plant ZOH-discretised
% with Ld/Ts samples of pure delay. y = temperature rise above ambient.
    sysd = c2d(ss(tf(K, conv([tau 1],[tau 1]))), Ts, 'zoh');
    [Ad, Bd, Cd] = ssdata(sysd);
    nd  = round(Ld/Ts);
    N   = round(Tend/Ts);
    x   = zeros(size(Ad,1), 1);
    buf = zeros(1, nd);
    I   = 0;
    y = zeros(N,1);  u = zeros(N,1);
    for k = 1:N
        yk = Cd*x;
        e  = r - yk;
        P  = Kp*e;
        uU = P + I;
        if ~((uU >= 1 && e > 0) || (uU <= 0 && e < 0))
            I = I + Ki*e*Ts;
        end
        I  = min(max(I, 0), 1);
        uk = min(max(P + I, 0), 1);
        buf = [buf uk];            %#ok<AGROW>  delay line
        ud  = buf(1);  buf(1) = [];
        x   = Ad*x + Bd*ud;
        y(k) = yk;  u(k) = uk;
    end
    t = (0:N-1)'*Ts;
end

function m = step_metrics(t, y, u, r)
    m.OS  = max(0, (max(y)/r - 1)*100);
    k     = find(abs(y - r) > 0.02*r, 1, 'last');
    if isempty(k),        m.Ts = 0;
    elseif k == numel(t), m.Ts = Inf;
    else,                 m.Ts = t(k+1);
    end
    i10 = find(y >= 0.1*r, 1);  i90 = find(y >= 0.9*r, 1);
    if isempty(i10), m.t10 = Inf; else, m.t10 = t(i10); end
    if isempty(i90), m.t90 = Inf; else, m.t90 = t(i90); end
    m.umax = max(u);
end

function plot_bode_margins(L, w, ttl)
% Bode plot with gain/phase margins marked.
    [mag, ph] = bode(L, w);  mag = squeeze(mag);  ph = squeeze(ph);
    [Gm, Pm, Wcg, Wcp] = margin(L);
    figure('Name', ttl, 'Color', 'w');

    subplot(2,1,1); semilogx(w, 20*log10(mag), 'LineWidth', 1.4); grid on; hold on;
    plot([1e-6 1e6], [0 0], 'k--');
    xlim([1e-6 1e6]); ylim([-260 80]); ylabel('Magnitude (dB)'); title(ttl);
    plot(Wcp, 0, 'ro', 'MarkerFaceColor', 'r');
    text(Wcp*1.5, 12, sprintf('\\omega_c = %.2e rad/s', Wcp), 'Color', 'r');
    if isfinite(Gm) && Wcg > 0
        plot([Wcg Wcg], [-20*log10(Gm) 0], 'm-', 'LineWidth', 2);
        plot(Wcg, -20*log10(Gm), 'ms', 'MarkerFaceColor', 'm');
        text(Wcg*1.5, -20*log10(Gm) - 12, sprintf('GM = %.1f dB @ %.2e rad/s', 20*log10(Gm), Wcg), 'Color', 'm');
    end

    subplot(2,1,2); semilogx(w, ph, 'LineWidth', 1.4); grid on; hold on;
    plot([1e-6 1e6], [-180 -180], 'k--');
    xlim([1e-6 1e6]); ylim([-540 0]); yticks(-540:90:0);
    ylabel('Phase (deg)'); xlabel('Frequency (rad/s)');
    plot([Wcp Wcp], [-180 -180 + Pm], 'r-', 'LineWidth', 2);
    plot(Wcp, -180 + Pm, 'ro', 'MarkerFaceColor', 'r');
    text(Wcp*1.5, -180 + Pm/2, sprintf('PM = %.1f° @ %.2e rad/s', Pm, Wcp), 'Color', 'r');
    if isfinite(Gm) && Wcg > 0
        plot(Wcg, -180, 'ms', 'MarkerFaceColor', 'm');
    end
end

function s = ternary(b)
    if b, s = 'YES'; else, s = 'NO'; end
end
