%% pi_design.m
% Automatic PI design (frequency-response method of the course).
% INPUT : plant G(s) and the overshoot requirement.
% OUTPUT: Kp, Ki, zero Ki/Kp, crossover wc, PM, GM, overshoot, Ts, poles + plots.
% Method: OS -> zeta -> PM target; for each candidate wc the PI is placed so
% that |L(jwc)| = 1 and the phase margin = PM (exact, no graph reading).
% The candidate with the shortest settling time that still meets OS is chosen.
% REQUIRES: Control System Toolbox
clear; close all; clc;

%% ===================== USER INPUT =====================
num    = 38.7;                  % plant numerator
den    = [125316 708 1];        % plant denominator
OS_max = 2;                     % maximum overshoot [%]
Ts_max = Inf;                   % optional: maximum settling time (2%) [s]
PM_extra = 0:5:20;              % extra PM tried on top of the 2nd-order estimate [deg]
dutyScale = 255;                % Arduino PWM scale (Kp,Ki x this for 0-255 output)

%% ===================== OS -> zeta -> PM =====================
G    = tf(num, den);
zeta = -log(OS_max/100) / sqrt(pi^2 + log(OS_max/100)^2);
PM0  = atand( 2*zeta / sqrt( sqrt(1 + 4*zeta^4) - 2*zeta^2 ) );   % exact 2nd-order
fprintf('OS_max = %.2f %%  ->  zeta = %.3f  ->  PM target = %.1f deg\n', OS_max, zeta, PM0);

%% ===================== SWEEP wc and PM =====================
p    = pole(G);  wref = min(abs(p(abs(p) > 0)));       % slowest plant pole
wcs  = wref * logspace(-2, 1, 150);
best = struct('ts', Inf);
for PM = PM0 + PM_extra
    for wc = wcs
        [Kp, Ki, ok] = designPI(G, wc, PM);
        if ~ok, continue, end
        T = feedback(tf([Kp Ki],[1 0]) * G, 1);
        if any(real(pole(T)) >= 0), continue, end
        si = stepinfo(T, 'SettlingTimeThreshold', 0.02);
        if ~isfinite(si.SettlingTime) || si.Overshoot > OS_max || si.SettlingTime > Ts_max
            continue
        end
        if si.SettlingTime < best.ts
            best = struct('ts', si.SettlingTime, 'Kp', Kp, 'Ki', Ki, 'wc', wc, ...
                          'PMd', PM, 'os', si.Overshoot, 'T', T);
        end
    end
end
if ~isfinite(best.ts)
    error('No PI found that meets OS_max / Ts_max. Relax the requirements.');
end

%% ===================== RESULTS =====================
C = tf([best.Kp best.Ki],[1 0]);   L = C*G;   T = best.T;
[Gm, Pm, Wcg, Wcp] = margin(L);
fprintf('\n===== RESULT =====\n');
fprintf('Kp = %.6g      Ki = %.6g      (PI zero Ki/Kp = %.5g rad/s)\n', best.Kp, best.Ki, best.Ki/best.Kp);
fprintf('Arduino (0-%d scale): Kp = %.4g, Ki = %.4g\n', dutyScale, best.Kp*dutyScale, best.Ki*dutyScale);
fprintf('Design: wc = %.5g rad/s, PM design = %.1f deg\n', best.wc, best.PMd);
fprintf('Check : PM = %.1f deg at %.5g rad/s, GM = %.3g (%.1f dB)\n', Pm, Wcp, Gm, 20*log10(Gm));
fprintf('Step  : overshoot = %.2f %%, Ts(2%%) = %.0f s\n', best.os, best.ts);
si = stepinfo(T);  fprintf('        rise time (10-90%%) = %.0f s\n', si.RiseTime);
fprintf('Bandwidth of T = %.5g rad/s,  peak Mr = %.3f\n', bandwidth(T), getPeakGain(T));
fprintf('Closed-loop poles:\n'); disp(pole(T));
fprintf('Closed-loop zero  : %.5g\n', zero(T));

%% ===================== PLOTS =====================
figure; margin(L); grid on
figure; nyquist(L); grid on
figure; pzmap(T); grid on
figure; step(T); grid on; title('Closed-loop step response')
figure; bode(L, T, 1/(1+L)); grid on; legend('L','T','S')

%% ===================== LOCAL FUNCTION =====================
function [Kp, Ki, ok] = designPI(G, wc, PM)
    % PI = Kp + Ki/s such that |L(jwc)| = 1 and angle(L(jwc)) = -180 + PM
    g  = squeeze(freqresp(G, wc));
    th = deg2rad(-180 + PM - rad2deg(angle(g)));     % required PI phase (rad)
    th = atan2(sin(th), cos(th));                    % wrap to (-pi, pi]
    m  = 1/abs(g);                                   % required |C(jwc)|
    Kp =  m*cos(th);
    Ki = -m*sin(th)*wc;
    ok = (th < 0) && (th > -pi/2) && Kp > 0 && Ki > 0;
end
