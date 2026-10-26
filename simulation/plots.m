%% Bode and Nyquist plots for the thermal control loop (PI + plant)
clear; clc; close all;

%% ---- User inputs ----
Kp = 0.028754;
Ki = 0.000203;

lw   = 1.5;          % line width [pt]
fs   = 11;           % font size [pt]
dpi  = 500;          % export resolution

wmin = 1e-6;         % rad/s
wmax = 1e6;          % rad/s
w    = logspace(log10(wmin), log10(wmax), 5000);

% Nyquist display window (zoomed near the -1 point so low-frequency
% blow-up from the integrator does not dominate the plot)
nyqXlim = [-4 1.5];
nyqYlim = [-4 4];

% Line color matching the default Bode plot line
lineColor = [0 0.4470 0.7410];

%% ---- Transfer functions ----
s = tf('s');
C = (Kp*s + Ki)/s;           % PI controller
G = 96.7/(283*s + 1);        % plant
L = C*G;                     % open-loop transfer function

%% ---- Stability margins ----
[Gm, Pm, Wcg, Wcp] = margin(L);

%% ---- Bode plot with margins ----
figBode = figure('Name','Bode Plot','Color','w');
margin(L, w);                % Bode plot + gain/phase margin annotations
grid on;

fontsize(figBode, fs, 'points');
set(findall(figBode, 'Type', 'line'), 'LineWidth', lw);

exportgraphics(figBode, 'bode_plot.png', 'Resolution', dpi);

%% ---- Nyquist plot (open-loop) ----
% Computed manually so the axes can be zoomed around the -1 point.
resp = squeeze(freqresp(L, w));
resp = resp(:);
re = real(resp);
im = imag(resp);

figNyq = figure('Name','Nyquist Plot','Color','w');
ax = axes(figNyq); hold(ax, 'on'); box(ax, 'on');

% Positive frequencies and mirror image for negative frequencies (both solid)
plot(ax, re,  im, '-', 'Color', lineColor, 'LineWidth', lw);
plot(ax, re, -im, '-', 'Color', lineColor, 'LineWidth', lw);

% Direction arrow on the positive-frequency branch (increasing w)
inView = find(re > nyqXlim(1) & re < nyqXlim(2) & abs(im) < nyqYlim(2));
if numel(inView) > 10
    k = inView(round(numel(inView)*0.5));
    quiver(ax, re(k), im(k), re(k+1)-re(k), im(k+1)-im(k), 0, ...
        'Color', lineColor, 'LineWidth', lw, 'MaxHeadSize', 2000, 'AutoScale', 'off');
end

% Unit circle (the curve crosses it at the gain crossover frequency)
th = linspace(0, 2*pi, 400);
plot(ax, cos(th), sin(th), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);

% Gain crossover point (phase margin is the angle from here to the -1 axis)
if isfinite(Pm) && isfinite(Wcp)
    Lc = squeeze(freqresp(L, Wcp));
    plot(ax, real(Lc), imag(Lc), 'ks', 'MarkerSize', 7, ...
        'MarkerFaceColor', 'k');
end

% Axes through the origin
xline(ax, 0, 'k-', 'LineWidth', 0.5);
yline(ax, 0, 'k-', 'LineWidth', 0.5);

% Critical point -1
plot(ax, -1, 0, 'r+', 'MarkerSize', 14, 'LineWidth', 2);
plot(ax, -1, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
text(ax, -1, 0.25, '-1', 'Color', 'r', 'FontSize', fs, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');

grid(ax, 'on');
axis(ax, 'equal');
xlim(ax, nyqXlim); ylim(ax, nyqYlim);

xlabel(ax, 'Real Axis');
ylabel(ax, 'Imaginary Axis');
title(ax, 'Nyquist Plot');

fontsize(figNyq, fs, 'points');

exportgraphics(figNyq, 'nyquist_plot.png', 'Resolution', dpi);

%% ---- Print stability margins ----
if isfinite(Gm) && Gm > 0
    fprintf('Gain margin  : %.2f dB at %.4g rad/s\n', 20*log10(Gm), Wcg);
else
    fprintf('Gain margin  : infinite (phase never reaches -180 deg)\n');
end
if isfinite(Pm)
    fprintf('Phase margin : %.2f deg at %.4g rad/s\n', Pm, Wcp);
else
    fprintf('Phase margin : none (no gain crossover)\n');
end