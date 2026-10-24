%% plots.m
% Bode (magnitude and phase as separate figures) and Nyquist plot of the
% OPEN-LOOP transfer function L(s) = C(s)*G(s), using MATLAB's built-in
% plotting functions. Each figure is saved as .fig and as a 500 dpi .png.
% REQUIRES: Control System Toolbox (R2016b or newer)

clear; close all; clc;

%% ===================== USER INPUT =====================

% --- PI controller gains ---
Kp = 0.028754;
Ki = 0.000203;

% --- Open-loop transfer function: L(s) = (Kp*s + Ki)/s * 96.7/(283*s + 1) ---
num = 96.7 * [Kp Ki];                            % numerator
den = conv([1 0], [283 1]);                      % denominator: s*(283*s + 1)

% --- Frequency range (rad/s) ---
wmin = 1e-6;
wmax = 1e6;

% --- Figure names (no extension) ---
fname_bode_mag   = 'bode_magnitude';
fname_bode_phase = 'bode_phase';
fname_nyquist    = 'nyquist';

dpi = 500;

% --- Figure adjustments ---
lwAdd   = 0.75;   % line width added to the default (pt)
fsAdd   = 3;      % font size added to the default (pt)
yPad    = 0.04;   % extra vertical spacing: 4% of the axis range above and below
nyqFsAdd = fsAdd - 1;   % Nyquist text 1 pt smaller than the other figures
bodeLblAdd = 2;  % extra pt for the Bode axis titles (on top of fsAdd)
nyqXPad = 0.10;   % extra horizontal spacing for the Nyquist plot (each side)

%% ===================== SYSTEM =====================
sys = tf(num, den);

%% ===================== BODE MAGNITUDE =====================
opts = bodeoptions;
opts.FreqUnits    = 'rad/s';
opts.PhaseVisible = 'off';                       % magnitude only
opts.YLimMode     = {'manual'; 'manual'};
opts.YLim         = {[-120 100]; [-120 100]};     % {magnitude (dB); phase (deg)}
hfig = figure;
bodeplot(sys, {wmin, wmax}, opts);
grid on
padAxes(hfig, 0, yPad);                          % vertical padding only (4%)
styleFigure(hfig, lwAdd, fsAdd);
boxAxes(hfig);                                   % boxed axes
enlargeLabels(hfig, bodeLblAdd);                     % larger Bode axis titles
savefig(hfig, [fname_bode_mag '.fig']);
print(hfig, fname_bode_mag, '-dpng', ['-r' num2str(dpi)], '-vector');

%% ===================== BODE PHASE =====================
opts = bodeoptions;
opts.FreqUnits  = 'rad/s';
opts.MagVisible = 'off';                         % phase only
opts.YLimMode   = {'manual'; 'manual'};
opts.YLim       = {[-190 -80]; [-190 -80]};
hfig = figure;
bodeplot(sys, {wmin, wmax}, opts);
grid on
padAxes(hfig, 0, yPad);                          % vertical padding only (4%)
styleFigure(hfig, lwAdd, fsAdd);
boxAxes(hfig);                                   % boxed axes
enlargeLabels(hfig, bodeLblAdd);                     % larger Bode axis titles
savefig(hfig, [fname_bode_phase '.fig']);
print(hfig, fname_bode_phase, '-dpng', ['-r' num2str(dpi)], '-vector');

%% ===================== NYQUIST =====================
nyqXLim = [-2.5 0.5];    % real axis window
nyqYLim = [-2.5 2.5];    % imaginary axis window

hfig = figure;
nyquistplot(sys, {wmin, wmax});
grid on
ax = visibleAxes(hfig);
for k = 1:numel(ax)
    ax(k).XLim = nyqXLim;
    ax(k).YLim = nyqYLim;
end
padAxes(hfig, nyqXPad, yPad);                    % 10% horizontal, 4% vertical
styleFigure(hfig, lwAdd, nyqFsAdd);
boxAxes(hfig);                                   % boxed axes
savefig(hfig, [fname_nyquist '.fig']);
print(hfig, fname_nyquist, '-dpng', ['-r' num2str(dpi)], '-vector');

%% ===================== LOCAL FUNCTIONS =====================
% (must stay at the END of the script file)

function ax = visibleAxes(hfig)
    % all visible axes of the figure (hidden bode axes are skipped)
    ax = findall(hfig, 'Type', 'axes');
    keep = false(size(ax));
    for k = 1:numel(ax)
        keep(k) = strcmp(char(ax(k).Visible), 'on');
    end
    ax = ax(keep);
end

function boxAxes(hfig)
    % boxed axes (matches time_response.m)
    ax = visibleAxes(hfig);
    for k = 1:numel(ax)
        box(ax(k), 'on');
    end
end

function enlargeLabels(hfig, extra)
    % make the axis titles (x/y labels) 'extra' pt larger than the current size
    fo = findall(hfig, '-property', 'XLabel');
    for k = 1:numel(fo)
        try, fo(k).XLabel.FontSize = fo(k).XLabel.FontSize + extra; catch, end
        try, fo(k).YLabel.FontSize = fo(k).YLabel.FontSize + extra; catch, end
    end
end

function padAxes(hfig, xPad, yPad)
    % extend axis limits by a fraction of the current range on each side
    ax = visibleAxes(hfig);
    for k = 1:numel(ax)
        if xPad > 0
            xl = ax(k).XLim;  r = diff(xl);
            ax(k).XLim = xl + xPad*r*[-1 1];
        end
        yl = ax(k).YLim;  r = diff(yl);
        ax(k).YLim = yl + yPad*r*[-1 1];
    end
end

function styleFigure(hfig, lwAdd, fsAdd)
    % thicker lines and larger text (relative to the current values)
    ln = findall(hfig, 'Type', 'line');
    for k = 1:numel(ln)
        ln(k).LineWidth = ln(k).LineWidth + lwAdd;
    end
    fo = findall(hfig, '-property', 'FontSize');
    sz = zeros(size(fo));
    for k = 1:numel(fo)
        sz(k) = fo(k).FontSize;                  % read all sizes first
    end
    for k = 1:numel(fo)
        fo(k).FontSize = sz(k) + fsAdd;
    end
end
