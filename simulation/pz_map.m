%% pz_map.m
% Pole-zero map (s-plane) of the CLOSED-LOOP transfer function T(s), using
% MATLAB's built-in pzmap. Every pole and zero gets a small label with its
% location. Saved as .fig and as a 500 dpi .png.
% REQUIRES: Control System Toolbox (R2016b or newer)

clear; close all; clc;

%% ===================== USER INPUT =====================

% --- PI controller gains ---
Kp = 0.022257;
Ki = 0.000141;

% --- Closed-loop transfer function (unity negative feedback) ---
%   T(s) = 96.7*(Kp*s + Ki) / ( 283*s^2 + (1 + 96.7*Kp)*s + 96.7*Ki )
num = 96.7 * [Kp Ki];                            % numerator
den = [283, 1 + 96.7*Kp, 96.7*Ki];               % denominator

% --- Figure name (no extension) ---
fname_pzmap = 'pole_zero_locations';

dpi = 500;

% --- Figure adjustments ---
lwAdd   = 0.75;   % line width added to the default (pt)
fsAdd   = 1;      % font size added to the default (pt)
yPad    = 0.08;   % extra vertical spacing: 8% of the axis range above and below
lblGap  = 0.04;   % distance between marker and label (fraction of y-range)

%% ===================== SYSTEM =====================
sys = tf(num, den);
pol = pole(sys);
zer = zero(sys);

disp('Closed-loop poles:');  disp(pol);
disp('Closed-loop zeros:');  disp(zer);

%% ===================== POLE-ZERO MAP =====================
hfig = figure;
pzmap(sys);
grid on
styleFigure(hfig, lwAdd, fsAdd);                 % thicker lines, larger text

% --- labels next to every pole and zero ---
ax  = visibleAxes(hfig);  ax = ax(1);
pts = [pol(:); zer(:)];
gap = lblGap * diff(ax.YLim);
lbl = gobjects(numel(pts), 1);
for k = 1:numel(pts)
    z = pts(k);
    if abs(imag(z)) < 1e-9*max(1, abs(z)), z = real(z); end   % clean up tiny imaginary parts
    if imag(z) >= 0
        va = 'bottom';  dy =  gap;               % label above the marker
    else
        va = 'top';     dy = -gap;               % label below the marker
    end
    lbl(k) = text(ax, real(z), imag(z) + dy, fmtc(z), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', va, ...
        'FontSize', ax.FontSize);
end

% --- make sure the labels fit inside the axes ---
xl = ax.XLim;  yl = ax.YLim;
for k = 1:numel(lbl)
    e  = lbl(k).Extent;                          % [x y width height] in data units
    xl = [min(xl(1), e(1)), max(xl(2), e(1)+e(3))];
    yl = [min(yl(1), e(2)), max(yl(2), e(2)+e(4))];
end
ax.XLim = xl;  ax.YLim = yl;
padAxes(hfig, yPad);                       % extra spacing

savefig(hfig, [fname_pzmap '.fig']);
print(hfig, fname_pzmap, '-dpng', ['-r' num2str(dpi)], '-vector');

disp('Done: pole-zero figure saved (.fig and .png).');

%% ===================== LOCAL FUNCTIONS =====================
% (must stay at the END of the script file)

function s = fmtc(z)
    % complex number as text: xxx + yyy i
    if imag(z) >= 0, sg = '+'; else, sg = '-'; end
    s = sprintf('%.4g %s %.4gi', real(z), sg, abs(imag(z)));
end

function ax = visibleAxes(hfig)
    % all visible axes of the figure
    ax = findall(hfig, 'Type', 'axes');
    keep = false(size(ax));
    for k = 1:numel(ax)
        keep(k) = strcmp(char(ax(k).Visible), 'on');
    end
    ax = ax(keep);
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
