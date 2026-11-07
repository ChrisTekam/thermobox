%% time_response.m
% Plot time response from 0 to 2000 seconds for output 'temp'
% Saved as .fig and as a 500 dpi .png, matching academic styling.
clear; close all; clc;

%% ===================== USER INPUT =====================
% --- Simulation settings ---
t_final = 2000;          % simulation end time (seconds)
fname   = 'temp_response';
dpi     = 500;

% --- Figure adjustments ---
lwAdd   = 0.75;   % line width added to the default (pt)
fsAdd   = 1;      % font size added to the default (pt)
yPad    = 0.04;   % extra vertical spacing: 4% of the axis range above and below
xPad    = 0.00;   % extra horizontal spacing

%% ===================== SIMULATION & DATA EXTRACTION =====================
% Ensure your To Workspace block has 'Save format' set to 'Timeseries' 
% and variable name set to 'temp'.
out = sim('thermobox'); 

% Extract time and temperature data from the To Workspace timeseries object
t = out.temp.Time;
y = out.temp.Data;

%% ===================== PLOTTING =====================
hfig = figure;
plot(t, y, 'LineWidth', 1.0); % Base width, will be incremented by styleFigure
grid on
box on

% --- Axis Limits & Padding ---
xlim([0 t_final]);
padAxes(hfig, xPad, yPad);

% --- Labels & Title ---
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Temperature (°C)', 'FontWeight', 'bold');
title('System Temperature Response', 'FontWeight', 'bold');

% --- Apply Aesthetic Style ---
styleFigure(hfig, lwAdd, fsAdd);

% --- Save Outputs ---
savefig(hfig, [fname '.fig']);
print(hfig, fname, '-dpng', ['-r' num2str(dpi)], '-vector');

%% ===================== LOCAL FUNCTIONS =====================
% (must stay at the END of the script file)
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