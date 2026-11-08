d = readtable('run_20261028_120857.csv');
[t,i] = unique(d.time_s,'stable'); y = d.temperature(i);   % drops duplicated tail rows
ton = 30; toff = 490;                                      % toff: supply cut by hand
Tamb = mean(y(t<28)); dy = y - Tamb;
ts = (0:1:t(end))'; u = double(ts>=ton & ts<toff);
model = @(p) interp1(ts+p(3), lsim(tf(p(1),conv([p(2) 1],[p(2) 1])),u,ts), t,'linear',0);
p = lsqnonlin(@(p) model(p)-dy, [40 350 25], [1 10 0], [1000 3000 120]);
fprintf('K=%.1f  tau=%.0f s (x2)  delay=%.0f s\n', p);
G = tf(p(1),conv([p(2) 1],[p(2) 1]),'InputDelay',p(3));