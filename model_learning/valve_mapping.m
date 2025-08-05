%% Valve Flow Mapping
% Description:
% This script consists of three main stages: data collection, model training,
% and export. It provides a template for implementing a data-driven pressure
% control scheme based on Gaussian processes.

% This file is part of piezo_valve_unit.
% Copyright (C) 2025 Alexander Pawluchin
%
% This program is free software: you can redistribute it and/or modify it
% under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.



%% --- Data Collection ---
% Description:
% The data was collected on a Speedgoat target machine connected to the valves and PAMs.
% "p" is the measured chamber pressure.
% The command signals "u" are defined as:
%   u_in  in [0, 1] (normalized inlet command)
%   u_out in [-1, 0] (normalized outlet command)
%
% 1. Please review the "valve_map.slx" Simulink file.
%
%   Procedure:
%   The valve input signals are bandwidth-limited via a second-order low-pass
%   filter with a 2 Hz cutoff to avoid beam dynamics of the piezo valves.

tmpdamp = 1;
tmpfreq = 2;
disfilt.ulimit = c2d(tf((2*pi*tmpfreq)^2, ...
    [1 2*tmpdamp*(2*pi*tmpfreq) (2*pi*tmpfreq)^2]), dt);

% In "valve_map.slx", a uniform random signal is used to generate desired
% pressure setpoints. These signals are converted to linearized pressure
% trajectories. A simple feedback loop with gain k = 1 is used to control
% the chamber pressure. The helper function "startStopFun" ensures
% consistent data collection intervals.

% Building and deploying the model (Matlab):
slbuild('valve_map');                 % Build application
load(target, 'valve_map');            % Load model onto Speedgoat

start(dev.tg, 'StopTime', 300);       % Run controller for 300 s
runIDs = Simulink.sdi.getAllRunIDs;   % Get IDs from logged data

% Save latest data to file
Simulink.sdi.exportRun(runIDs(end), "to", "file", ...
    "filename", "data/valveblock1");

% Repeat process, for each valve pair (inlet and outlet).



%% --- Training Process ---
% The script below processes data consistent with "valve_map.slx"

valvelblock = 1;  % Select the valve block number, in my case 1 (PAM 1) or 2 (PAM 2)
vindex = num2str(valvelblock);
valveIdentPath = ['./data/valveblock', vindex, '.mat'];
readFile = load(valveIdentPath);

tRange = [1 180];  % Data selection window [s]

% Extract and preprocess time vector
timeraw = readFile.data.get('p1').Values.Time;
timeSelector = timeraw > tRange(1) & timeraw < tRange(2);
time = timeraw(timeSelector);
time = time - time(1);  % Time offset removal
dtRec = time(2) - time(1);

% Retrieve and filter valve-specific data
p = readFile.data.get(['p', vindex]).Values.Data(timeSelector);
[p, dp] = ff_forwardbackward(p, filtFreqPres, dtRec);

uIn  = readFile.data.get(['u', vindex, '_in']).Values.Data(timeSelector);
uOut = readFile.data.get(['u', vindex, '_out']).Values.Data(timeSelector);


% --- Data Selection and Preprocessing ---
% Separate data based on pressure derivative thresholds
thdp = 0.2;
thu  = 0.2;

dataSelectp = (dp >  thdp) & (uIn  >  thu);
dataSelectn = (dp < -thdp) & (uOut < -thu);

% Downsample selected training points
samplesPlot = 4000;
dsSamplep = round(sum(dataSelectp) / samplesPlot);
dsSamplen = round(sum(dataSelectn) / samplesPlot);

x1p = downsample(p(dataSelectp), dsSamplep);
x1n = downsample(p(dataSelectn), dsSamplen);
x2p = downsample(dp(dataSelectp), dsSamplep);
x2n = downsample(dp(dataSelectn), dsSamplen);
up  = downsample(uIn(dataSelectp), dsSamplep);
un  = downsample(uOut(dataSelectn), dsSamplen);

% Plot selected samples
figure
scatter3(x1p, x2p, up, 20, 'filled', 'MarkerEdgeColor', clascent, ...
    'MarkerFaceColor', clascent);
scatter3(x1n, x2n, un, 20, 'filled', 'MarkerEdgeColor', cldescent, ...
    'MarkerFaceColor', cldescent);


% --- GP Regression ---
Xp = [x1p, x2p];
yp = up;
disp(['Training inlet GP with ', num2str(length(yp)), ' samples...'])
gpInvp = fitrgp(Xp, yp, ...
    'KernelFunction', 'ardsquaredexponential', ...
    'FitMethod', 'exact', 'BasisFunction', 'linear');

Xn = [x1n, x2n];
yn = un;
gpInvn = fitrgp(Xn, yn, ...
    'KernelFunction', 'ardsquaredexponential', ...
    'FitMethod', 'exact', 'BasisFunction', 'linear');

% --- LUT generation ---
% Depending on LUT implementation it is advisible to cap the command signal
pFieldCalc = linspace(0, max(p(:,1)), graphRes);

% Inlet side
dpFieldp = linspace(0, max(Xp(:,2)), graphRes);
[x1gridp, x2gridp] = meshgrid(pFieldCalc, dpFieldp);
gridArrangedp = [x1gridp(:), x2gridp(:)];
predSysp = predict(gpInvp, gridArrangedp);
uPredp = reshape(predSysp, [length(pFieldCalc), length(dpFieldp)]);
uPredp = max(0, min(uPredp, 1)); % cap

% Outlet side
dpFieldn = linspace(min(Xn(:,2)), 0, graphRes);
[x1gridn, x2gridn] = meshgrid(pFieldCalc, dpFieldn);
gridArrangedn = [x1gridn(:), x2gridn(:)];
predSysn = predict(gpInvn, gridArrangedn);
uPredn = reshape(predSysn, [length(pFieldCalc), length(dpFieldn)]);
uPredn = max(-1, min(uPredn, 0)); % cap

% Visualize prediction surfaces
figure; hold on;
surf(x1gridp, x2gridp, uPredp);
scatter3(Xp(:,1), Xp(:,2), yp, 'filled', 'r')

surf(x1gridn, x2gridn, uPredn);
scatter3(Xn(:,1), Xn(:,2), yn, 'filled', 'r')

xlabel('pressure in bar')
ylabel('diff. pressure in bar/s')
zlabel('input normalized')
zlim([-1, 1])
grid on;

% --- Export LUT ---
vBlockParams.(['valveblock', vindex]).pField = pFieldCalc;
vBlockParams.(['valveblock', vindex]).dpField.pos = dpFieldp;
vBlockParams.(['valveblock', vindex]).dpField.neg = dpFieldn;
vBlockParams.(['valveblock', vindex]).uPred.pos = uPredp;
vBlockParams.(['valveblock', vindex]).uPred.neg = uPredn;