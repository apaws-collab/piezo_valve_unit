%% Volume adaptation
% Description:
% This script trains a data-driven volume adaptation model for pressure control in
% soft pneumatic actuators. It consists of:
%   1. Data collection using the "volume_adapt.slx" Simulink model
%   2. Training of GP models for volume adaptation (mu prediction)
%   3. Export of lookup tables (LUTs)

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



%% --- Data collection: run the simulink model ---
slbuild('volume_adapt');                            % Build model
load(dev.tg, 'volume_adapt');                       % Load model onto Speedgoat
start(dev.tg, 'StopTime', pDesIn1.time(end));       % Run until desired signal ends

runIDs = Simulink.sdi.getAllRunIDs();               % Get Simulink data log
Simulink.sdi.exportRun(runIDs(end), ...
    "to", "file", "filename", "data/PAM_splines_training");



%% --- Load and preprocess training data ---
pam = load('./data/PAM_splines_training.mat');

time = pam.data.get('p1').Values.Time;
tRange = [3 inf];                                   % Ignore startup transients
tSelector = (time >= tRange(1) & time <= tRange(2));
time = time(tSelector);
time = time - time(1);                              % time shift
dtRec = time(2) - time(1);                          % Sample time

% Pressure measurements and derivatives
p1 = pam.data.get('p1').Values.Data(tSelector);
[p1, dp1] = ff_forwardbackward(p1, filtFreqPres, dtRec);

p2 = pam.data.get('p2').Values.Data(tSelector);
[p2, dp2] = ff_forwardbackward(p2, filtFreqPres, dtRec);

% Control signals (mu from valve map)
mu1 = pam.data.get('mu1').Values.Data(tSelector);
mu2 = pam.data.get('mu2').Values.Data(tSelector);

% Raw valve commands
u1In  = pam.data.get('u1_in').Values.Data(tSelector);
u1Out = pam.data.get('u1_out').Values.Data(tSelector);
u2In  = pam.data.get('u2_in').Values.Data(tSelector);
u2Out = pam.data.get('u2_out').Values.Data(tSelector);

% Position states
phi1   = pam.data.get('phi1').Values.Data(tSelector);
dphi1  = pam.data.get('dphi1').Values.Data(tSelector);

% check data
figure
scatter3(dp1, phi1, mu1, 4, dphi1)
figure
scatter3(dp2, phi1, mu2, 4, dphi1)



%% --- Data selection ---
% Exclude saturated valve conditions
dataSelect = u1In < 1 & u1Out > -1 & u2In < 1 & u2Out > -1;

samplesPlot = 5000;
dsSample = round(sum(dataSelect) / samplesPlot);

% Downsample and extract final training inputs
p1Sel    = downsample(p1(dataSelect), dsSample);
dp1Sel   = downsample(dp1(dataSelect), dsSample);
p2Sel    = downsample(p2(dataSelect), dsSample);
dp2Sel   = downsample(dp2(dataSelect), dsSample);
phi1Sel  = downsample(phi1(dataSelect), dsSample);
dphi1Sel = downsample(dphi1(dataSelect), dsSample);
mu1Sel   = downsample(mu1(dataSelect), dsSample);
mu2Sel   = downsample(mu2(dataSelect), dsSample);

% Check data
figure
scatter(dp1Sel, mu1Sel, 4, phi1Sel)

figure
scatter(dp2Sel, mu2Sel, 4, phi1Sel)



%% --- Fit GP regression models ---
% PAM 1
X1 = [dp1Sel, phi1Sel, dphi1Sel];
y1 = mu1Sel;
gpVcomp1 = fitrgp(X1, y1, ...
    'KernelFunction','ardsquaredexponential', ...
    'BasisFunction','linear', ...
    'Fitmethod', 'exact');

% PAM 2
X2 = [dp2Sel, phi1Sel, dphi1Sel];
y2 = mu2Sel;
gpVcomp2 = fitrgp(X2, y2, ...
    'KernelFunction','ardsquaredexponential', ...
    'BasisFunction','linear', ...
    'Fitmethod', 'exact');



%% --- Export ---
% plots
dpMax = max(max(dp1), max(dp2));
dpMin = min(min(dp1), min(dp2));

dpFieldGrid = linspace(dpMin, dpMax, graphRes)';
phiField = linspace(min(phi1), max(phi1), graphRes)';
dphiField = linspace(min(dphi1), max(dphi1), graphRes)';

[x1grid, x2grid] = meshgrid(dpFieldGrid, phiField);
gridArranged = [x1grid(:), x2grid(:)];

dphiTest = 0;
dpPred1 = predict(gpVcomp1, [gridArranged dphiTest.*ones(length(gridArranged),1)]);
dpPred1Mesh = reshape(dpPred1, [graphRes, graphRes]);

dpPred2 = predict(gpVcomp2, [gridArranged dphiTest.*ones(length(gridArranged),1)]);
dpPred2Mesh = reshape(dpPred2, [graphRes, graphRes]);

figure;
tiledlayout(1,2, 'TileSpacing','Compact', 'Padding', 'Compact');
nexttile;
scatter3(X1(:,1), X1(:,2), y1, 3, 'filled', 'r')
hold on;
surf(x1grid, x2grid, dpPred1Mesh);
xlabel('eta1')
ylabel('phi')
zlabel('mu1')
grid on;

nexttile;
scatter3(X2(:,1), X2(:,2), y2, 3, 'filled', 'r')
hold on;
surf(x1grid, x2grid, dpPred2Mesh);
xlabel('eta2')
ylabel('phi')
zlabel('mu2')
grid on;

% prepare for simulink
[D1, D2, D3] = ndgrid(dpFieldGrid, phiField, dphiField);
shapedInputs = [D1(:), D2(:), D3(:)];
muPred1 = predict(gpVcomp1, shapedInputs);
muPred1LUT = reshape(muPred1, size(D1));

muPred2 = predict(gpVcomp2, shapedInputs);
muPred2LUT = reshape(muPred2, size(D1));

% Store as LUT structs
vBlockParams.valveblock1.eta   = dpFieldGrid;
vBlockParams.valveblock1.phi   = phiField;
vBlockParams.valveblock1.dphi  = dphiField;
vBlockParams.valveblock1.muPred = muPred1LUT;

vBlockParams.valveblock2.eta   = dpFieldGrid;
vBlockParams.valveblock2.phi   = phiField;
vBlockParams.valveblock2.dphi  = dphiField;
vBlockParams.valveblock2.muPred = muPred2LUT;