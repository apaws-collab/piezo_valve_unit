%% Data-Driven Pressure Control Based on Gaussian processes
%
% Author: Alexander Pawluchin
% Date: 04.08.2025
%
% Description:
% This script demonstrates a data-driven pressure controller
% based on Gaussian processes regression. All auxiliary functions 
% such as filtering, data interpolation, and motion generation are 
% included and should work out of the box.

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



%% Global Parameters
% Info: Execute this section before proceeding with the "Procedure" below.

addpath(genpath('./data'))           % Folder containing collected training data
addpath(genpath('./filtering'))      % Filter implementations
addpath(genpath('./motion_gen'))     % Smooth spline trajectory generation (based on Peter Corke’s toolbox)
addpath(genpath('./model_learning')) % Includes "valve_map" and "spa_adaptation" scripts

% Main control loop parameters
dt = 0.005; % Sampling time [s] for data acquisition and control loop

% Filtering parameters
filtFreqPres = 16; % Cutoff frequency [Hz] for smoothing pressure derivative

% Discrete filter for joint angle sensor
tmpfreq = 10;      % Cutoff frequency [Hz]
tmpdamp = 1;       % Damping, response without overshooting
disfilt.angle = c2d(tf((2*pi*tmpfreq)^2, ...
                      [1, 2*tmpdamp*(2*pi*tmpfreq), (2*pi*tmpfreq)^2]), dt);

% Lookup table (LUT) export parameters
vBlockParams = struct;
graphRes = 50;     % Resolution of LUT in each input dimension

% Plot colors
cldatain = [0.0745, 0.6235, 1.0000];      % Color for inlet valve data
cldataout = [1.0000, 0.4118, 0.1608];     % Color for outlet valve data
clu = [0.0745, 0.6235, 1.0000];           % Color for control input
cldata = [0.5020, 0.5020, 0.5020];        % General data color
clascent = [0.3922, 0.8314, 0.0745];      % Color for ascending trajectories
cldescent = [0.6353, 0.0784, 0.1843];     % Color for descending trajectories



%% Procedure
% 1. Run the valve mapping:
%    Use "valve_map()" in the "model_learning" folder to identify
%    the static flow characteristics of the inlet and outlet valves:
%           Input: chamber pressure p and rate dp
%           Output: control voltage u
%
%    The template guides you through:
%    - Loading the data
%    - Selecting and filtering training data
%    - Training the GP models
%    - Exporting results to LUTs

% 2. Run the SPA volume adaptation:
%    Use "spa_adaptation()" in the "model_learning" folder to learn
%    the actuator's volume-dependent dynamics:
%           Input: pressure rate mu/eta
%           Outout: pressure rate dp (measured)
%
%    The same workflow 
%    (load → train → export LUT) applies.