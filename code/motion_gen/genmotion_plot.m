function genmotion_plot(motion)
% Description: Plots the generated motion

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

dta = motion.traj1.values(2,1)-motion.traj1.values(1,1);
% text
if isempty(motion)
    error('motion stack is empty! Please use motion generator first')
end

figure('name', 'current motion stack')
tiledlayout(2,1, 'TileSpacing','Compact', 'Padding', 'Compact');

nexttile
nT = fieldnames(motion);
trajSize = numel(nT);
for i=1:trajSize
    timeline = motion.(nT{i}).values(:,1);
    desPos = motion.(nT{i}).values(:,2);
   
    plot(timeline, desPos', 'Linewidth', 1, 'Displayname', ['ref. ', nT{i}])
    hold on;
end
legend()
grid on; grid minor;

moTime = motiontime_estimator(motion);
title(['accumulated motion execution time : ', char(moTime), 'min'])

nexttile
nT = fieldnames(motion);
trajSize = numel(nT);
for i=1:trajSize
    timeline = motion.(nT{i}).values(:,1);
    desVel = motion.(nT{i}).values(:,3);
    plot(timeline, desVel', 'Linewidth', 1, 'Displayname', ['ref. ', nT{i}])
    hold on;
end

xlabel(['time in seconds, step size=', num2str(dta), ' s'])
legend()
grid on; grid minor;
end