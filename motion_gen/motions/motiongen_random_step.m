function motion = motiongen_random_step(motion, mparams, dta)
% Description: random step motion generator

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

% --- load motion
if ~isempty(motion)
    motionNumber = numel(fieldnames(motion))+1;
else
   motionNumber = 1; 
end

% --- load params
% min/max
x0 = mparams.x0;
xMin = mparams.xMin;
xMax = mparams.xMax;
dxMin = mparams.dxMin;
dxMax = mparams.dxMax;

% dynamic behavior
minMoveTime = mparams.minMoveTime;
maxMoveTime = mparams.maxMoveTime;

% static behavior
minWaitTime = mparams.minWaitTime;
maxWaitTime = mparams.maxWaitTime;
maxRunTime = mparams.maxRunTime;

% number of trajectories
numOfMotions = mparams.numOfMotions;

% adaptation
scaling = mparams.scale;

% internal
moveToZero_ = mparams.moveToZero;
stayConst_ = 5; % [s]

% --- motion generation
% start sequence
startSeq = [x0, x0 stayConst_ 0;];

for iMo = 1:numOfMotions
    sumTime = 0;
    trajStack = startSeq;
    while sumTime < maxRunTime
        endPt = (xMax-xMin)*rand() + xMin;
        startPt = trajStack(end,2);
        if abs(endPt-startPt) < dxMin || abs(endPt-startPt) > dxMax
           continue; 
        end
        moveTime = minMoveTime + (maxMoveTime-minMoveTime)*rand();
        waitTime = minWaitTime + (maxWaitTime-minWaitTime)*rand();
        midVel = abs(endPt-startPt)/moveTime;
        if midVel > dxMax
            continue;
        end
        newStep = [trajStack(end,2), endPt, moveTime, waitTime];
        trajStack = [trajStack; newStep];
        sumTime = sum(sum(trajStack(:,3:4)));
    end
    % add rest position
    trajStack = [trajStack; [trajStack(end,2) x0 moveToZero_ 1]];
    trajStack(:,1:2) = trajStack(:,1:2)*scaling;
    motion.(['traj', num2str(motionNumber)]).values = motion_splines_t(trajStack, dta);
end

%% plot
genmotion_plot(motion);
end