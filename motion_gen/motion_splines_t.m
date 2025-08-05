function refMotion = motion_splines_t(tra, h)
% Description: Motion spline assembly, based on Peter Corke's vision and
% control toolbox (jtraj function), see http://www.petercorke.com

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

% --- description
% [m, n] = size(tra)
% m is the number of spline steps
% n = 4, includes [startpoint, endpoint, movetime, waittime (wait after motion)]
% h: sampling rate [s] for velocity and acceleration

% --- check input
[numOfSteps, stackSize] = size(tra);
if stackSize ~= 4
    error('input trajectory stack has wrong size!')
end

xRef = [];
for i=1:numOfSteps
    xRef = [xRef; tra(i,1) tra(i,2) tra(i,3); tra(i,2) tra(i,2) tra(i,4)];
end
% remove zero lines
xRef = xRef(xRef(:,3)>0,:);
[xindex, ~] = size(xRef); % size of reference
refMotion = [];
for i = 1:xindex
    tVec = 0:xRef(i,3)/h;
    [q, qd, qdd] = jtraj(xRef(i,1), xRef(i,2), tVec); % start, stop, timevector
    refMotion = [refMotion; q, qd./h, qdd./h^2];
end
time = (0:length(refMotion)-1).*h;
refMotion = [time', refMotion];
end
