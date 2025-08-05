function [ffdata, dffdata] = ff_forwardbackward(data, freq, dta)
% Description: forward backward filter based on zero overshoot IIR filter

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

% window left and right
WINDOWFILT = 1000;

T = 1/(2*pi*freq);
filtobj = c2d(tf(1,[T^2 2*T, 1]),dta);

qfnum = filtobj.num{:};
qfden = filtobj.den{:};

% row-wise
[m,n] = size(data);
if m ~= 1 && n ~= 1
    error('wrong vector size!')
end
if m > n
    data = transpose(data);
end


% forward-backward filtering
dataWindow = [ones(1,WINDOWFILT)*data(1), data, ones(1,WINDOWFILT)*data(end)];
filterstep1 = flip(filter(qfnum, qfden, dataWindow));
filterstep2 = flip(filter(qfnum, qfden, filterstep1));
ffdata = filterstep2(WINDOWFILT+1:end-WINDOWFILT);

if m > n
    ffdata = transpose(ffdata);
end

% derivative
dffdata = diff(ffdata)/dta;
dffdata(1) = 0;
dffdata(end+1) = dffdata(end);
end