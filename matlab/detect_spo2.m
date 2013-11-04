%
% Copyright (C) 2011 Maxim Osipov
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU Affero General Public License as published
% by the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Affero General Public License for more details.
%
% You should have received a copy of the GNU Affero General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
% NOTE: There are 2 ratios to calculate SpO2 in the literature:
%         (1) R = AC(Red)/DC(Red) / AC(IR)/DC(IR)
%         (2) R = log10(Iac,red) / log10(Iac,ir)
%       We will use formula (1) in our calculations.
    
function [spo2, raw] = detect_spo2(amb, red, ir, motion, meamon, old)
    % calculate SpO2 ratios
    ir_dc = mean(ir);
    red_dc = mean(red);
    ir_ac = sum((ir-ir_dc).^2)/length(ir);
    red_ac = sum((red-red_dc).^2)/length(red);
    % ...and finally...
    spo2 = 100 - (red_ac/red_dc) / (ir_ac/ir_dc);
    raw = ones(size(amb))*spo2;
end
