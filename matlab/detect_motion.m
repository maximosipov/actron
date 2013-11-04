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
    
function motion = detect_motion(amb, ir, red, meamon)
    % calculate mean using 1 second window
    win_a = meamon.settings.device.sampling;
    mean_a = filter(ones(1,win_a)/win_a, 1, amb);
    mean_a = [ones(win_a,1)*mean_a(win_a);...
              mean_a(win_a+1:length(mean_a))];
    % find variance
    function f = func_v(x1,x2)
        if x2 == 0,
            f = 1;
        else
            f = abs(x1-x2)/x2;
        end
    end
    motion = arrayfun(@(x1,x2)func_v(x1,x2), amb, mean_a);
    motion = motion*100;
end
    