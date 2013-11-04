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
% Assume that the amplitude change of color light is proportional to
% the change of ambient signal, multiplied by the ratio between two.
% !!! This doesn't really work ;( !!!
    
function adjusted = adjust_l(color, ambient)  
    % filter ambient signal
    ambient_m = medfilt1(ambient, 5);
    % shift/scale interpolation to match color signal
    function f = fshift_m(x, ref, model)
        f = sum((ref-(model+x)).^2);
    end
    function f = fscale_m(x, ref, model)
        f = sum((ref-(model*x)).^2);
    end
    shift = fminsearch(@(x) fshift_m(x, color, ambient_m), 1);
    scale = fminsearch(@(x) fscale_m(x, color, ambient_m+shift), 1);
    color_m = shift + ambient_m*scale;
    % substract model from color to get signal of interest
    adjusted = color - color_m;
end
