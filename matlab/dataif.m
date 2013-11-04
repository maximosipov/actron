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

function dataout = dataif(s, datain)
    dataout = zeros(size(datain));
    old_recs = size(datain, 1);
    new_recs = 0;
    % read in the buffered input data
    while s.BytesAvailable > 0,
        new_recs = new_recs + 1;
        datatmp = fscanf(s, '%i,%i,%i,%f,%f,%i,%i,%i\n');
        if length(datatmp) == size(dataout,2),
            dataout(new_recs,:) = datatmp;
        else
            fprintf('Bad data!\n');
        end
    end
    % dataout = filter(ones(1,5)/5,1,dataout(1:new_recs, :));
    % prepare the output data
    if new_recs == 0,
        dataout = datain;
    elseif new_recs >= old_recs,
        dataout = dataout(((new_recs-old_recs+1):new_recs), :);
        fprintf('Overrun!\n');
    else
        dataout = [ datain((new_recs+1):old_recs, :);...
                    dataout(1:new_recs, :) ];
    end
end
