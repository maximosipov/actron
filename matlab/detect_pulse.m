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
% NOTE: Pulse is detected using average inter-peak distance for 10 seconds.
    
function [pulse, raw, peaks] = detect_pulse(amb, red, ir, motion, meamon)
    % use last 10 seconds
    pulse_time = 10*meamon.settings.device.sampling;
    pulse_start = length(ir) - pulse_time + 1;
    pulse_end = length(ir);
    % rescale and filter signals (0.2 seconds window)
    ir_z = zscore(ir(pulse_start:pulse_end));
    red_z = zscore(red(pulse_start:pulse_end));
    win_f = ceil(0.2*meamon.settings.device.sampling);
    ir_f = filter(ones(1,win_f)/win_f, 1, ir_z);
    red_f = filter(ones(1,win_f)/win_f, 1, red_z);
    % count peaks and average inter-peak distance
    function p = func_peaks(x)
        x_peaks = [];
        % 0.3 seconds window (we may actually want to implement adaptive
        % window, i.e. don't allow HR to change for more than 10% of mean)
        x_win = 30;
        % 1 - look for max, 2 - look for min
        if max(x(1:x_win)) < max(x(x_win:x_win*2)),
            x_search = 1;
        else
            x_search = 2;
        end
        i = 1;
        while i < length(x)-x_win,
            if x_search == 1,
                if x(i) <= max(x(i+1:i+x_win)),
                    % set i to last highest element in the window
                    x_tmp = (x(i+1:i+x_win) >= max(x(i+1:i+x_win)));
                    i = i+find(x_tmp, 1, 'last');
                else
                    x_peaks = [x_peaks i];
                    x_search = 2;
                end
            else
                if x_search == 2,
                    if x(i) >= min(x(i+1:i+x_win)),
                        % set i to last lowest element in the window
                        x_tmp = (x(i+1:i+x_win) <= min(x(i+1:i+x_win)));
                        i = i+find(x_tmp, 1, 'last');
                    else
                        x_search = 1;
                    end
                end
            end
        end
        p = x_peaks;
    end
    raw = (ir_f+red_f)./2;
    peaks = func_peaks(raw);
    peaks_d = peaks(2:length(peaks)) - peaks(1:length(peaks)-1);
    pulse = meamon.settings.device.sampling / mean(peaks_d) * 60;
end
