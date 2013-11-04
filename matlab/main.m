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

clear all; close all;

%=========================================================================
% Initial configuration data

% device settings
meamon.settings.device.sampling = 150;
meamon.settings.device.channels = 8;
% alarms configuration
meamon.settings.alarm.sound = true;
meamon.settings.alarm.vibrate = false;
meamon.settings.alarm.sms = false;
meamon.settings.alarm.call = false;
meamon.settings.alarm.numbers = [ 447909637667, 123456789012 ];
% baby configuration
meamon.settings.baby.birth = '01-JAN-2011';
meamon.settings.baby.weight = 1000;
meamon.settings.baby.height = 53;
% data sharing
meamon.settings.data.share = true;
% monitor
meamon.monitor.time = 20;
meamon.monitor.data = zeros(meamon.settings.device.sampling*...
    meamon.monitor.time,...
    meamon.settings.device.channels);
meamon.monitor.spo2 = zeros(meamon.monitor.time, 1);
meamon.monitor.pulse = zeros(meamon.monitor.time, 1);
meamon.monitor.motion = zeros(meamon.monitor.time, 1);
% statistics
meamon.statistics.time = 14*24*60*60;
meamon.statistics.apnea = zeros(meamon.statistics.time, 1);
meamon.statistics.activity = 0;
meamon.statistics.weigth = ones(meamon.statistics.time, 1)*...
    meamon.settings.baby.weight;
meamon.statistics.heigth = ones(meamon.statistics.time, 1)*...
    meamon.settings.baby.height;


%=========================================================================
% Initialization
s = serial('/dev/ttyS999', 'BaudRate', 115200, 'Terminator','CR/LF');
fopen(s);
c = onCleanup(@()fclose(s));
s.ReadAsyncMode = 'continuous';
h = ui();
hs = guihandles(h);

%=========================================================================
% Main processing loop
while 1,
    pause(0.01);
    meamon.monitor.data = dataif(s, meamon.monitor.data);
    amb_raw = meamon.monitor.data(:,1);
    ir_raw = meamon.monitor.data(:,2);
    red_raw = meamon.monitor.data(:,3);
    temp_raw = meamon.monitor.data(:,4);
    hum_raw = meamon.monitor.data(:,5);
    % plot all data for debug purpose
    semilogy(hs.axes_data, amb_raw, 'g');
    hold(hs.axes_data, 'on');
    semilogy(hs.axes_data, ir_raw, 'b');
    semilogy(hs.axes_data, red_raw, 'r');
    hold(hs.axes_data, 'off');
    ylim(hs.axes_data, [0 100000]);
    xlim(hs.axes_data, [1 length(amb_raw)]);
    legend(hs.axes_data, 'Ambient', 'IR', 'Red');
    % get last 10 seconds
    frame_time = 10;
    frame_start = meamon.settings.device.sampling*meamon.monitor.time -...
        meamon.settings.device.sampling*frame_time;
    frame_end = meamon.settings.device.sampling*meamon.monitor.time;
    amb = amb_raw(frame_start:frame_end);
    ir = ir_raw(frame_start:frame_end);
    red = red_raw(frame_start:frame_end);
    temp = temp_raw(frame_start:frame_end);
    hum = hum_raw(frame_start:frame_end);
    % Motion detection
    motion = detect_motion(amb, ir, red, meamon);
    plot(hs.axes_motion, motion, 'g');
    ylim(hs.axes_motion, [0 100]);
    xlim(hs.axes_motion, [1 length(motion)]);
    set( hs.text_motion, 'String',...
        ['Motion: ',int2str(motion(length(motion))),' %'] );
    % Pulse detection
    [pulse, raw, peaks] = detect_pulse(amb, ir, red, motion, meamon);
    if isnan(pulse)
        set( hs.text_hr, 'String','Heart Rate: ? BPM' );
    else
        plot(hs.axes_hr, raw, 'b');
        hold(hs.axes_hr, 'on');
        scatter(hs.axes_hr, peaks, raw(peaks));
        hold(hs.axes_hr, 'off');
        ylim(hs.axes_hr, [min(raw)-1 max(raw)+1]);
        xlim(hs.axes_hr, [1 length(raw)]);
        set( hs.text_hr, 'String',...
            ['Pulse: ',int2str(pulse),' BPM'] );
    end
    % SpO2 detection
    [spo2, raw] = detect_spo2(amb, ir, red, motion, meamon);
    if isnan(spo2)
        set( hs.text_spo2, 'String','SpO2: ? %' );
    else
        plot(hs.axes_spo2, raw, 'b');
        ylim(hs.axes_spo2, [min(raw)-1 max(raw)+1]);
        xlim(hs.axes_spo2, [1 length(raw)]);
        set( hs.text_spo2, 'String',...
            ['SpO2: ',int2str(spo2),' %'] );
    end
    % Temperature
    plot(hs.axes_temp, temp, 'r');
    ylim(hs.axes_temp, [-50 50]);
    xlim(hs.axes_temp, [1 length(temp)]);
    set(hs.text_temp, 'String',...
        ['Temperature: ', num2str(temp(length(temp)), '%g'), ' C'] );
    % Humidity
    plot(hs.axes_hum, hum, 'b');
    ylim(hs.axes_hum, [0 100]);
    xlim(hs.axes_hum, [1 length(hum)]);
    set(hs.text_hum, 'String',...
        ['Humidity: ', num2str(hum(length(hum)), '%g'), ' %'] );
end


%=========================================================================
% Clean-up
fclose(s);
