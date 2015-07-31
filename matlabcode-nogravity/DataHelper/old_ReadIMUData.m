function [timeList_IMU, accelList, angrtList, pivotTime] = ReadIMUData(dataFile)

fid = fopen(dataFile, 'r');
formatSpec = '%s %s %s %s %s %s %s %s';
rawData = textscan(fid, formatSpec);
fclose(fid);

%% Initialization
numLine = length(rawData{1});
timeList_IMU = zeros(1, numLine);
accelList = zeros(3, numLine);
angrtList = zeros(3, numLine);

pivotTime = datenum(rawData{1}{1});

%% Get IMU data
for ix = 1 : numLine
    % retrieve time stamp
    tmp = rawData{1}{ix};
	timeList_IMU(ix) = (datenum(tmp) - pivotTime) * 24 * 3600; % change to seconds
    
    % retrieve acceleration
    x = str2double(rawData{2}{ix}); y = str2double(rawData{3}{ix}); z = str2double(rawData{4}{ix});
    accelList(:, ix) = [x y z]';
    
    % retrieve angular rate
    x = str2double(rawData{5}{ix}); y = str2double(rawData{6}{ix}); z = str2double(rawData{7}{ix});
    angrtList(:, ix) = [x y z]';
end

end

%% Change from 'HH-mm-ss-SSS' to 'HH:mm:ss.FFF'
% function newTime = changeTimeFormat(oldTime)
% newTime = strrep(oldTime, '-', ':');
% pos = strfind(newTime, ':');
% newTime(pos(end)) = '.';
% end