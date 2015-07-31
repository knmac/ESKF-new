function [timeList_IMU, accelList, angrtList, diffQuatList] = ReadIMUData(dataFile, pivotTime_IMG, GYRO_SENSTV, UNIT_G, gravity_scale, gravity_bias)

rawData = load(dataFile);
switch size(rawData, 2)
    case 7
        HAS_QUAT = 0;
    case 11
        HAS_QUAT = 1;
end

% LATENCY = 1.73*10^-3;
LATENCY = 0;

timeList_IMU = (rawData(:,1) - pivotTime_IMG - LATENCY)';
accelList = rawData(:, 2:4)';
accelList = bsxfun(@minus, accelList, gravity_bias);
accelList = gravity_scale * accelList * UNIT_G;
angrtList = rawData(:, 5:7)' * GYRO_SENSTV;

if HAS_QUAT
    diffQuatList = rawData(:, 8:11)';
end

end