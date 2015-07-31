function [timeList_IMU, accelList, angrtList] = ReadIMUData(dataFile, pivotTime_IMG, GYRO_SENSTV, UNIT_G, gravity_scale, gravity_bias)

rawData = load(dataFile);

timeList_IMU = (rawData(:,1) - pivotTime_IMG)';
accelList = rawData(:, 2:4)';
accelList = bsxfun(@minus, accelList, gravity_bias);
accelList = gravity_scale * accelList * UNIT_G;
angrtList = rawData(:, 5:7)' * GYRO_SENSTV;
end