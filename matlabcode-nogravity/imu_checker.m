clearvars; close all; clc;

SetupEnv
SetupExperiment

dataLocation = '~/Desktop/';
imuFile = strcat(dataLocation, 'sensor_data360yaw.txt');

%% Calibrate IMU sensor
if ~isempty(IMU_CAL_FILE) && exist(IMU_CAL_FILE, 'file')
    load(IMU_CAL_FILE);
else
    [a_b, a_b_var, v_b_var, w_b, w_b_var, ang_b_var, ...
        gravity_scale, gravity_bias] = CalibrateIMU(UNIT_G, GYRO_SENSTV);
    save(IMU_CAL_FILE, 'a_b', 'a_b_var', 'v_b_var', 'w_b', 'w_b_var', ...
        'ang_b_var', 'gravity_scale', 'gravity_bias');
end

%% Read data and initialize the state
cam = InitCam(CAM_RES);
pivotTime_IMG = 0;

[timeList_IMU, accelList, angrtList, diffQuatList] = ReadIMUData(imuFile, ...
    pivotTime_IMG, GYRO_SENSTV, UNIT_G, gravity_scale, gravity_bias);

% NUM_G_EST = find(timeList_IMU>0, 1)-1;
NUM_G_EST = 50;
x_I = InitIMUState(accelList(:,1:NUM_G_EST), a_b, w_b);

% Remove data used for estimating initial gravity
timeList_IMU(1:NUM_G_EST) = [];
accelList(:,1:NUM_G_EST) = [];
angrtList(:,1:NUM_G_EST) = [];
diffQuatList(:,1:NUM_G_EST) = [];

samples = length(timeList_IMU);

prevTime = timeList_IMU(1); currTime = 0;
all_states = [];

%%
for ixIMU = 2 : samples
    currTime = timeList_IMU(ixIMU);
    duration = currTime - prevTime;
    
    % Prediction-------------------------------------------------------
    x_I = PredictIMUState(x_I, accelList(:,ixIMU), angrtList(:,ixIMU), duration);
    
    % Update time (time duration only matters for IMU)
    prevTime = currTime;    
    all_states = [all_states, x_I]; %#ok<AGROW>
end

%%
correct_mark = zeros(size(timeList_IMU));
Visualizer;

quatGlobalList = [1 0 0 0]';
pitch = 0;
yaw = 0;
roll = 0;
for ix = 1:length(diffQuatList)
    q = quatGlobalList(:, end);
    r = diffQuatList(:, ix);
    t = r(2); r(2) = r(4); r(4) = t;
    
    qNew = quatmultiply(q', r')';
    [oriY, oriX, oriZ] = quat2angle(qNew', 'YXZ');
    
    quatGlobalList = [quatGlobalList, qNew]; %#ok<AGROW>
    pitch = [pitch, rad2deg(oriX)]; %#ok<AGROW>
    yaw = [yaw, rad2deg(oriY)]; %#ok<AGROW>
    roll = [roll, rad2deg(oriZ)]; %#ok<AGROW>
end

figure;hold on;
plot(pitch,'r');
plot(yaw,'g');
plot(roll,'b');
plot(corr_idx_idx,pitch(corr_idx_idx),'or');
plot(corr_idx_idx,yaw(corr_idx_idx),'og');
plot(corr_idx_idx,roll(corr_idx_idx),'ob');
title('Sensor''s Filtered Rotation');
xlabel('Samples');ylabel('Degree');
grid on;
legend('Pitch (X-axis)', 'Yaw (Y-axis)', 'Roll (Z-axis)');
hold off;