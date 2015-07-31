%% The main file of the project
clearvars; close all; clc;

SetupEnv
cam = InitCam();

dataLocation = '~/Desktop/';
imuFile = strcat(dataLocation, 'sensor_data.txt');

%% Calibrate IMU sensor
if ~isempty(IMU_CAL_FILE) && exist(IMU_CAL_FILE, 'file')
    load(IMU_CAL_FILE);
else
    [a_b, a_b_var, v_b_var, w_b, w_b_var, ang_b_var, gravity_scale, gravity_bias] = CalibrateIMU(UNIT_G, GYRO_SENSTV);
    save(IMU_CAL_FILE, 'a_b', 'a_b_var', 'v_b_var', 'w_b', 'w_b_var', 'ang_b_var', 'gravity_scale', 'gravity_bias');
end

%% Read data and initialize the state
[timeList_IMU, accelList, angrtList] = ReadIMUData(...
    imuFile, 0, GYRO_SENSTV, UNIT_G, gravity_scale, gravity_bias);
timeList_IMU(1:15) = [];
accelList(:,1:15) = [];
angrtList(:,1:15) = [];


NUM_G_EST = 100;
x_I = InitIMUState(accelList(:,1:NUM_G_EST), a_b, w_b);

% Remove data used for estimating initial gravity
timeList_IMU(1:NUM_G_EST) = [];
accelList(:,1:NUM_G_EST) = [];
angrtList(:,1:NUM_G_EST) = [];

samples = length(timeList_IMU);

prevTime = timeList_IMU(1); currTime = 0;
states = [];

%%
for ixIMU = 2 : samples
    currTime = timeList_IMU(ixIMU);
    duration = currTime - prevTime;
    
    % Prediction-------------------------------------------------------
    x_I = PredictIMUState(x_I, accelList(:,ixIMU), angrtList(:,ixIMU), duration);
    
    % Update time (time duration only matters for IMU)
    prevTime = currTime;    
    states = [states, x_I];
end

%%
figure();
rotate3d on
plot3(states(1,:), states(2,:), states(3,:), '--p');
xlabel('X-axis');ylabel('Y-axis');zlabel('Z-axis');
title('Position');
grid on; box on;
daspect([1 1 1]);

%%
figure();
q = states(7:10,:)';
% [x,y,z] = quat2angle(q, 'XYZ');
[y,x,z] = quat2angle(q, 'YXZ');
hold on
plot(rad2deg(x), 'r');
plot(rad2deg(y), 'g');
plot(rad2deg(z), 'b');
hold off
title('Angle (deg) (using prediction)');
xlabel('Samples');ylabel('Angle (degree)'); grid on;

%%
delta_t = timeList_IMU(2:end) - timeList_IMU(1:end-1);
angrtList=bsxfun(@minus,angrtList,w_b);
angles = bsxfun(@times, angrtList(:,1:end-1), delta_t);
angles = rad2deg(angles);
angles_sum = cumsum(angles,2);

figure();
hold on
plot(angles_sum(1,:), 'r');
plot(angles_sum(2,:), 'g');
plot(angles_sum(3,:), 'b');
hold off
title('Cummulative angle (deg) (from raw)');
xlabel('Samples');ylabel('Angle (degree)'); grid on;