%% Calibrate IMU on six different poses. 
% The input data for calibration is fixed

function [acce_bias, acce_bias_var, velo_bias_var, gyro_bias, gyro_bias_var,...
    ang_bias_var, gravity_scale, gravity_bias] = CalibrateIMU(UNIT_G, GYRO_SENSTV)
location = '../data/Data_Calibrate2/';
dataFile1 = strcat(location,'calibrate1.txt');
dataFile2 = strcat(location,'calibrate2.txt');
dataFile3 = strcat(location,'calibrate3.txt');
dataFile4 = strcat(location,'calibrate4.txt');
dataFile5 = strcat(location,'calibrate5.txt');
dataFile6 = strcat(location,'calibrate6.txt');
dataFile_gyro = strcat(location,'calibrate_gyro.txt');

content = load(dataFile1);
time1 = content(:,1);
accelList1 = content(:,2:4)';
% angrtList1 = content(:,5:7)';
samples1 = length(time1);

content = load(dataFile2);
time2 = content(:,1);
accelList2 = content(:,2:4)';
% angrtList2 = content(:,5:7)';
samples2 = length(time2);

content = load(dataFile3);
time3 = content(:,1);
accelList3 = content(:,2:4)';
% angrtList3 = content(:,5:7)';
samples3 = length(time3);

content = load(dataFile4);
time4 = content(:,1);
accelList4 = content(:,2:4)';
% angrtList4 = content(:,5:7)';
samples4 = length(time4);

content = load(dataFile5);
time5 = content(:,1);
accelList5 = content(:,2:4)';
% angrtList5 = content(:,5:7)';
samples5 = length(time5);

content = load(dataFile6);
time6 = content(:,1);
accelList6 = content(:,2:4)';
% angrtList6 = content(:,5:7)';
samples6 = length(time6);

content = load(dataFile_gyro);
time_gyro = content(:,1);
angrtList_gyro = content(:,5:7)';

%% calibrate gyroscope
% gyro_all = [angrtList1, angrtList2, angrtList3, angrtList4, angrtList5, angrtList6]*GYRO_SENSTV;
gyro_all = angrtList_gyro*GYRO_SENSTV;
gyro_bias = mean(gyro_all, 2);
gyro_bias_var = var(gyro_all, 0, 2);

%% calibrate angle
% w1 = GetAngle(angrtList1, time1);
% w2 = GetAngle(angrtList2, time2);
% w3 = GetAngle(angrtList3, time3);
% w4 = GetAngle(angrtList4, time4);
% w5 = GetAngle(angrtList5, time5);
% w6 = GetAngle(angrtList6, time6);
% ang_bias_var = var([w1,w2,w3,w4,w5,w6], 0, 2);

w = GetAngle(angrtList_gyro, time_gyro);
ang_bias_var = var(w, 0, 2);

%% calibrate accelerometer
Y1 = repmat([0 -1 0], samples1, 1);
Y2 = repmat([0 0 1], samples2, 1);
Y3 = repmat([-1 0 0], samples3, 1);
Y4 = repmat([0 0 -1], samples4, 1);
Y5 = repmat([1 0 0], samples5, 1);
Y6 = repmat([0 1 0], samples6, 1);
Y = [Y1; Y2; Y3; Y4; Y5; Y6];

X1 = [accelList1', ones(samples1,1)];
X2 = [accelList2', ones(samples2,1)];
X3 = [accelList3', ones(samples3,1)];
X4 = [accelList4', ones(samples4,1)];
X5 = [accelList5', ones(samples5,1)];
X6 = [accelList6', ones(samples6,1)];
X = [X1; X2; X3; X4; X5; X6];

w = X\Y;

gravity_bias = w(4,:);
gravity_scale = w(1:3,:);

acce_unbiased = bsxfun(@minus, X(:,1:3), gravity_bias);
acce_expected = gravity_scale * acce_unbiased';

startIndex =1;
endIndex = samples1;
accelList1 = acce_expected(:,startIndex:endIndex);

startIndex = startIndex + samples1;
endIndex = startIndex + samples2 - 1;
accelList2 = acce_expected(:,startIndex:endIndex);

startIndex = startIndex + samples2;
endIndex = startIndex + samples3 - 1;
accelList3 = acce_expected(:,startIndex:endIndex);

startIndex = startIndex + samples3;
endIndex = startIndex + samples4 - 1;
accelList4 = acce_expected(:,startIndex:endIndex);

startIndex = startIndex + samples4;
endIndex = startIndex + samples5 - 1;
accelList5 = acce_expected(:,startIndex:endIndex);

startIndex = startIndex + samples5;
endIndex = startIndex + samples6 - 1;
accelList6 = acce_expected(:,startIndex:endIndex);

acce_bias = mean(accelList1,2) - [0;-1;0];
% mean2 = mean(accelList2,2);
% mean3 = mean(accelList3,2);
% mean4 = mean(accelList4,2);
% mean5 = mean(accelList5,2);
% mean6 = mean(accelList6,2);


acce_noise = UNIT_G * bsxfun(@minus, (X(:,1:3) - acce_expected'), acce_bias');
acce_bias_var = var(acce_noise, [], 1)';
gravity_bias = gravity_bias';

%% calibrate velocity
startIndex =1;
endIndex = samples1;
v1 = GetVelocity(acce_noise(startIndex:endIndex,:), time1);

startIndex = startIndex + samples1;
endIndex = startIndex + samples2 - 1;
v2 = GetVelocity(acce_noise(startIndex:endIndex,:), time2);

startIndex = startIndex + samples2;
endIndex = startIndex + samples3 - 1;
v3 = GetVelocity(acce_noise(startIndex:endIndex,:), time3);

startIndex = startIndex + samples3;
endIndex = startIndex + samples4 - 1;
v4 = GetVelocity(acce_noise(startIndex:endIndex,:), time4);

startIndex = startIndex + samples4;
endIndex = startIndex + samples5 - 1;
v5 = GetVelocity(acce_noise(startIndex:endIndex,:), time5);

startIndex = startIndex + samples5;
endIndex = startIndex + samples6 - 1;
v6 = GetVelocity(acce_noise(startIndex:endIndex,:), time6);

%v_b_std = std([v1,v2,v3,v4,v5,v6],[],2);
velo_bias_var = [var(v1,[],2), var(v2,[],2), var(v3,[],2), ...
    var(v4,[],2), var(v5,[],2), var(v6,[],2)];
velo_bias_var=sum(velo_bias_var,2);


end

%%
function w = GetAngle(angrtList, timeList)

for ix=1:length(timeList)-1
    t = timeList(ix+1) - timeList(ix);
    w(:,ix) = angrtList(:,ix)*t;
end

end

%%
function v = GetVelocity(accelList, timeList)
accelList = accelList';
for ix=1:length(timeList)-1
    t = timeList(ix+1) - timeList(ix);
    v(:,ix) = accelList(:,ix)*t;
end

end