%% Calibrate IMU on six different poses. 
% The input data for calibration is fixed

function [acce_bias, acce_bias_var, velo_bias_var, gyro_bias, gyro_bias_var, ang_bias_var] = CalibrateIMU()
location = '../data/Data_Calibrate1/';
dataFile1 = strcat(location,'calibrate1.txt');
dataFile2 = strcat(location,'calibrate2.txt');
dataFile3 = strcat(location,'calibrate3.txt');
dataFile4 = strcat(location,'calibrate4.txt');
dataFile5 = strcat(location,'calibrate5.txt');
dataFile6 = strcat(location,'calibrate6.txt');

content = load(dataFile1);
accelList1 = content(:,2:4)';
angrtList1 = content(:,5:7)';
time1 = GetTime(dataFile1);

content = load(dataFile2);
accelList2 = content(:,2:4)';
angrtList2 = content(:,5:7)';
time2 = GetTime(dataFile2);

content = load(dataFile3);
accelList3 = content(:,2:4)';
angrtList3 = content(:,5:7)';
time3 = GetTime(dataFile3);

content = load(dataFile4);
accelList4 = content(:,2:4)';
angrtList4 = content(:,5:7)';
time4 = GetTime(dataFile4);

content = load(dataFile5);
accelList5 = content(:,2:4)';
angrtList5 = content(:,5:7)';
time5 = GetTime(dataFile5);

content = load(dataFile6);
accelList6 = content(:,2:4)';
angrtList6 = content(:,5:7)';
time6 = GetTime(dataFile6);

%% calibrate gyroscope
gyro_all = [angrtList1, angrtList2, angrtList3, angrtList4, angrtList5, angrtList6];
gyro_bias = mean(gyro_all, 2);
gyro_bias_var = var(gyro_all, 0, 2);

%% calibrate accelerometer
acce_mean_1 = mean(accelList1, 2);
acce_mean_2 = mean(accelList2, 2);
acce_mean_3 = mean(accelList3, 2);
acce_mean_4 = mean(accelList4, 2);
acce_mean_5 = mean(accelList5, 2);
acce_mean_6 = mean(accelList6, 2);

acce_all_x = [accelList1(1,:), accelList3(1,:), accelList5(1,:), accelList6(1,:)];
acce_all_y = [accelList1(2,:), accelList2(2,:), accelList4(2,:), accelList6(2,:)];
acce_all_z = [accelList2(3,:), accelList3(3,:), accelList4(3,:), accelList5(3,:)];

acce_mean_x = [acce_mean_1(1,:), acce_mean_3(1,:), acce_mean_5(1,:), acce_mean_6(1,:)];
acce_mean_y = [acce_mean_1(2,:), acce_mean_2(2,:), acce_mean_4(2,:), acce_mean_6(2,:)];
acce_mean_z = [acce_mean_2(3,:), acce_mean_3(3,:), acce_mean_4(3,:), acce_mean_5(3,:)];

accel_x_pos = sum(acce_mean_x>=0);
accel_x_neg = sum(acce_mean_x<0);
accel_y_pos = sum(acce_mean_y>=0);
accel_y_neg = sum(acce_mean_y<0);
accel_z_pos = sum(acce_mean_z>=0);
accel_z_neg = sum(acce_mean_z<0);

acce_bias =zeros(3,1);

if accel_x_pos > accel_x_neg
    acce_bias(1) = mean(acce_mean_x(acce_mean_x>=0));
else
    acce_bias(1) = mean(acce_mean_x(acce_mean_x<0));
end

if accel_y_pos > accel_y_neg
    acce_bias(2) = mean(acce_mean_y(acce_mean_y>=0));
else
    acce_bias(2) = mean(acce_mean_y(acce_mean_y<0));
end

if accel_z_pos > accel_z_neg
    acce_bias(3) = mean(acce_mean_z(acce_mean_z>=0));
else
    acce_bias(3) = mean(acce_mean_z(acce_mean_z<0));
end

acce_bias_var = [var(accelList1,[],2), var(accelList2,[],2), var(accelList3,[],2), ...
    var(accelList4,[],2), var(accelList5,[],2), var(accelList6,[],2)];
acce_bias_var=sum(acce_bias_var,2);


% acce_bias = [mean(acce_all_x); mean(acce_all_y); mean(acce_all_z)];
%acce_bias_std = [std(acce_all_x); std(acce_all_y); std(acce_all_z)];


% acce_std_1 = std(accelList1, 0, 2);
% acce_std_2 = std(accelList2, 0, 2);
% acce_std_3 = std(accelList3, 0, 2);
% acce_std_4 = std(accelList4, 0, 2);
% acce_std_5 = std(accelList5, 0, 2);
% acce_std_6 = std(accelList6, 0, 2);



g1 = acce_mean_1(3) - acce_bias(3);
g2 = acce_mean_2(1) - acce_bias(1);
g3 = acce_mean_3(2) - acce_bias(2);
g4 = acce_mean_4(1) - acce_bias(1);
g5 = acce_mean_5(2) - acce_bias(2);
g6 = acce_mean_6(3) - acce_bias(3);

g = mean([abs(g1),abs(g2),abs(g3),abs(g4),abs(g5),abs(g6)]);

% acce_bias_x = mean(accelList1(1,:));
% acce_bias_y = mean(accelList1(2,:));
% acce_bias_z = mean(accelList2(3,:));
% acce_bias = [acce_bias_x; acce_bias_y; acce_bias_z];
% 
% acce_bias_std_x = std(accelList1(1,:));
% acce_bias_std_y = std(accelList1(2,:));
% acce_bias_std_z = std(accelList2(3,:));
% acce_bias_std = [acce_bias_std_x; acce_bias_std_y; acce_bias_std_z];

%% calibrate velocity
v1 = GetVelocity(accelList1, time1);
v2 = GetVelocity(accelList2, time2);
v3 = GetVelocity(accelList3, time3);
v4 = GetVelocity(accelList4, time4);
v5 = GetVelocity(accelList5, time5);
v6 = GetVelocity(accelList6, time6);

%v_b_std = std([v1,v2,v3,v4,v5,v6],[],2);
velo_bias_var = [var(v1,[],2), var(v2,[],2), var(v3,[],2), ...
    var(v4,[],2), var(v5,[],2), var(v6,[],2)];
velo_bias_var=sum(velo_bias_var,2);

%% calibrate angle
w1 = GetAngle(angrtList1, time1);
w2 = GetAngle(angrtList2, time2);
w3 = GetAngle(angrtList3, time3);
w4 = GetAngle(angrtList4, time4);
w5 = GetAngle(angrtList5, time5);
w6 = GetAngle(angrtList6, time6);

ang_bias_var = var([w1,w2,w3,w4,w5,w6], 0, 2);

end

%%
function time = GetTime(dataFile)
fid = fopen(dataFile,'r');
timeraw = textscan(fid, '%s %*[^\n]');
fclose(fid);

time = zeros(length(timeraw{1}), 1);

for ix=1:length(timeraw{1})
    time(ix) = datenum(timeraw{1}{ix});
end

end

%%
function v = GetVelocity(accelList, timeList)

for ix=1:length(timeList)-1
    t = timeList(ix+1) - timeList(ix);
    v(:,ix) = accelList(:,ix)*t;
end

end

%%
function w = GetAngle(angrtList, timeList)

for ix=1:length(timeList)-1
    t = timeList(ix+1) - timeList(ix);
    w(:,ix) = angrtList(:,ix)*t;
end

end