close all; clearvars; clc;
SetupEnv
SetupExperiment
load('exp16-room1turn');

%% Read data and initialize
len = size(all_states, 2);
cam = InitCam(CAM_RES);
load(IMU_CAL_FILE);

switch DATA_TYPE
    case 'ROS'
        [timeList_IMG, imgList, depList, pivotTime_IMG] = ...
            ReadVisualData_Kinect(visionLocation, TIME_DIFF);
    case 'ONI'
        [timeList_IMG, imgList, depList, pivotTime_IMG] = ...
            ReadVisualData_Oni(visionLocation, TIME_DIFF);
end
[timeList_IMU, accelList, angrtList] = ReadIMUData(imuFile, ...
    pivotTime_IMG, GYRO_SENSTV, UNIT_G, gravity_scale, gravity_bias);

prevFeat = ExtractFeat([visionLocation,imgList{1}],...
    [visionLocation,depList{1}], cam, IGN_MARGIN, STR_FEAT_NUM, DATA_TYPE, FEAT_TYPE);

%% First plot
im = imread(prevFeat.imFile);
im = rgb2gray(im);
fig1 = figure(1);
imshow(im);
% set(fig1, 'Position', [0, 450, 480, 360]);
truesize;

dep = LoadDepth(prevFeat.depFile, cam);
fig2 = figure(2);
imagesc(dep); colorbar;
% set(fig2, 'Position', [0, 0, 480, 360]);
truesize;

[p, ~, q] = State2Data(all_states(:,1));
[rotY, rotX, rotZ] = quat2angle(q', 'YXZ');
rotX = rad2deg(rotX); rotY = rad2deg(rotY); rotZ = rad2deg(rotZ);

fig3 = figure(3);
clf;
% set(fig3, 'Position', [590, 0, 480, 800]);
hold on;
% subplot(2,1,1);
plot3(p(1), p(3), p(2), '.b');
view(3);
set(gca,'ZDir','Reverse')
xlabel('X-axis (m)');ylabel('Z-axis (m)');zlabel('Y-axis (m)');
dcm_obj = datacursormode(fig3);
set(dcm_obj,'UpdateFcn', @myupdatefcn);
grid on; box on; daspect([1 1 1]);
title('Position');
drawnow;

%%
ixIMG = 1;
for ix = 2:len
    [p, ~, q] = State2Data(all_states(:,ix));
    [rotY, rotX, rotZ] = quat2angle(q', 'YXZ');
    rotX = rad2deg(rotX); rotY = rad2deg(rotY); rotZ = rad2deg(rotZ);
    
    if correct_mark(ix)
        ixIMG = ixIMG + 1;
        currFeat = ExtractFeat([visionLocation,imgList{ixIMG}],...
            [visionLocation,depList{ixIMG}], cam, IGN_MARGIN, STR_FEAT_NUM, ...
            DATA_TYPE, FEAT_TYPE);
        
        MatchFeat(prevFeat, currFeat, FEAT_TYPE, 1);
        
        dep = LoadDepth(currFeat.depFile, cam);
        figure(2); imagesc(dep); colorbar;
        
        figure(3);
        hold on;
        plot3(p(1), p(3), p(2), '.r');
        set(gca,'ZDir','Reverse')
        xlabel('X-axis (m)');ylabel('Z-axis (m)');zlabel('Y-axis (m)');
        dcm_obj = datacursormode(fig3);
        set(dcm_obj,'UpdateFcn', @myupdatefcn);
        grid on; box on; daspect([1 1 1]);
        title('Position');
        drawnow;
        
        prevFeat = currFeat;
    else
        figure(3);
        hold on;
        plot3(p(1), p(3), p(2), '.b');
        set(gca,'ZDir','Reverse')
        xlabel('X-axis (m)');ylabel('Z-axis (m)');zlabel('Y-axis (m)');
        dcm_obj = datacursormode(fig3);
        set(dcm_obj,'UpdateFcn', @myupdatefcn);
        grid on; box on; daspect([1 1 1]);
        title('Position');
        drawnow;
    end
end