%% The main file of the project
clearvars; close all; clc;
tic
SetupEnv
cam = InitCam();

%% Choose which experiment to run
OPT = 1;
DEBUG = 0;

switch OPT
    case 1
        dataLocation = '../data/Data1/';
        imuFile = strcat(dataLocation, 'sensor_data_still.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 2.3;
    case 2
        dataLocation = '../data/Data2/';
        imuFile = strcat(dataLocation, 'sensor_data_5min.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 3.8263;
    case 3
        dataLocation = '../data/Data3/';
        imuFile = strcat(dataLocation, 'sensor_data180.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 1.935;
    case 4
        dataLocation = '../data/Data4/';
        imuFile = strcat(dataLocation, 'sensor_data_table_rot.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 1.935;
end

if DEBUG
    imuFile = '../data/Data_Artificial/artData3.txt';
    NUM_G_EST = 1;
end

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
[timeList_IMG, imgList, depList, pivotTime_IMG] = ReadVisualData_Kinect(...
    visionLocation, TIME_DIFF);
[timeList_IMU, accelList, angrtList] = ReadIMUData(imuFile, ...
    pivotTime_IMG, GYRO_SENSTV, UNIT_G, gravity_scale, gravity_bias);

% TODO: this resets IMU's reading to zero
% accelList = zeros(1,size(accelList,2));a_b=[0 0 0]';
% angrtList = zeros(1,size(angrtList,2));w_b=[0 0 0]';
% accelList(3,:) = zeros(1,size(accelList,2));a_b(3)=0;

NUM_G_EST = find(timeList_IMU>0,1)-1;
x_I = InitIMUState(accelList(:,1:NUM_G_EST), a_b, w_b);
[Gx_p_handle, Gx_R_handle, Gf_handle] = InitInvObsJacob();

if DEBUG
    [p, v, q, a_b, w_b, g] = State2Data(x_I);
    p=[1;0;0];
    v=[-0.0616799542835500;3.92634488476604;0];
    a_b =zeros(3,1);
    w_b=zeros(3,1);
    x_I= Data2State(p, v, q, a_b, w_b, g);
end

prevFeat = ExtractFeat([visionLocation,imgList{1}],...
    [visionLocation,depList{1}], cam, IGN_MARGIN, STR_FEAT_NUM);
P = InitCovMat(LMK_COV, IMU_LEN, size(prevFeat.features,1));

% Remove data used for estimating initial gravity
timeList_IMU(1:NUM_G_EST) = [];
accelList(:,1:NUM_G_EST) = [];
angrtList(:,1:NUM_G_EST) = [];

prevTime = timeList_IMU(1); currTime = 0;
ixIMU = 1; ixIMG = 2;

%% ESKF process
nbIMU = length(timeList_IMU);
nbIMG = length(timeList_IMG);
if DEBUG
    nbIMG = 100;
end

all_states = x_I;
all_features = prevFeat;
correct_mark = 0;
while ixIMU < nbIMU && ixIMG < nbIMG
    %======================================================================
    % Error-State Kalman Filter
    %======================================================================
    if timeList_IMU(ixIMU+1) < timeList_IMG(ixIMG)
        ixIMU = ixIMU + 1;
        currTime = timeList_IMU(ixIMU);
        duration = currTime - prevTime;
        correct_mark = [correct_mark, 0]; %#ok<AGROW>
        
        % Prediction-------------------------------------------------------
        x_I = PredictIMUState(x_I, accelList(:,ixIMU), angrtList(:,ixIMU), duration);
        
        % F_x = GetJacobF(.); x_I_err = F_x * x_I_err;
        P = PredictCovMat(P, x_I, accelList(:,ixIMU), angrtList(:,ixIMU), ...
            v_b_var, ang_b_var, a_b_var, w_b_var, duration);
        
        % Update time (time duration only matters for IMU)
        prevTime = currTime;
    else
        imFile = [visionLocation, imgList{ixIMG}];
        depFile = [visionLocation, depList{ixIMG}];
%         imFile = [visionLocation, imgList{1}];
%         depFile = [visionLocation, depList{1}];
        correct_mark = [correct_mark, 1]; %#ok<AGROW>
        duration = timeList_IMG(ixIMG) - prevTime;
        
        % Prediction at the image moment-----------------------------------
        x_I = PredictIMUState(x_I, accelList(:,ixIMU+1), angrtList(:,ixIMU+1), duration);
        P = PredictCovMat(P, x_I, accelList(:,ixIMU+1), angrtList(:,ixIMU+1), ...
            v_b_var, ang_b_var, a_b_var, w_b_var, duration);
        prevTime = timeList_IMG(ixIMG);

        % Correction preparation-------------------------------------------
        % Extract features
        currFeat = ExtractFeat(imFile, depFile, cam, IGN_MARGIN, STR_FEAT_NUM);
        
        % Match 2D features
        [firstColOfPair, ~, currFeat, status] = MatchFeat(prevFeat, currFeat);
        numFeatToAdd = length(firstColOfPair);
        if status ~= 0 % error in matching
            warning('No match found');
            ixIMG = ixIMG + 1;
            continue;
        end
        
        % Match pointcloud and update the 3D world coordinates of feature
        % currFeat = MatchPtCloud(prevFeat, currFeat, cam, GRIDSIZE);
        % TrackWorldCoor(prevFeat,firstColOfPair,currFeat);
        
        % Covariance matrix blocks
        prevInlierNum = (size(P,1) - IMU_LEN)/FEAT_LEN;
        numBlocksToUpdate = firstColOfPair(firstColOfPair <= prevInlierNum);
        numBlocksToKeep = length(numBlocksToUpdate);
        blocksToDelete = setdiff(1:prevInlierNum, numBlocksToUpdate);
        blocksToAdd = setdiff(firstColOfPair,numBlocksToUpdate); % wrt the 1st col
        [~, blocksToAddIdx] = ismember(blocksToAdd, firstColOfPair);
        numBlocksToAdd = length(blocksToAdd);
        
        % Compute Jacobian H
        worldPosInlier = prevFeat.worldCoord(numBlocksToUpdate, :);
        measuredPosInlier = currFeat.worldCoord(1:numBlocksToKeep, :);
        H = GetJacobH(x_I, worldPosInlier, numBlocksToKeep, R_CI, T_IC);
        
        % Add - Drop some entries to P and update
        P = RemoveCovariance(P, blocksToDelete);
        
        % Expand IMU state -> full state
        x_all = ExpandState(x_I, worldPosInlier);
        
        % Correction-------------------------------------------------------
        V = diag(repmat(MSR_COV, 1, numBlocksToKeep));        
        Z = H*P*H' + V; % innovation covariance
        
        % Compute residual (innovation)
        z = ComputeResidual(x_I, worldPosInlier, measuredPosInlier, R_CI, T_IC);
        % z(abs(z)<10^-4)=0;
        [z, P, H, currFeat, numBlocksToUpdate] = MahalonobisTesting(z, P, H, Z ...
                        , currFeat, numBlocksToKeep, IMU_LEN, FEAT_LEN);
        numEntriesToUpdate = IMU_LEN+FEAT_LEN*numBlocksToUpdate;
        % if numBlocksToKeep < 20, warning('%d features left', numBlocksToUpdate);end
        PToUpdate = P(1:numEntriesToUpdate,1:numEntriesToUpdate);
        V = diag(repmat(MSR_COV, 1, length(z)/2));
        Z= H * PToUpdate * H' + V;
        
        % Compute Kalman gain K
        K = PToUpdate * H' / Z;
             
        % Correct errror state
        x_all_err = K * z;
        
        % Correct P (P <- (I-KH)P)
        tmp = eye(size(K,1))-K*H;
        PToUpdate = tmp*PToUpdate*tmp' + K*V*K';
        
        % Injection--------------------------------------------------------
        x_I = InjectErr(x_I, x_all_err);
        PToUpdate = ResetESKF(x_all_err, PToUpdate);
        P(1:numEntriesToUpdate,1:numEntriesToUpdate) = PToUpdate;
        
        % Update-----------------------------------------------------------
        % Update features
        % currFeat.worldCoord(1:numBlocksToKeep,:) = prevFeat.worldCoord(firstColOfPair(1:numBlocksToKeep),:);
        currFeat = UpdateFeat(x_I, x_all_err, currFeat, numBlocksToUpdate, R_CI, T_IC);
        % currFeat = InjectWorldCoord(x_I, x_all_err, currFeat, R_CI, T_IC, blocksToKeep, numBlocksToKeep, cam)
        P = AddNewCovariancePrior(P, numBlocksToAdd, FEAT_LEN, LMK_COV);
%         P = AddNewCovariance(P, numBlocksToAdd, currFeat.imgCoord(blocksToAddIdx,:), ...
%             currFeat.worldCoord(blocksToAddIdx,:), IMU_LEN, LMK_COV, ...
%             Gx_p_handle, Gx_R_handle, Gf_handle, x_I);
        
        % Update state
        figure(1); imagesc(P); colorbar; 
        str = sprintf('%d landmarks (keep %d, add %d)', ...
            (size(P,1)-IMU_LEN)/FEAT_LEN, numBlocksToKeep, numBlocksToAdd);
        title(str);
        prevFeat = currFeat;
        all_features = [all_features, prevFeat]; %#ok<AGROW>
        ixIMG = ixIMG + 1;
    end
    
    fprintf('Index:%d - IMU:%d - vision:%d...\n', length(correct_mark), ...
        ixIMU, ixIMG-1);
    
    %======================================================================
    % Save the position
    all_states = [all_states, x_I]; %#ok<AGROW>
    
    % Check jumps
    prev_state = all_states(:,end);
    [~, prev_v, prev_q] = State2Data(prev_state);
    [p, v, q] = State2Data(x_I);
    [prev_Y, prev_X, prev_Z] = quat2angle(prev_q', 'YXZ');
    [Y, X, Z] = quat2angle(q', 'YXZ');
    prev_X=rad2deg(prev_X);prev_Y=rad2deg(prev_Y);prev_Z=rad2deg(prev_Z);
    X=rad2deg(X);Y=rad2deg(Y);Z=rad2deg(Z);
    if find(abs(v-prev_v) > 5)
        warning('Velocity jump\n');
    end
    if find(abs([X,Y,Z]-[prev_X,prev_Y,prev_Z]) > 5)
        warning('Rotation jump\n');
    end
    
    % if abs(p(1))>10 || abs(p(2))>10 || abs(p(3))>10, break, end
    if isnan(p), break; end
    if timeList_IMG(ixIMG) > 40, break; end %stop after 40 seconds
%     if length(correct_mark) == 850, break; end
end

toc
%% visualize result
fprintf('Plotting results...\n');
p = all_states(1:3,:);
v = all_states(4:6,:);
q = all_states(7:10,:);
a_b = all_states(11:13,:);
w_b = all_states(14:16,:);
g = all_states(17:19,:);

corr_idx_idx = find(correct_mark);

% plot position------------------------------------------------------------
figure;hold on;
plot3(p(1,:), p(2,:), p(3,:), '-');
plot3(p(1,correct_mark>0), p(2,correct_mark>0), p(3,correct_mark>0), 'or');
view(3);
xlabel('X-axis (m)');ylabel('Y-axis (m)');zlabel('Z-axis (m)');
title('Position');
grid on; box on;rotate3d on;
daspect([1 1 1]);
hold off;

% plot velocity------------------------------------------------------------
figure;hold on;
plot(v(1,:),'r');
plot(v(2,:),'g');
plot(v(3,:),'b');
plot(corr_idx_idx,v(1,corr_idx_idx),'or');
plot(corr_idx_idx,v(2,corr_idx_idx),'og');
plot(corr_idx_idx,v(3,corr_idx_idx),'ob');
title('Velocity');
xlabel('Samples');ylabel('m/s');
grid on;
legend('v_x', 'v_y', 'v_z');
hold off;

% plot angle---------------------------------------------------------------
[yaw, pitch, roll] = quat2angle(q', 'YXZ');
yaw = rad2deg(unwrap(yaw));
pitch = rad2deg(unwrap(pitch));
roll = rad2deg(unwrap(roll));

figure;hold on;
plot(yaw,'r');
plot(pitch,'g');
plot(roll,'b');
plot(corr_idx_idx,yaw(corr_idx_idx),'or');
plot(corr_idx_idx,pitch(corr_idx_idx),'og');
plot(corr_idx_idx,roll(corr_idx_idx),'ob');
title('Rotation');
xlabel('Samples');ylabel('Degree');
grid on;
legend('Yaw (Y-axis)', 'Pitch (X-axis)', 'Roll (Z-axis)');
hold off;

% plot accelerometer bias--------------------------------------------------
figure;hold on;
plot(a_b(1,:),'r');
plot(a_b(2,:),'g');
plot(a_b(3,:),'b');
plot(corr_idx_idx,a_b(1,corr_idx_idx),'or');
plot(corr_idx_idx,a_b(2,corr_idx_idx),'og');
plot(corr_idx_idx,a_b(3,corr_idx_idx),'ob');
title('Accelerometer bias');
xlabel('Samples');ylabel('m/s^{2}');
grid on;
legend('a_{b_x}', 'a_{b_y}', 'a_{b_z}');
hold off;

% plot gyro bias-----------------------------------------------------------
figure;hold on;
plot(w_b(1,:),'r');
plot(w_b(2,:),'g');
plot(w_b(3,:),'b');
plot(corr_idx_idx,w_b(1,corr_idx_idx),'or');
plot(corr_idx_idx,w_b(2,corr_idx_idx),'og');
plot(corr_idx_idx,w_b(3,corr_idx_idx),'ob');
title('Gyro bias');
xlabel('Samples');ylabel('rad/s');
grid on;
legend('w_{b_x}', 'w_{b_y}', 'w_{b_z}');
hold off;

% plot gravity-------------------------------------------------------------
figure;hold on;
plot(g(1,:),'r');
plot(g(2,:),'g');
plot(g(3,:),'b');
plot(corr_idx_idx,g(1,corr_idx_idx),'or');
plot(corr_idx_idx,g(2,corr_idx_idx),'og');
plot(corr_idx_idx,g(3,corr_idx_idx),'ob');
title('Gravity');
xlabel('Samples');ylabel('m/s^{2}');
grid on;
legend('g_x', 'g_y', 'g_z');
hold off;

% plot point cloud---------------------------------------------------------
% mergeSize = 0.015;
% bigPtCloud = GetPointCloud(all_features(1).imFile, all_features(1).depFile, cam);
% for ix=2:length(all_features)
%     ptCloud = GetPointCloud(all_features(ix).imFile, all_features(ix).depFile, cam);
%     ptCloud = pctransform(ptCloud, all_features(ix).accTform3d);
%     bigPtCloud = pcmerge(bigPtCloud, ptCloud, mergeSize);
% end
% figure;showPointCloud(bigPtCloud);
% xlabel('X-axis (m)');ylabel('Y-axis (m)');zlabel('Z-axis (m)');
% title('Point cloud');
