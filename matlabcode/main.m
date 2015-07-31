%% The main file of the project
clearvars; close all; clc;
tic
SetupEnv
SetupExperiment

%% Calibrate IMU sensor
if ~isempty(IMU_CAL_FILE) && exist(IMU_CAL_FILE, 'file')
    load(IMU_CAL_FILE);
else
    [a_b, a_b_var, v_b_var, w_b, w_b_var, ang_b_var, ...
        gravity_scale, gravity_bias] = CalibrateIMU(UNIT_G, GYRO_SENSTV);
    save(IMU_CAL_FILE, 'a_b', 'a_b_var', 'v_b_var', 'w_b', 'w_b_var', ...
        'ang_b_var', 'gravity_scale', 'gravity_bias');
end

%% Read data and initialize
cam = InitCam(CAM_RES);

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

% downsampling visual data (if necessary)
if strcmp(DATA_TYPE, 'ONI')
%     timeList_IMG = timeList_IMG(1:5:end);
%     imgList = imgList(1:5:end);
%     depList = depList(1:5:end);
end

% TODO: this resets IMU's reading to zero
% accelList = zeros(1,size(accelList,2));a_b=[0 0 0]';
% angrtList = zeros(1,size(angrtList,2));w_b=[0 0 0]';
% accelList(3,:) = zeros(1,size(accelList,2));a_b(3)=0;

NUM_G_EST = find(timeList_IMU>0, 1)-1;
x_I = InitIMUState(accelList(:,1:NUM_G_EST), a_b, w_b);
[~, ~, ~, ~, ~, initG] = State2Data(x_I);
[Gx_p_handle, Gx_R_handle, Gf_handle] = InitInvObsJacob(CAM_RES);

prevFeat = ExtractFeat([visionLocation,imgList{1}],...
    [visionLocation,depList{1}], cam, IGN_MARGIN, STR_FEAT_NUM, DATA_TYPE, FEAT_TYPE);
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

all_states = x_I;
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
            v_b_var, ang_b_var, a_b_var, w_b_var, duration, IMU_LEN);
        
        % Update time (time duration only matters for IMU)
        prevTime = currTime;
    else
        imFile = [visionLocation, imgList{ixIMG}];
        depFile = [visionLocation, depList{ixIMG}];
        % imFile = [visionLocation, imgList{1}];
        % depFile = [visionLocation, depList{1}];
        correct_mark = [correct_mark, 1]; %#ok<AGROW>
        duration = timeList_IMG(ixIMG) - prevTime;
        
        % Prediction at the image moment-----------------------------------
        x_I = PredictIMUState(x_I, accelList(:,ixIMU+1), angrtList(:,ixIMU+1), duration);
        P = PredictCovMat(P, x_I, accelList(:,ixIMU+1), angrtList(:,ixIMU+1), ...
            v_b_var, ang_b_var, a_b_var, w_b_var, duration, IMU_LEN);
        prevTime = timeList_IMG(ixIMG);
        
        x_I_beforeCorrect = x_I;
        P_beforeCorrect = P;       
        
        % Correction preparation-------------------------------------------
        % Extract features
        currFeat = ExtractFeat(imFile, depFile, cam, IGN_MARGIN, STR_FEAT_NUM, DATA_TYPE, FEAT_TYPE);
        
        % Match 2D features
        [firstColOfPair, ~, currFeat, status] = MatchFeat(prevFeat, currFeat, FEAT_TYPE, DEBUG_IMG);
        numFeatToAdd = length(firstColOfPair);
        if status ~= 0 % error in matching
            warning('No match found. Skip correction phase.');
            
            prevFeat = UpdateFeat(x_I, [], currFeat, 0, R_CI, T_IC, IMU_LEN);
%             prevFeat = UpdateFeat(x_I,prevFeat,currFeat, [], 0, R_CI, T_IC);
            ixIMG = ixIMG + 1;
            continue;
        end
        
        % Match pointcloud and update the 3D world coordinates of feature
        % currFeat = MatchPtCloud(prevFeat, currFeat, cam, GRIDSIZE);
        % TrackWorldCoor(prevFeat,firstColOfPair,currFeat);
        
        % Covariance matrix blocks
        prevInlierNum = (size(P,1) - IMU_LEN)/FEAT_LEN;
        blocksToKeep = firstColOfPair(firstColOfPair <= prevInlierNum);
        numBlocksToKeep = length(blocksToKeep);
        blocksToDelete = setdiff(1:prevInlierNum, blocksToKeep);
        blocksToAdd = setdiff(firstColOfPair,blocksToKeep); % wrt the 1st col
        [~, blocksToAddIdx] = ismember(blocksToAdd, firstColOfPair);
        numBlocksToAdd = length(blocksToAdd);
        
        % Compute Jacobian H
        worldPosInlier = prevFeat.worldCoord(blocksToKeep, :);
        measuredPosInlier = currFeat.worldCoord(1:numBlocksToKeep, :);
%         H = GetJacobH(x_I, worldPosInlier, numBlocksToKeep, R_CI, T_IC);
        H = GetJacobH(x_I, measuredPosInlier, numBlocksToKeep, R_CI, T_IC, IMU_LEN, FEAT_LEN);
        
        % Add - Drop some entries to P and update
        P = RemoveCovariance(P, blocksToDelete, IMU_LEN);
        
        % Expand IMU state -> full state
        x_all = ExpandState(x_I, worldPosInlier);
        
        % Correction-------------------------------------------------------
        V = diag(repmat(MSR_COV, 1, numBlocksToKeep));
        Z = H*P*H' + V; % inZnovation covariance
        
        % Compute residual (innovation)
        z = ComputeResidual(x_I, worldPosInlier, measuredPosInlier, R_CI, T_IC);
        % z(abs(z)<10^-4)=0;
        
        tmp = length(z)/2;
        [z, P, H, Z, V, currFeat, blocksToKeep] = MahalonobisTesting(z, P, H, Z, MSR_COV, currFeat, blocksToKeep, IMU_LEN, FEAT_LEN);
        numBlocksToKeep = length(blocksToKeep);
        if numBlocksToKeep < FEAT_NUM_THRES && numBlocksToKeep ~= tmp,
            warning('%d/%d features left', numBlocksToKeep, tmp);
        end
        
        % Compute Kalman gain K
        K = P*H' / Z;
         
        % Correct errror state
        x_all_err = K * z;
                
        % Correct P (P <- (I-KH)P)
        tmp = eye(size(K,1))-K*H;
        P = tmp*P*tmp' + K*V*K';
        
        % Injection--------------------------------------------------------        
        x_I = InjectErr(x_I, x_all_err, IMU_LEN);
        P = ResetESKF(x_all_err, P, IMU_LEN);
        
        % Update-----------------------------------------------------------
        % Update features
        feat_beforeUpdate = currFeat;        
        currFeat = UpdateFeat(x_I, x_all_err, currFeat, numBlocksToKeep, R_CI, T_IC, IMU_LEN);
%         currFeat = UpdateFeat(x_I,prevFeat,currFeat, blocksToKeep, numBlocksToKeep, R_CI, T_IC);
        P = AddNewCovariancePrior(P, numBlocksToAdd, FEAT_LEN, LMK_COV);
%         [P,dontAdd] = AddNewCovariance(P, numBlocksToAdd, currFeat.imgCoord(blocksToAddIdx,:), ...
%             currFeat.worldCoord(blocksToAddIdx,:), IMU_LEN, LMK_COV, ...
%             Gx_p_handle, Gx_R_handle, Gf_handle, x_I);
%         if ~isempty(dontAdd) 
%             currFeat = dontAddFeat(currFeat,dontAdd,numBlocksToKeep);
%         end

        % Update state
        prevFeat = currFeat;
        ixIMG = ixIMG + 1;
    end
    
    fprintf('Index:%d - IMU:%d - vision:%d...\n', length(correct_mark), ...
        ixIMU, ixIMG-1);
    
    %======================================================================
    % Save the position
    if size(all_states, 2) > HIST_WIN
        prev_state = all_states(:,end-HIST_WIN:end);
    else
        prev_state = all_states;
    end
    prev_state = median(prev_state, 2);
    all_states = [all_states, x_I]; %#ok<AGROW>
    
    % Check jumps    
    [prev_p, prev_v, prev_q] = State2Data(prev_state);
    [p, v, q] = State2Data(x_I);
    [prev_Y, prev_X, prev_Z] = quat2angle(prev_q', 'YXZ');
    [curr_Y, curr_X, curr_Z] = quat2angle(q', 'YXZ');
    prev_X=rad2deg(prev_X);prev_Y=rad2deg(prev_Y);prev_Z=rad2deg(prev_Z);
    curr_X=rad2deg(curr_X);curr_Y=rad2deg(curr_Y);curr_Z=rad2deg(curr_Z);
    
    jump = 0;
    if abs(norm(p-prev_p)) > MAX_ALLOW_P
        warning('Position jump');
        jump = 1;
    end
    if abs(norm(v-prev_v)) > MAX_ALLOW_V
        warning('Velocity jump');
        jump = 1;
    end
%     if (abs(curr_X-prev_X) > MAX_ALLOW_W || ...
%             abs(curr_Y-prev_Y) > MAX_ALLOW_W || abs(curr_Z-prev_Z) > MAX_ALLOW_W)
%         warning('Rotation jump');
%         jump = 1;
%     end    
%     if jump
%         x_I = x_I_beforeCorrect;
%         P = P_beforeCorrect;
%         all_states(:,end) = x_I;
%     end
    
    % if abs(p(1))>10 || abs(p(2))>10 || abs(p(3))>10, break, end
    if isnan(p), break; end
    % if timeList_IMG(ixIMG) > 40, break; end %stop after 40 seconds
    
    %% draw on-the-fly for debug
    if DEBUG_IMG
        fig = figure(2); hold on;
        plot3(x_I(1), x_I(3), x_I(2), '.b');
        set(gca,'ZDir','Reverse')
        xlabel('X');ylabel('Z');zlabel('Y');
        xlim([-10,10]);ylim([-10,10]);zlim([-10,10]);
        dcm_obj = datacursormode(fig);
        set(dcm_obj,'UpdateFcn',@myupdatefcn)
        grid on; box on;rotate3d on;
        daspect([1 1 1]);
        drawnow;
    end
end

toc

if DEBUG_IMG, figure(2); hold off; end

%%
Visualizer;