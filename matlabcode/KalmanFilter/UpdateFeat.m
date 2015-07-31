% function currFeat = UpdateFeat(x_I,prevFeat,currFeat, blocksToKeep, numBlocksToKeep, R_CI, T_IC)
%     [p_I, ~, q_GI, ~, ~, ~] = State2Data(x_I); % the quaternion here is from IMU to global frame 
%     R_GI = Quat2RotMat(q_GI'); % TODO: check if inversion needed
%     R_IC = inv(R_CI); % rotation matrix from camera to IMU frame
%     
%     worldCoord = currFeat.worldCoord'; % the wordCoord now is wrt current camera
%     worldCoord = bsxfun(@minus,worldCoord,T_IC);
%     worldCoord = R_GI * R_IC * worldCoord;
%     worldCoord = bsxfun(@plus, worldCoord, p_I);
%     
%     % Inject noise from error state
% %     featNoise = x_all_err(IMU_LEN+1:end);
% %     featNoise = reshape(featNoise, 3, []);
% %     
% %     worldCoord(:, 1:numBlocksToKeep) = worldCoord(:, 1:numBlocksToKeep) + featNoise;
% 
%     worldCoord(:, 1:numBlocksToKeep) = prevFeat.worldCoord(blocksToKeep,:)';
%     
%     currFeat.worldCoord = worldCoord';
% end

function [currFeat] = UpdateFeat(x_I,x_all_err,currFeat, numBlocksToKeep, R_CI, T_IC, IMU_LEN)
    [p_I, ~, q_GI, ~, ~, ~] = State2Data(x_I); % the quaternion here is from IMU to global frame 
    R_GI = Quat2RotMat(q_GI'); % TODO: check if inversion needed
    R_IC = inv(R_CI); % rotation matrix from camera to IMU frame
    
    worldCoord = currFeat.worldCoord'; % the wordCoord now is wrt current camera
    worldCoord = bsxfun(@minus,worldCoord,T_IC);
    worldCoord = R_GI * R_IC * worldCoord;
    worldCoord = bsxfun(@plus, worldCoord, p_I);
    
    % Inject noise from error state
    featNoise = x_all_err(IMU_LEN+1:end);
    featNoise = reshape(featNoise, 3, []);
    
    worldCoord(:, 1:numBlocksToKeep) = worldCoord(:, 1:numBlocksToKeep) + featNoise;
    
    currFeat.worldCoord = worldCoord';
end