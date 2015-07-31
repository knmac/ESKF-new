function [currFeat] = UpdateFeat(x_I,x_all_err,currFeat, numBlocksToUpdate, R_CI, T_IC)
    [p_I, ~, q_GI, ~, ~, ~] = State2Data(x_I); % the quaternion here is from IMU to global frame 
    R_GI = Quat2RotMat(q_GI'); % TODO: check if inversion needed
    R_IC = inv(R_CI); % rotation matrix from camera to IMU frame
    
    worldCoord = currFeat.worldCoord'; % the wordCoord now is wrt current camera
    worldCoord = bsxfun(@minus,worldCoord,T_IC);
    worldCoord = R_GI * R_IC * worldCoord;
    worldCoord = bsxfun(@plus, worldCoord, p_I);
    
    % Inject noise from error state
    featNoise = x_all_err(19:end);
    featNoise = reshape(featNoise, 3, []);
    
    worldCoord(:, 1:numBlocksToUpdate) = worldCoord(:, 1:numBlocksToUpdate) + featNoise;
    
    currFeat.worldCoord = worldCoord';
end