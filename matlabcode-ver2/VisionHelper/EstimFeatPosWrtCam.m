%% Estimate feature position in camera frame
% x_I: IMU state
% worldPos: 3xN matrix for x, y, z position of N features in global frame

function featPosWrtCam = EstimFeatPosWrtCam(p_I, R_IG, worldPos, R_CI, T_IC)

if size(worldPos,1) == size(worldPos,2) && size(worldPos,1) == 3
    warning('Warning too few features detected...')
end

if size(worldPos, 2)==3
    worldPos = worldPos';
end

featPosWrtCam = R_CI*R_IG*(bsxfun(@minus,worldPos,p_I));
featPosWrtCam = bsxfun(@plus,featPosWrtCam,T_IC);

end