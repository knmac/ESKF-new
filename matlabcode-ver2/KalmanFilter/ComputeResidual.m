function r = ComputeResidual(x_I, worldPosInlier, measuredPosInlier, R_CI, T_IC)

% global 3D -> local 3D
[p_I, ~, q_GI, ~, ~, ~] = State2Data(x_I); % the quaternion here is from IMU to global frame
q_IG = quatinv(q_GI'); % TODO: check if inversion needed
R_IG = Quat2RotMat(q_IG);

localPosInlier = EstimFeatPosWrtCam(p_I, R_IG, worldPosInlier, R_CI, T_IC);

% 3D -> 2D h measurement function
u = localPosInlier(1,:) ./ localPosInlier(3,:); %C_x/C_z
v = localPosInlier(2,:) ./ localPosInlier(3,:); %C_y/C_z
estimCamPos = [u; v];

measuredPosInlier = measuredPosInlier';

y = [measuredPosInlier(1,:) ./ measuredPosInlier(3,:) ;...
    measuredPosInlier(2,:) ./ measuredPosInlier(3,:)];

% find residual
r = y - estimCamPos;
r = r(:);

end