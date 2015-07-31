function H = GetJacobH(x_I, worldPosInlier, numFeat, R_CI, T_IC, IMU_LEN, FEAT_LEN)

[p_I, ~, q_GI, ~, ~, ~] = State2Data(x_I); % the quaternion here is from IMU to global frame
q_IG = quatinv(q_GI'); % TODO: check if inversion needed
R_IG = Quat2RotMat(q_IG);

% featPosWrtCam = EstimFeatPosWrtCam(p_I, R_IG, worldPosInlier, R_CI, T_IC);
featPosWrtCam = worldPosInlier';

H = zeros(2*numFeat, IMU_LEN+FEAT_LEN*(numFeat));
for ix=1:numFeat
    x = featPosWrtCam(1,ix);
    y = featPosWrtCam(2,ix);
    z = featPosWrtCam(3,ix);
    if z==0, z = eps; end

    J = (1/z) * [1 0 -x/z; 0 1 -y/z];

    Hf = J * R_CI * R_IG;
    ssm = GetSkewSymMat(worldPosInlier(ix,:)' - p_I);
    
    % HI = Hf * cat(2, ssm, -eye(3), zeros(3)); 
    % the order of concatenated matrix should agree with the element order of the state
    HI = Hf * cat(2, -eye(3), zeros(3), ssm, zeros(3,9)); % --> [p, v, ang, a_b, w_b, g]

    headZero = zeros(2, 3*(ix-1));
    tailZero = zeros(2, 3*(numFeat-ix));
    tmp = cat(2, HI, headZero, Hf, tailZero);
    H(2*ix-1:2*ix,:) = tmp;
end

% H = sparse(H);

%% TODO: check the relationship with X (Sola's paper)


end