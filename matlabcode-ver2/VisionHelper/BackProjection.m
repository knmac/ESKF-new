% projection: lambda * p = K*(R_CI*R_IG * (worldCoord - IMU_pos)) + T_IC
function wordCoord = BackProjection(stateX, stateY, stateZ, rotX, rotY, rotZ, u, v, Z)
R_CI = eye(3);
T_IC = zeros(3,1);
cam = InitCam(); K = cam.K;

p_I = [stateX, stateY, stateZ]';
R_GI = Angle2RotMatYXZ(rotY, rotX, rotZ);
uv = [u v 1]';

inv_R = R_GI / R_CI;
inv_K = inv(K);
ray = inv_R * (inv_K * uv - T_IC);
lambda = (Z - stateZ) / ray(3);

wordCoord = inv_R * (inv_K * lambda*uv - T_IC) + p_I;
wordCoord = wordCoord';
end