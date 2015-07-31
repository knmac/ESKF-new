function P = InitCovMat(LMK_COV, IMU_LEN, featnum)
P = eye(IMU_LEN + featnum*3);

% p = [10^-3, 10^-3, 10^-3];
% v = [10^-3, 10^-3, 10^-3];
% ang = [10^-3, 10^-3, 10^-3];
% a_b = [10^-3, 10^-3, 10^-3];
% w_b = [10^-3, 10^-3, 10^-3];
% g = [10^-3, 10^-3, 10^-3];

p = [10^1, 10^1, 10^1];
v = [10^1, 10^1, 10^1];
ang = [10^1, 10^1, 10^1];
a_b = [10^1, 10^1, 10^1];
w_b = [10^1, 10^1, 10^1];
% g = [10^-1, 10^-1, 10^-1];

% P(1:IMU_LEN,1:IMU_LEN) = diag([p, v, ang, a_b, w_b, g]);
P(1:IMU_LEN,1:IMU_LEN) = diag([p, v, ang, a_b, w_b]);

%
initLMK = repmat(LMK_COV, 1, featnum);
initLMK = diag(initLMK);
P(IMU_LEN+1:end, IMU_LEN+1:end) = initLMK;

end