%% Predict covariance matrix of the pertubation impulses

function P_new = PredictCovMat(P, x_I, a_m, w_m, v_b_var, ang_b_var, a_b_var, w_b_var, delta_t, IMU_LEN)

V_i = v_b_var * delta_t^2;
Theta_i = ang_b_var * delta_t^2;
A_i = a_b_var * delta_t;
Omega_i = w_b_var * delta_t;

F_x = GetJacobF(x_I, a_m, w_m, delta_t);
Q_i = diag([V_i; Theta_i; A_i; Omega_i]);
F_i = [zeros(3,12); eye(12,12); zeros(3,12)]; % TODO: change if dont use gravity

P_x = P(1:IMU_LEN,1:IMU_LEN);
P_x = F_x*P_x*F_x' + F_i*Q_i*F_i';

P_xm = P(1:IMU_LEN,IMU_LEN+1:end);
P_xm = F_x * P_xm;

P(1:IMU_LEN,1:IMU_LEN) = P_x;
P(1:IMU_LEN,IMU_LEN+1:end) = P_xm; 
P(IMU_LEN+1:end,1:IMU_LEN) = P_xm';
P_new = P;
end