function P = ResetESKF(x_all_err, P, IMU_LEN)

delta_angle = x_all_err(7:9);
P_x = P(1:IMU_LEN,1:IMU_LEN);
G = eye(IMU_LEN);
G(7:9,7:9) = eye(3) - GetSkewSymMat(delta_angle/2);

P_x = G*P_x*G';
P(1:IMU_LEN,1:IMU_LEN) = P_x;

end