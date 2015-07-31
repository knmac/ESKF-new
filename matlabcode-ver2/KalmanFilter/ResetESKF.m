function P = ResetESKF(x_all_err, P)

delta_angle = x_all_err(7:9);
P_x = P(1:18,1:18);
G = eye(18);
G(7:9,7:9) = eye(3) - GetSkewSymMat(delta_angle/2);

P_x = G*P_x*G';
P(1:18,1:18) = P_x;

end