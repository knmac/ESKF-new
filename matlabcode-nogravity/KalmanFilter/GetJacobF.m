function F_x = GetJacobF(x_I, a_m, w_m, delta_t)

[~, ~, q, a_b, w_b, ~] = State2Data(x_I);
R = Quat2RotMat(q);
I = eye(3);
O = zeros(3);

R_a = -R * GetSkewSymMat(a_m-a_b);
ang = (w_m - w_b)*delta_t;
q = angle2quat(ang(2), ang(1), ang(3), 'YXZ');
R_w = Quat2RotMat(q)';

% Fp = cat(2, I, I*delta_t, O, O, O, O);
% Fv = cat(2, O, I, R_a*delta_t, -R*delta_t, O, I*delta_t);
% Fang = cat(2, O, O, R_w, O, -I*delta_t, O);
% Fa_b = cat(2, O, O, O, I, O, O);
% Fw_b = cat(2, O, O, O, O, I, O);
% Fg = cat(2, O, O, O, O, O, I);
% 
% F_x = cat(1, Fp, Fv, Fang, Fa_b, Fw_b, Fg);

Fp = cat(2, I, I*delta_t, O, O, O);
Fv = cat(2, O, I, R_a*delta_t, -R*delta_t, O);
Fang = cat(2, O, O, R_w, O, -I*delta_t);
Fa_b = cat(2, O, O, O, I, O);
Fw_b = cat(2, O, O, O, O, I);

F_x = cat(1, Fp, Fv, Fang, Fa_b, Fw_b);

end