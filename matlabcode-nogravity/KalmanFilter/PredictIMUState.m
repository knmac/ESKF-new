function nextState = PredictIMUState(IMUstate, a_m, w_m, delta_t)

[p, v, q, a_b, w_b, g] = State2Data(IMUstate);
R = Quat2RotMat(q);   % rotation matrix corresponding to q

%%
rotation = (w_m - w_b) * delta_t;
r = angle2quat(rotation(2), rotation(1), rotation(3), 'YXZ');
q_next = quatmultiply(q', r);
q_next = (q_next / quatnorm(q_next))'; % normalize quaternion

v_next = v + (R*(a_m - a_b) + g) * delta_t;

p_next = p + v*delta_t + 1/2 * (R*(a_m - a_b) + g) * delta_t^2;

% a_x = R*(a_m - a_b) + g;
% a_x=a_x(1);

%%
% nextState = [q_next; p_next; v_next; omega_b_next; a_b_next; g]; %
nextState = Data2State(p_next, v_next, q_next, a_b, w_b, g);
% nextState = round(nextState, 4);