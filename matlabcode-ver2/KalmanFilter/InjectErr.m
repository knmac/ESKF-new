function x_I = InjectErr(x_I, x_all_err)

x_I_err = x_all_err(1:18);

[p, v, q, a_b, w_b, g] = State2Data(x_I);
[delta_p,delta_v,delta_ang,delta_a_b,delta_w_b,delta_g] = Err2Data(x_I_err);

delta_q = angle2quat(delta_ang(2), delta_ang(1), delta_ang(3), 'YXZ');

p = p + delta_p;
v = v + delta_v;
q = (quatmultiply(q', delta_q))';

a_b = a_b + delta_a_b;
w_b = w_b + delta_w_b;
g = g + delta_g;

x_I = Data2State(p, v, q, a_b, w_b, g);

end