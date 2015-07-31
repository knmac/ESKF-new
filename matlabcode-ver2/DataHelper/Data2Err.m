function x_I_err = Data2Err(delta_p,delta_v,delta_ang,delta_a_b,delta_w_b,delta_g)

if isrow(delta_p), delta_p=delta_p'; end
if isrow(delta_v), delta_v=delta_v'; end
if isrow(delta_ang), delta_ang=delta_ang'; end
if isrow(delta_a_b), delta_a_b=delta_a_b'; end
if isrow(delta_w_b), delta_w_b=delta_w_b'; end
if isrow(delta_g), delta_g=delta_g'; end

x_I_err = [delta_p' delta_v' delta_ang' delta_a_b' delta_w_b' delta_g']';

end