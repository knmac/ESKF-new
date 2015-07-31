function [delta_p,delta_v,delta_ang,delta_a_b,delta_w_b] = Err2Data(x_I_err)

if isrow(x_I_err)
    x_I_err = x_I_err';
end

delta_p = x_I_err(1:3);
delta_v = x_I_err(4:6);
delta_ang = x_I_err(7:9);
delta_a_b = x_I_err(10:12);
delta_w_b = x_I_err(13:15);
% delta_g = x_I_err(16:18);

end