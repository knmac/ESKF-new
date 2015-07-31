function [p, v, q, a_b, w_b, g] = State2Data(x_I)

if isrow(x_I)
    x_I = x_I';
end

p = x_I(1:3);
v = x_I(4:6);
q = x_I(7:10);
a_b = x_I(11:13);
w_b = x_I(14:16);
g = x_I(17:19);

end