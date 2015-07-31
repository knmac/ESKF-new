function x_I = Data2State(p, v, q, a_b, w_b, g)

if isrow(p), p=p'; end
if isrow(v), v=v'; end
if isrow(q), q=q'; end
if isrow(a_b), a_b=a_b'; end
if isrow(w_b), w_b=w_b'; end
if isrow(g), g=g'; end

x_I = [p' v' q' a_b' w_b' g']';

end