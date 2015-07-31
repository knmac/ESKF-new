function mat33 = GetSkewSymMat(vector)
% Return a 3x3 skew symmetric matrix corresponding to a 3x1 (or 1x3)
% vector.

a = vector(1); b = vector(2); c = vector(3);

mat33 = ...
    [0      -c       b; ...
     c       0      -a; ...
    -b       a       0];

end