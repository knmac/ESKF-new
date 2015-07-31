function rotMat = Quat2RotMat(quaternion)
% Return the rotation matrix of a corresponding quaternion (w,x,y,z)

w = quaternion(1); x = quaternion(2); y = quaternion(3); z = quaternion(4); 
%{
rotMat = ...
    [1-2*y^2-2*z^2,     2*x*y-2*z*w,        2*x*z+2*y*w; ...
    2*x*y+2*z*w,        1-2*x^2-2*z^2,      2*y*z-2*x*w; ...
    2*x*z-2*y*w,        2*y*z+2*x*w,        1-2*x^2-2*y^2];
%}

n = w * w + x * x + y * y + z * z;
if n == 0
    s = 0;
else
    s = 2/n;
end
wx = s * w * x; wy = s * w * y; wz = s * w * z;
xx = s * x * x; xy = s * x * y; xz = s * x * z;
yy = s * y * y; yz = s * y * z; zz = s * z * z;

rotMat = ...
[1 - (yy + zz)         xy - wz          xz + wy  ; ...
      xy + wz     1 - (xx + zz)         yz - wx  ; ...
      xz - wy          yz + wx     1 - (xx + yy) ];
  
end