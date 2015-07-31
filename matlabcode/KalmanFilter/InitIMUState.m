%% Initialize IMU state
% IMU starts with:
% - Position p=[0 0 0]
% - Velocity v=[0 0 0]
% - Quaternion q=[1 0 0 0]. The quaternion follows Hamilton format
% [w,x,y,z] (scalar then vector.)
% - Accelerometer bias a_b is estimated using calibration data.
% - Gyroscope bias w_b is estimated using calibration data.
% - Initial gravity g is estimated from the first N acceleration readings.
% The device is assumed to be hold still during that time.

function x_I = InitIMUState(accelList, a_b, w_b)

p = [0 0 0]';
v = [0 0 0]';
q = [1 0 0 0]';

g = -median(accelList, 2) + a_b;

x_I = Data2State(p, v, q, a_b, w_b, g);

end