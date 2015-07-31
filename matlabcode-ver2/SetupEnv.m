%% Setup working environment

%% Add paths
addpath('DataHelper/')
addpath('KalmanFilter/')
addpath('VisionHelper/');

%% Constant values
UNIT_G = 9.80665;               % standard gravity value (m/s^2)
% GYRO_SENSTV = 0.00116;        % gyroscope sensitivity (rad/s)
GYRO_SENSTV = 1;                % gyroscope sensitivity (rad/s)
IMU_CAL_FILE = 'imu_cal.mat';   % where to save IMU calibration values
IGN_MARGIN = 10;                % margin on image that can be ignored (in pixel)
GRIDSIZE = 0.1;                 % gridsize to downsample the point cloud
xyzAngle = deg2rad([0, 0, 0]');
R_CI = Angle2RotMatYXZ(xyzAngle(2),xyzAngle(1),xyzAngle(3));  % FIXME: rotation from IMU frame to camera frame
T_IC = [0 0 0]';                % FIXME: translation from camera to IMU
STR_FEAT_NUM = 100;              % number of strongest SURF features to keep. [] = take all
IMU_LEN = 18;                   % Length of IMU part for covariance matrices
FEAT_LEN = 3;                   % Length of each feature part
LMK_COV = [0.1,0.1,0.1];             % initial covariance for landmarks
MSR_COV = [0.1,0.1];                % initial covariance for vision measurement