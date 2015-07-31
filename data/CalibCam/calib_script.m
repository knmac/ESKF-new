% Auto-generated by cameraCalibrator app on 13-Jul-2015
%-------------------------------------------------------


% Define images to process
imageFileNames = {'101839.jpg',...
    '107201.jpg',...
    '119200.jpg',...
    '122384.jpg',...
    '125836.jpg',...
    '127981.jpg',...
    '136628.jpg',...
    '140349.jpg',...
    '164413.jpg',...
    '170044.jpg',...
    '180233.jpg',...
    '188310.jpg',...
    '195281.jpg',...
    '85517.jpg',...
    '92052.jpg',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Generate world coordinates of the corners of the squares
squareSize = 25;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm');

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams, 'BarGraph');

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
originalImage = imread(imageFileNames{1});
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('SparseReconstructionExample')
save('CameraParams.mat', 'cameraParams');