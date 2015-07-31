function imgFeat = ExtractFeat(imFile, depFile, cam, IGN_MARGIN, STR_FEAT_NUM, DATA_TYPE, FEAT_TYPE)

imgFeat.imFile = imFile;
imgFeat.depFile = depFile;

%% Extract features
im = imread(imFile);
load('CameraParams.mat');
% im = undistortImage(im, cameraParams);
if strcmp(DATA_TYPE, 'ONI'), im = rgb2gray(im); end
switch FEAT_TYPE
    case 'SURF'
        points = detectSURFFeatures(im);
        if ~isempty(STR_FEAT_NUM)
            points = points.selectStrongest(STR_FEAT_NUM);
        end
        [features, points] = extractFeatures(im, points);
        img_x = points.Location(:,1);
        img_y = points.Location(:,2);
    case 'HARRIS'
        points = detectHarrisFeatures(im);
        [features, points] = extractFeatures(im, points);
        features = single(features.Features);
        img_x = points.Location(:,1);
        img_y = points.Location(:,2);
    case 'SIFT'
        [points, features] = vl_sift(single(im));
        
        % remove duplicates
        p = table(points(1,:)', points(2,:)');
        [~, nondup] = unique(p, 'stable'); % keep the same order
        
        img_x = single(points(1,nondup))';
        img_y = single(points(2,nondup))';
        features = single(features(:,nondup))';
end

%% Load depth
switch DATA_TYPE
    case 'ROS'
        depMat = load(depFile);
    case 'ONI'
        depMat = LoadDepth(depFile, cam);
end

%% Remove points outside the ignore margin
ignList = [find(img_x < IGN_MARGIN); find((img_x > cam.w-IGN_MARGIN)); ...
    find(img_y < IGN_MARGIN); find(img_y > cam.h-IGN_MARGIN)];
ignList = unique(ignList);
img_x(ignList) = [];
img_y(ignList) = [];
features(ignList,:) = [];

%% Convert to world indices
imgCoordinates = round([img_y, img_x]);
depIndices = sub2ind(size(depMat), imgCoordinates(:,1), imgCoordinates(:,2));

%% Remove NaN depth
world_Z = depMat(depIndices);
nanList = find(isnan(world_Z));
img_x(nanList) = [];
img_y(nanList) = [];
features(nanList,:) = [];
world_Z(nanList) = [];

%% Convert to world coordinates
world_X = (img_x-cam.cx) / cam.fx;
world_Y = (img_y-cam.cy) / cam.fy;

% img_x = (img_x-cam.cx);
% img_y = (img_y-cam.cy);


%% Distortion model
% r2 = world_X.^2 + world_Y.^2;
% r4 = r2.^2;
% world_X = world_X + world_X.*(cam.kc(1)*r2 + cam.kc(2)*r4);
% world_Y = world_Y + world_Y.*(cam.kc(1)*r2 + cam.kc(2)*r4);
% 
img_x = cam.fx * world_X + cam.cx;
img_y = cam.fy * world_Y + cam.cy;

world_X = world_X .* world_Z;
world_Y = world_Y .* world_Z;

% r2 = img_x.^2 + img_y.^2;
% r4 = r2.^2;
% img_x = img_x + img_x.*(cam.kc(1)*r2 + cam.kc(2)*r4);
% img_y = img_y + img_y.*(cam.kc(1)*r2 + cam.kc(2)*r4);
% 
% world_X = (img_x - cam.cx)/ cam.fx;
% world_Y = (img_y  - cam.cy) / cam.fy;
% 
% img_x = img_x + cam.cx;
% img_y = img_y + cam.cy;
% 
% world_X = world_X .* world_Z;
% world_Y = world_Y .* world_Z;

imgFeat.worldCoord = [world_X, world_Y, world_Z];
imgFeat.imgCoord = [img_x, img_y];
imgFeat.features = features;

%% Transformation matrix
% imgFeat.accTform3d = affine3d(eye(4));

end