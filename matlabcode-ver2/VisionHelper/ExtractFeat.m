function imgFeat = ExtractFeat(imFile, depFile, cam, IGN_MARGIN, STR_FEAT_NUM)
imgFeat.imFile = imFile;
imgFeat.depFile = depFile;

% Extract features
im = imread(imFile);
points = detectSURFFeatures(im);
if ~isempty(STR_FEAT_NUM)
    points = points.selectStrongest(STR_FEAT_NUM);
end
[features, points] = extractFeatures(im, points);


depMat = load(depFile);
img_x = points.Location(:,1);
img_y = points.Location(:,2);

% Remove points outside the ignore margin
ignList = [find(img_x < IGN_MARGIN); find((img_x > cam.w-IGN_MARGIN)); ...
    find(img_y < IGN_MARGIN); find(img_y > cam.h-IGN_MARGIN)];
ignList = unique(ignList);
img_x(ignList) = [];
img_y(ignList) = [];
features(ignList,:) = [];

% Convert to world indices
imgCoordinates = round([img_y, img_x]);
depIndices = sub2ind(size(depMat), imgCoordinates(:,1), imgCoordinates(:,2));

% Remove NaN depth
world_Z = depMat(depIndices);
nanList = find(isnan(world_Z));
img_x(nanList) = [];
img_y(nanList) = [];
features(nanList,:) = [];
world_Z(nanList) = [];

% Convert to world coordinates
world_X = (img_x-cam.cx) / cam.fx;
world_Y = (img_y-cam.cy) / cam.fy;

imgFeat.worldCoord = [world_X, world_Y, world_Z];
imgFeat.imgCoord = [img_x, img_y];
imgFeat.features = features;

% Transformation matrix
% imgFeat.accTform3d = affine3d(eye(4));

end