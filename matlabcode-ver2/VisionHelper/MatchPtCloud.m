function currFeat = MatchPtCloud(prevFeat, currFeat, cam, GRIDSIZE)

if isempty(GRIDSIZE)
    GRIDSIZE = 0.1;
end

% Get point clouds
ptCloudRef = GetPointCloud(prevFeat.imFile, prevFeat.depFile, cam);
ptCloudCurrent = GetPointCloud(currFeat.imFile, currFeat.depFile, cam);

% Downsampling the point clouds
fixed = pcdownsample(ptCloudRef, 'gridAverage', GRIDSIZE);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', GRIDSIZE);

% Align point clouds and compute accumulated transformation
[tform, ~, ~] = pcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);

currFeat.accTform3d = affine3d(prevFeat.accTform3d.T * tform.T);

% Update world coordinate of the current feature
l = size(currFeat.worldCoord, 1);
xyz1 = [currFeat.worldCoord'; ones(1, l)];

% Tranpose is there because convention of transform in Matlab is
% different. Translation is at the last row. In our convention it should
% be at last column.
new_xyz1 = inv(currFeat.accTform3d.T') * xyz1;
new_xyz1 = new_xyz1(1:3,:);
currFeat.worldCoord = new_xyz1';

end