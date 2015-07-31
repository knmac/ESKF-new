%% Match visual features
% Input: features of previous and current images
% Output:
% - firstColOfPair: first column of the matching inlier pair
% - currNames: name of current feature
% - currFeat: current feature after being sorted according to the feature
% name
% - status: 0=no error, 1=not enough feature, 2=note enough inlier found

function [firstColOfPair, secondColOfPair, currFeat, status] = MatchFeat(prevFeat, currFeat, FEAT_TYPE, DEBUG_IMG)

if strcmp(FEAT_TYPE, 'SURF') || strcmp(FEAT_TYPE, 'HARRIS')
    indexPairs = matchFeatures(prevFeat.features, currFeat.features, 'Unique', true);
elseif strcmp(FEAT_TYPE, 'SIFT')
    [indexPairs, scores] = vl_ubcmatch(prevFeat.features', currFeat.features');
    indexPairs = indexPairs';
    
    % remove duplicated match
    [indexPairs, scores] = RemoveDuplicateMatches(indexPairs, scores);
end

prevMatchPts = prevFeat.imgCoord(indexPairs(:,1), :);
currMatchPts = currFeat.imgCoord(indexPairs(:,2), :);

% Estimate the transformation
[tform, currInliers, prevInliers, status] = estimateGeometricTransform(...
    currMatchPts, prevMatchPts, 'projective', 'Confidence', 99.9, ...
    'MaxNumTrials', 5000);

% alignPts = AlignPoints(currMatchPts, tform.T);
% diff = alignPts - prevMatchPts;
% dist = sqrt(diff(:,1).^2 + diff(:,2).^2);

% currInliers = currMatchPts; prevInliers = prevMatchPts;
if DEBUG_IMG && status == 0
    VisualizeFeatures(prevFeat, prevInliers, currFeat, currInliers);
end

% Remove outlier indices
l = size(prevInliers,1);
rowList = zeros(l,1);
for ix=1:l
    pt = prevInliers(ix,:);
    rowList(ix) = find(ismember(prevMatchPts,pt,'rows'));
end

inlierPairs = indexPairs(rowList,:);

% Generate name for the current inliers
% prevNumFeat = size(prevFeat.features,1);
% currNames= zeros(1,size(currFeat.features,1));
% currNames(inlierPairs(:,2)) =inlierPairs(:,1);
firstColOfPair = inlierPairs(:,1);
secondColOfPair = inlierPairs(:,2);
%
% idxToFill = find(~currNames);
% newNames = prevNumFeat+1 : prevNumFeat+size(idxToFill,2);
% currNames(idxToFill) = newNames;

currInlierIndex = inlierPairs(:,2)';
normalIndexList = 1:size(currFeat.features,1);
newFeatOrder = [currInlierIndex, setdiff(normalIndexList, currInlierIndex)];

% Sort the order of feature according to the name
M = MergeFeatData(currFeat);
% M = SortByOrder(M, inlierPairs(:,2));
M= M(newFeatOrder,:);
[currFeat.imgCoord, currFeat.worldCoord, currFeat.features] = SplitFeatData(M);
end

%%
function M = MergeFeatData(feat)
M = [feat.imgCoord, feat.worldCoord, feat.features];
end

%%
function [imgCoord, worldCoord, features] = SplitFeatData(M)
imgCoord = M(:,1:2);
worldCoord = M(:,3:5);
features = M(:,6:end);
end

%%
function [] = VisualizeFeatures(prevFeat, prevInliers, currFeat, currInliers)
prevImg = imread(prevFeat.imFile);
currImg = imread(currFeat.imFile);

prevKeyPoints = round(prevInliers);
currKeyPoints = round(currInliers);

figure(1);
showMatchedFeatures(prevImg, currImg, prevKeyPoints, currKeyPoints);

% match_plot(prevImg, currImg, prevKeyPoints, currKeyPoints);
end

function h = match_plot(img1,img2,points1,points2)
h = figure;
colormap = {'b','r','m','y','g','c'};
height = max(size(img1,1),size(img2,1));
img1_ratio = height/size(img1,1);
img2_ratio = height/size(img2,1);
img1 = imresize(img1,img1_ratio);
img2 = imresize(img2,img2_ratio);
points1 = points1 * img1_ratio;
points2 = points2 * img2_ratio;
points1 = [points1(:,2) points1(:,1)];
points2 = [points2(:,2) points2(:,1)];
match_img = zeros(height, size(img1,2)+size(img2,2), size(img2,3));
match_img(1:size(img1,1),1:size(img1,2),:) = img1;
match_img(1:size(img2,1),size(img1,2)+1:end,:) = img2;
imshow(uint8(match_img));
hold on;
for i=1:size(points1,1)
    plot([points1(i,2) points2(i,2)+size(img1,2)],[points1(i,1) points2(i,1)],colormap{mod(i,6)+1});
end

hold off;
drawnow
end
%%
% function sortMat = SortByOrder(M, order)
% tranFlag = 0;
%
% if isrow(order)
%     order = order';
% end
%
% if size(M,1) ~= size(order,1)
%     M = M';
%     tranFlag = 1;
% end
%
% bigMat = [order, M];
% bigMat = sortrows(bigMat, 1);
%
% sortMat = bigMat(:,2:end);
%
% if tranFlag
%     sortMat = sortMat';
% end
%
% end

%%
% function alignPts = AlignPoints(points, tform)
% points = ToHomogeneous(points);
% alignPts = tform * points;
% alignPts = FromHomogeneous(alignPts);
% end
%
% function points = ToHomogeneous(points)
% points = [points, ones(size(points,1),1)];
% points = points';
% end
%
% function points = FromHomogeneous(points)
% points = points(1:2, :);
% points = points';
% end