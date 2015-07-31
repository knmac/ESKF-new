function [timeList_IMG, imgList, depList] = ReadVisualData_Kinect(location, timeDiff)

iListing = dir(fullfile(location, '*.pgm'));
dListing = dir(fullfile(location, '*.txt'));
numFile = size(iListing, 1);

timeList_IMG = [];
imgList = {};
depList = {};
pivotTimeIMG = [];

for ix=1:numFile
    imgList{ix} = iListing(ix).name;
    depList{ix} = dListing(ix).name;
    
    sec = strrep(imgList{ix}, 'color.pgm', '');
    sec = str2num(sec);
    
    if isempty(pivotTimeIMG), pivotTimeIMG = sec; end
    
    timeList_IMG(ix) = sec-pivotTimeIMG+timeDiff;
end

end