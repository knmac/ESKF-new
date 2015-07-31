function [timeList_IMG, imgList, depList, pivotTime_IMG] = ReadVisualData_Oni(location, timeDiff)

iListing = dir(fullfile(location, '*.jpg'));
dListing = dir(fullfile(location, '*.txt'));
numFile = size(iListing, 1);

timeList_IMG = [];
imgList = {};
depList = {};
pivotTime_IMG = [];

for ix=1:numFile
    imgList{ix} = iListing(ix).name;
    depList{ix} = dListing(ix).name;
    
    msec = strrep(imgList{ix}, '.jpg', '');
    sec = str2num(msec)/1000;
    
    if isempty(pivotTime_IMG), pivotTime_IMG = sec+timeDiff; end
    
    timeList_IMG(ix) = sec+timeDiff-pivotTime_IMG;
end

end