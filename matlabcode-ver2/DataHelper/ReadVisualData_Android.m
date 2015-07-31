function [timeList_IMG, imgList] = ReadVisualData_Android(location, pivotTime)

%% Get image data
listing = dir(fullfile(location, '*.jpg'));
numFile = size(listing, 1);

timeList_IMG = [];
imgList = {};

for ix = 1 : numFile
    imgList{ix} = listing(ix).name;
    
    tmp = strrep(imgList{ix}, 'IMG_', '');
    tmp = strrep(tmp, '.jpg', '');
    tmp = changeTimeFormat(tmp);
    
    timeList_IMG(ix) = (datenum(tmp) - pivotTime) * 24 * 3600; % change to seconds
    
%     % trim excessive images
%     if timeList_IMG(ix) > timeList_IMU(end)
%         break; 
%     end
%     
%     % avoid duplication
%     k = find(timeList_IMU == timeList_IMG(ix));
%     if k~=0
%         timeList_IMG(ix) = timeList_IMG(ix) - (timeList_IMU(k) - timeList_IMU(k-1))/2;
%     end
end

end

%% Change from 'HH-mm-ss-SSS' to 'HH:mm:ss.FFF'
function newTime = changeTimeFormat(oldTime)
newTime = strrep(oldTime, '-', ':');
pos = strfind(newTime, ':');
newTime(pos(end)) = '.';
end