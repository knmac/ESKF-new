function [] = TrackWorldCoor(prevFeat,firstColOfPair,currFeat)

    pointsToTrack=[110,187;68,325;188,117;85,265];
    
    prevCoor = prevFeat.imgCoord(firstColOfPair,:);
    prevWorldCoor = prevFeat.worldCoord(firstColOfPair,:);
    
    for i=1:4
        point = pointsToTrack(i,:);
        row = prevCoor(:,1) < point(1) +1 & prevCoor(:,1) > point(1) -1;
        col = prevCoor(:,2) < point(2) +1 & prevCoor(:,2) > point(2) -1;
        idx = row & col;
        
        fprintf('img:%d, %d - world:%d, %d, %d\n', point(1), point(2), prevWorldCoor(idx,1),prevWorldCoor(idx,2), prevWorldCoor(idx,3));
    end
end