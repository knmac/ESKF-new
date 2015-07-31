function currFeat = dontAddFeat(currFeat, dontAdd, numBlocksToKeep )
    tempImgCoord = currFeat.imgCoord(numBlocksToKeep+dontAdd,:);
    
    tempWorldCoord = currFeat.worldCoord(numBlocksToKeep+dontAdd,:);
    
    tempFeature = currFeat.features(numBlocksToKeep+dontAdd,:);
    
    currFeat.imgCoord(numBlocksToKeep+dontAdd,:)=[];
    currFeat.worldCoord(numBlocksToKeep+dontAdd,:)=[];
    currFeat.features(numBlocksToKeep+dontAdd,:)=[];
    
    currFeat.imgCoord= [currFeat.imgCoord; tempImgCoord];
    currFeat.worldCoord= [currFeat.worldCoord; tempWorldCoord];
    currFeat.features = [currFeat.features; tempFeature];
    
    
end