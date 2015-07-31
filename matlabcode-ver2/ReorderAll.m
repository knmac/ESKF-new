function [P, H, feat] = ReorderAll(blocksToMove, numBlocksToKeep, IMU_LEN, FEAT_LEN, P, H, feat)

P = ReorderP(blocksToMove, IMU_LEN, FEAT_LEN, P);
feat = ReorderFeat(blocksToMove, numBlocksToKeep, feat);
H= DropH(H,blocksToMove, IMU_LEN, FEAT_LEN);

end

function P = ReorderP(blocksToMove, IMU_LEN, FEAT_LEN, P)

while ~isempty(blocksToMove)
    ix = blocksToMove(1);
    start = IMU_LEN+FEAT_LEN*ix-2;
    fin = IMU_LEN+FEAT_LEN*ix;
    
    % copy values
    blck = P(start:fin, start:fin);
    rows = P(start:fin, :);
    cols = P(:, start:fin);
    
    % remove blocks
    P(start:fin, :) = [];
    P(:, start:fin) = [];
    rows(:, start:fin) = [];
    cols(start:fin, :) = [];
    
    % add blocks
    P = [P cols; rows blck];
    
    % next
    blocksToMove(1) = [];
    blocksToMove = blocksToMove - 1;
end

end

function [H] = DropH(H, blocksToMove, IMU_LEN, FEAT_LEN)
    temp = 2*blocksToMove;
    vertHDel = [temp-1;temp];
    H(vertHDel,:) = [];
    
    temp = FEAT_LEN*blocksToMove;
    hortHDel = IMU_LEN+[temp-2;temp-1;temp];
    H(:,hortHDel) = [];
end

function [feat] = ReorderFeat(blocksToMove, numBlocksToKeep, feat)

    newOrder = setdiff(1:numBlocksToKeep, blocksToMove);
    newOrder = [newOrder, blocksToMove'];

    feat.imgCoord(1:numBlocksToKeep, :) = feat.imgCoord(newOrder, :);
    feat.worldCoord(1:numBlocksToKeep, :) = feat.worldCoord(newOrder, :);
    feat.features(1:numBlocksToKeep, :) = feat.features(newOrder, :);

end