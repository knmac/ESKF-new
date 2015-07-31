function newP = AddDropCovMat(P, firstColOfPair, LMK_COV)
IMU_LEN = 18;
FEAT_LEN = 3;
FEAT_NUM = length(firstColOfPair);
prevInlierNum = (size(P,1) - IMU_LEN)/FEAT_LEN;

% Init P with IMU part and fill the rest with initial values
newP = eye(IMU_LEN + FEAT_LEN * FEAT_NUM);
newP(1:IMU_LEN, 1:IMU_LEN) = P(1:IMU_LEN, 1:IMU_LEN);

initLMK = repmat(LMK_COV, 1, FEAT_NUM);
initLMK = diag(initLMK);
newP(IMU_LEN+1:end,IMU_LEN+1:end) = initLMK;

% Remove obsolete feature covariance blocks
blocksToKeep = firstColOfPair(firstColOfPair <= prevInlierNum);
blocksToDelete = setdiff(1:prevInlierNum, blocksToKeep);
[P_if, P_ff] = deleteCols(P(:,IMU_LEN+1:end), blocksToDelete, IMU_LEN);

% randsize1 = IMU_LEN + size(P_ff,2);
% randsize2 = FEAT_LEN*FEAT_NUM-size(P_ff,2);


% randMatVert = randn(randsize1,randsize2);
% squareRandMat = randn(randsize2,randsize2);
% squareRandMat = tril(squareRandMat) + triu(squareRandMat);
% squareRandMat = squareRandMat - diag(diag(squareRandMat)) + MSR_COV*eye(randsize2);

% Merge
N = IMU_LEN +size(P_if,2);

newP(1:IMU_LEN, IMU_LEN+1:N) = P_if;
newP(IMU_LEN+1:N, 1:IMU_LEN) = P_if';
newP(IMU_LEN+1:N, IMU_LEN+1:N) = P_ff;


% newP(1:randsize1, end-randsize2+1:end) = randMatVert;
% newP(end-randsize2+1:end, 1:randsize1) = randMatVert';
% newP(randsize1+1:end,randsize1+1:end) = squareRandMat;
end

%% Add new correlation for new features
function [P_if, P_ff] = deleteCols(P_right, blocksToDelete, IMU_LEN)

numBlocksToDelete = length(blocksToDelete);
colsToDelete = zeros(3*numBlocksToDelete,1);
indexToAdd = [-2,-1,0];
for ix=1:numBlocksToDelete
    colsToDelete(ix*3-2:3*ix) = indexToAdd + 3*blocksToDelete(ix);
end

P_right(:,colsToDelete) =[];
P_if = P_right(1:IMU_LEN,:);
P_ff = P_right(IMU_LEN+1:end,:);
P_ff(colsToDelete,:) = [];

end