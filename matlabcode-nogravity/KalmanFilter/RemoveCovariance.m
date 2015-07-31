function [newP] = RemoveCovariance(P, blocksToDelete, IMU_LEN)

if blocksToDelete > 0
    [P_if, P_ff] = deleteCols(P(:,IMU_LEN+1:end), blocksToDelete, IMU_LEN);
    N = IMU_LEN +size(P_if,2);
    newP = zeros(N);

    newP(1:IMU_LEN, 1:IMU_LEN) = P(1:IMU_LEN, 1:IMU_LEN);
    newP(1:IMU_LEN, IMU_LEN+1:N) = P_if;
    newP(IMU_LEN+1:N, 1:IMU_LEN) = P_if';
    newP(IMU_LEN+1:N, IMU_LEN+1:N) = P_ff;
else
    newP =P;
end

end
%% deletes entries
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