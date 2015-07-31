function  [z, P, H, currFeat, numBlocksToUpdate] = MahalonobisTesting(z, P, H, Z, currFeat, numBlocksToKeep, IMU_LEN, FEAT_LEN)
    threshold = 5.99146454710798198687; %95 percentile of Chi Square Dist.
%     threshold= 0.1025865887751010668524;
%     threshold = 0.01;

    %z(abs(z)<10^-4)=0;
    
    len_z =length(z);
    odd= 1:2:len_z;
    even = 2:2:len_z;
    residual = [z(odd),z(even)];

    gamma = zeros(len_z/2,1); % mahalonobis distance
%     warning('off', 'MATLAB:nearlySingularMatrix'); % suppress singularity warning
    for ix=1:len_z/2           
        e_ix=2*ix;
        s_ix=e_ix-1;
        gamma(ix) = (residual(ix,:) / Z(s_ix:e_ix,s_ix:e_ix)) * residual(ix,:)';
    end
%     warning('on', 'MATLAB:nearlySingularMatrix');
    
    blocksToMove = find(gamma > threshold);
    
    residual(blocksToMove,:)=[];
    residual = residual';
    z = residual(:);
    
%     %---
%     P = RemoveCovariance(P, featToDel);
%     
%     %---
%     temp = 2*featToDel;
%     vertHDel = [temp-1;temp];
%     H(vertHDel,:) = [];
%     
%     temp = 3*featToDel;
%     hortHDel = 18+[temp-2;temp-1;temp];
%     H(:,hortHDel) = [];
%     
%     %---
%     V = diag(repmat(MSR_COV, 1, length(z)/2));
%     Z= H*P*H' + V;
%     
%     %---
%     blocksToKeep(featToDel) = [];
    numBlocksToUpdate = (len_z / 2) - length(blocksToMove);
    [P, H, currFeat] = ReorderAll(blocksToMove, numBlocksToKeep, IMU_LEN, FEAT_LEN, P, H, currFeat);
end