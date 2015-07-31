function  [z, P, H, Z, V, currFeat, blocksToKeep] = MahalonobisTesting(z, P, H, Z, MSR_COV, currFeat, blocksToKeep, IMU_LEN, FEAT_LEN)
%     threshold = 5.99146454710798198687; %95 percentile of Chi Square Dist.
%     threshold= 0.1025865887751010668524;
    
    %z(abs(z)<10^-4)=0;
    
    percentile = 0.95;
    threshold = chi2inv(percentile, 2*length(blocksToKeep)-3);
    
    len_z =length(z);
    odd= 1:2:len_z;
    even = 2:2:len_z;
    residual = [z(odd),z(even)];

    gamma = zeros(len_z/2,1); % mahalonobis distance
    for ix=1:len_z/2           
        e_ix=2*ix;
        s_ix=e_ix-1;
        gamma(ix) = (residual(ix,:) / Z(s_ix:e_ix,s_ix:e_ix)) * residual(ix,:)';
    end
    
    featToDel = find(gamma > threshold);
    if ~isempty(featToDel)
        featToDel
    end
%     if length(featToDel) > 0.8*length(gamma), featToDel = [], end
    
    residual(featToDel,:)=[];
    residual = residual';
    z = residual(:);
    
    %---
    P = RemoveCovariance(P, featToDel, IMU_LEN);
    
    %---
    temp = 2*featToDel;
    colHDel = [temp-1;temp];
    H(colHDel,:) = [];
    
    temp = FEAT_LEN*featToDel;
    rowHDel = IMU_LEN + [temp-2;temp-1;temp];
    H(:,rowHDel) = [];
    
    %---
    V = diag(repmat(MSR_COV, 1, length(z)/2));
    Z = H*P*H' + V;
    
    %---
    blocksToKeep(featToDel) = [];
    
    %---
    % FIXME: THIS SHOULD HAVE BEEN USED FOR THE SAKE OF UPDATING FEATURES!!!
%     worldCoord = currFeat.worldCoord(featToDel, :);
%     imgCoord = currFeat.imgCoord(featToDel, :);
%     features = currFeat.features(featToDel, :);
%     
%     currFeat.worldCoord(featToDel, :) = [];
%     currFeat.imgCoord(featToDel, :) = [];
%     currFeat.features(featToDel, :) = [];
%     
%     currFeat.worldCoord = [currFeat.worldCoord; worldCoord];
%     currFeat.imgCoord = [currFeat.imgCoord; imgCoord];
%     currFeat.features = [currFeat.features; features];
end