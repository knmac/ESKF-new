function  [z, P, H, Z, V, blocksToKeep] = MahalonobisTesting(z, P, H, Z, MSR_COV, blocksToKeep, IMU_LEN, FEAT_LEN)
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
    
%     if length(featToDel) > 0.8*length(gamma), featToDel = [], end
    
    residual(featToDel,:)=[];
    residual = residual';
    z = residual(:);
    
    %---
    P = RemoveCovariance(P, featToDel, IMU_LEN);
    
    %---
    temp = 2*featToDel;
    vertHDel = [temp-1;temp];
    H(vertHDel,:) = [];
    
    temp = FEAT_LEN*featToDel;
    hortHDel = IMU_LEN + [temp-2;temp-1;temp];
    H(:,hortHDel) = [];
    
    %---
    V = diag(repmat(MSR_COV, 1, length(z)/2));
    Z = H*P*H' + V;
    
    %---
    blocksToKeep(featToDel) = [];
end