function [newP] = AddNewCovariancePrior(P, numBlocksToAdd, FEAT_LEN, LMK_COV)
    oldSizeP = size(P);      
    
    initLMK = repmat(LMK_COV, 1, numBlocksToAdd);
    initLMK = diag(initLMK);   
    
    P_if_new = zeros(oldSizeP(1), FEAT_LEN*numBlocksToAdd);
    P_fi_new = P_if_new';
    
    newP= [P, P_if_new];
    newP= [newP ; [P_fi_new, initLMK]];
end