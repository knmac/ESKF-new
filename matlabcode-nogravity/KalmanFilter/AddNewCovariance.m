function [P,dontAdd] = AddNewCovariance(P, numBlocksToAdd, imgCoord, worldCoord, IMU_LEN, LMK_COV, ...
    Gx_p_handle, Gx_R_handle, Gf_handle, x_I)
[p_I, ~, q_GI, ~, ~, ~] = State2Data(x_I);
[rotY,rotX,rotZ] = quat2angle(q_GI', 'YXZ');

P_xx = P(1:IMU_LEN, 1:IMU_LEN); % robot state part
O3 = zeros(3);
dontAdd=[];
for ix=1:numBlocksToAdd
    u = imgCoord(ix,1);
    v = imgCoord(ix,2);
    Z = worldCoord(ix,3);
    
    P_xf = P(1:IMU_LEN, IMU_LEN+1:end); % robot state - feature parts
    
    Gx_p = Gx_p_handle(rotX, rotY, rotZ, u, v);
%     Gx_p = eye(3);
    Gx_R = Gx_R_handle(Z, rotX, rotY, rotZ, p_I(3), u, v);
    
    % jacobian of inverse observation wrt IMU state
    Gx = cat(2, Gx_p, O3, Gx_R, O3, O3, O3); 
    
    % jacobian of inverse observation wrt features
    Gf = Gf_handle(Z, rotX, rotY, rotZ, p_I(3), u, v); 
    
    P_ll = Gx*P_xx*Gx' + Gf*diag(LMK_COV)*Gf';
    if max(max(P_ll)) > 30
        dontAdd= [dontAdd,ix];
        continue
        
    end
    P_lx = Gx * [P_xx, P_xf];
    P = [P P_lx'; P_lx P_ll]; %#ok<AGROW>
end

end