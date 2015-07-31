% function currFeat = InjectWorldCoord(x_I, x_all_err, currFeat, R_CI, T_IC, featToUpdate, numFeatToUpdate,cam)
%     [p_I, ~, q_GI, ~, ~, ~] = State2Data(x_I); % the quaternion here is from IMU to global frame 
%     R_GI = Quat2RotMat(q_GI'); % TODO: check if inversion needed
%     
%      imgCoord = currFeat.imgCoord(featToUpdate,:);
%     
%     odd = [1:2:numFeatToUpdate*2]+18;
%     even = [2:2:numFeatToUpdate *2]+18;
%     
%     pixelError = [cam.fx*x_all_err(odd)+cam.cx, cam.fy*x_all_err(even)+cam.cy];
%     imgCoord = imgCoord + pixelError;
%     
%     depMat = load(currFeat.depFile);
%     depCoordinates = round(fliplr(imgCoord));
%     depIndices = sub2ind(size(depMat), depCoordinates(:,1), depCoordinates(:,2));
%     depth = depMat(depIndices);
%     
%     imgCoord = bsxfun(@minus, imgCoord, [cam.cx,cam.cy]);
%     imgCoord = bsxfun(@rdivide, imgCoord, [cam.fx,cam.fy]);
%     worldCoord = [imgCoord(1,:).* depth, imgCoord(2,:).* depth, depth];
%     
%     worldCoord=worldCoord';
%     worldCoord= bsxfun(@minus,worldCoord,T_IC);
%     worldCoord = inv(R_CI) * R_GI * worldCoord;
%     worldCoord = bsxfun(@plus, worldCoord, p_I);
%     
%     currFeat.worldCoord(featToUpdate,:) = worldCoord';
% end