function ptCloud = GetPointCloud(imFile, depFile, cam)

im = imread(imFile);
depMat = load(depFile);

x = repmat(1:cam.w, cam.h, 1);
y = repmat((1:cam.h)', 1, cam.w);
z = depMat;

x = (x-cam.cx)/cam.fx;
y = (y-cam.cy)/cam.fy;

xyzPoint = cat(3,x,y,z);

rgb = repmat(im,1,1,3);
ptCloud = pointCloud(xyzPoint, 'Color', rgb);

end