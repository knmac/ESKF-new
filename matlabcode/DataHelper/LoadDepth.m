function depMat = LoadDepth(depFile, cam)

if ~isempty(cam)
    w = cam.w;
    h = cam.h;
else
    w = 320;
    h = 240;
end

raw = load(depFile);
depMat = reshape(raw, w, h)' / 1000;
% depMat(depMat == 0) = NaN;

% Eliminate points with out-of-range depth. The range might depends on the
% devices.
MIN_DEP = 0.08;
MAX_DEP = 3.5;

depMat(depMat < MIN_DEP) = NaN;
depMat(depMat > MAX_DEP) = NaN;

end