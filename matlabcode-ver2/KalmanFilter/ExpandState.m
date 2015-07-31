function x_all = ExpandState(x_I, worldPosInlier)

tmp = worldPosInlier';
x_all = [x_I; tmp(:)];

end