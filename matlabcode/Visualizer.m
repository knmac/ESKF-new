close all;
DRAW_CORR = 0;
NATURAL_VIEW = 1;

%% visualize result
fprintf('Plotting results...\n');
p = all_states(1:3,:);
v = all_states(4:6,:);
q = all_states(7:10,:);
a_b = all_states(11:13,:);
w_b = all_states(14:16,:);
g = all_states(17:19,:);

corr_idx_idx = find(correct_mark);

% plot position------------------------------------------------------------
fig=figure;hold on;
if NATURAL_VIEW
    plot3(p(1,:), p(3,:), p(2,:), '-');
    if DRAW_CORR
        plot3(p(1,correct_mark>0), p(3,correct_mark>0), p(2,correct_mark>0), 'or');
    end
    view(3);
    set(gca,'ZDir','Reverse');
    xlabel('X-axis (m)');ylabel('Z-axis (m)');zlabel('Y-axis (m)');
    dcm_obj = datacursormode(fig);
    set(dcm_obj,'UpdateFcn',@myupdatefcn);
else
    plot3(p(1,:), p(2,:), p(3,:), '-');
    if DRAW_CORR
        plot3(p(1,correct_mark>0), p(2,correct_mark>0), p(3,correct_mark>0), 'or');
    end
    view(3);
    xlabel('X-axis (m)');ylabel('Y-axis (m)');zlabel('Z-axis (m)');
end
title('Position');
grid on; box on;rotate3d on;
daspect([1 1 1]);
hold off;

% plot velocity------------------------------------------------------------
figure;hold on;
plot(v(1,:),'r');
plot(v(2,:),'g');
plot(v(3,:),'b');
if DRAW_CORR
    plot(corr_idx_idx,v(1,corr_idx_idx),'or');
    plot(corr_idx_idx,v(2,corr_idx_idx),'og');
    plot(corr_idx_idx,v(3,corr_idx_idx),'ob');
end
title('Velocity');
xlabel('Samples');ylabel('m/s');
grid on;
legend('v_x', 'v_y', 'v_z');
hold off;

% plot angle---------------------------------------------------------------
[yaw, pitch, roll] = quat2angle(q', 'YXZ');
yaw = rad2deg(yaw);
pitch = rad2deg(pitch);
roll = rad2deg(roll);

figure;hold on;
plot(pitch,'r');
plot(yaw,'g');
plot(roll,'b');
if DRAW_CORR
    plot(corr_idx_idx,pitch(corr_idx_idx),'or');
    plot(corr_idx_idx,yaw(corr_idx_idx),'og');
    plot(corr_idx_idx,roll(corr_idx_idx),'ob');
end
title('Rotation');
xlabel('Samples');ylabel('Degree');
grid on;
legend('Pitch (X-axis)', 'Yaw (Y-axis)', 'Roll (Z-axis)');
hold off;

% plot accelerometer bias--------------------------------------------------
figure;hold on;
plot(a_b(1,:),'r');
plot(a_b(2,:),'g');
plot(a_b(3,:),'b');
if DRAW_CORR
    plot(corr_idx_idx,a_b(1,corr_idx_idx),'or');
    plot(corr_idx_idx,a_b(2,corr_idx_idx),'og');
    plot(corr_idx_idx,a_b(3,corr_idx_idx),'ob');
end
title('Accelerometer bias');
xlabel('Samples');ylabel('m/s^{2}');
grid on;
legend('a_{b_x}', 'a_{b_y}', 'a_{b_z}');
hold off;

% plot gyro bias-----------------------------------------------------------
figure;hold on;
plot(w_b(1,:),'r');
plot(w_b(2,:),'g');
plot(w_b(3,:),'b');
if DRAW_CORR
    plot(corr_idx_idx,w_b(1,corr_idx_idx),'or');
    plot(corr_idx_idx,w_b(2,corr_idx_idx),'og');
    plot(corr_idx_idx,w_b(3,corr_idx_idx),'ob');
end
title('Gyro bias');
xlabel('Samples');ylabel('rad/s');
grid on;
legend('w_{b_x}', 'w_{b_y}', 'w_{b_z}');
hold off;

% plot gravity-------------------------------------------------------------
figure;hold on;
plot(g(1,:),'r');
plot(g(2,:),'g');
plot(g(3,:),'b');
if DRAW_CORR
    plot(corr_idx_idx,g(1,corr_idx_idx),'or');
    plot(corr_idx_idx,g(2,corr_idx_idx),'og');
    plot(corr_idx_idx,g(3,corr_idx_idx),'ob');
end
title('Gravity');
xlabel('Samples');ylabel('m/s^{2}');
grid on;
legend('g_x', 'g_y', 'g_z');
hold off;

% plot point cloud---------------------------------------------------------
% mergeSize = 0.015;
% bigPtCloud = GetPointCloud(all_features(1).imFile, all_features(1).depFile, cam);
% for ix=2:length(all_features)
%     ptCloud = GetPointCloud(all_features(ix).imFile, all_features(ix).depFile, cam);
%     ptCloud = pctransform(ptCloud, all_features(ix).accTform3d);
%     bigPtCloud = pcmerge(bigPtCloud, ptCloud, mergeSize);
% end
% figure;showPointCloud(bigPtCloud);
% xlabel('X-axis (m)');ylabel('Y-axis (m)');zlabel('Z-axis (m)');
% title('Point cloud');
