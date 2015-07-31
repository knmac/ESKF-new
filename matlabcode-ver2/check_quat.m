close all; clearvars; clc;

w = -180:45:180;
p = -90+1:15:90-1;
lp=length(p);
l = length(w);

x = zeros(lp*l^2,1);
y = zeros(lp*l^2,1);
z = zeros(lp*l^2,1);

x2 = zeros(lp*l^2,1);
y2 = zeros(lp*l^2,1);
z2 = zeros(lp*l^2,1);

count = 1;
for ix=1:lp
    for jx=1:l
        for kx=1:l
            x(count) = p(ix);
            y(count) = w(jx);
            z(count) = w(kx);
            count = count+1;
        end
    end
end

%%
q = angle2quat(deg2rad(y),deg2rad(x),deg2rad(z), 'YXZ');
[y2, x2, z2] = quat2angle(q, 'YXZ');
y2=real(y2);

x2=rad2deg(x2);
y2=rad2deg(y2);
z2=rad2deg(z2);

%%
figure(1); hold on;
plot(x, '-or');
plot(y, '-og');
plot(z, '-ob');
hold off;

figure(2); hold on;
% x2(x2<0) = x2(x2<0) + 180;
plot(x2, '-or');
plot(y2, '-og');
plot(z2, '-ob');
hold off;