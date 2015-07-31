function [out] = projection(X,Y,Z,Tx,Ty,Tz, Rx,Ry,Rz)

R= Angle2RotMatYXZ(Ry,Rx,Rz);
cam = InitCam();
K= cam.K;

featCoord = [X;Y;Z];
camCoord = [Tx;Ty;Tz];

K(1,1)=2;K(2,2)=3;K(1,3)=4;K(2,3)=5;
out=R*(featCoord - camCoord);
gamma = out(3);
out = out / gamma;
out = out(1:2);
end