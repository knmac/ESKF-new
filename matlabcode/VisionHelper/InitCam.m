function cam = InitCam(CAM_RES)

switch CAM_RES
    case '640x480'
        cam.w = 640;
        cam.h = 480;
        cam.fx = 540.457688019406;
        cam.fy = 538.138092348107;
        cam.cx = 319.226514250297;
        cam.cy = 235.189957883424;
        cam.kc = [0, 0];
        cam.K = [...
            cam.fx,      0,          cam.cx;...
            0,           cam.fy,     cam.cy;...
            0,           0,          1];
    case '320x240'
        load('CameraParams.mat');
        cam.w  = 320;
        cam.h  = 240;
        cam.fx = cameraParams.FocalLength(1);
        cam.fy = cameraParams.FocalLength(2);
        cam.cx = cameraParams.PrincipalPoint(1);
        cam.cy = cameraParams.PrincipalPoint(2);
        cam.kc = cameraParams.RadialDistortion;
        cam.K  = cameraParams.IntrinsicMatrix';
end

end