function cam = InitCam()

cam.w = 640;
cam.h = 480;
cam.fx = 540.457688019406;
cam.fy = 538.138092348107;
cam.cx = 319.226514250297;
cam.cy = 235.189957883424;
cam.kx = 0;
cam.ky = 0;

cam.K = [...
   cam.fx,      0,          cam.cx;...
   0,           cam.fy,     cam.cy;...
   0,           0,          1];

end