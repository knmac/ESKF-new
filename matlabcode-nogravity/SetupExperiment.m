OPT = 26;

switch OPT
    case 1 % ros - stand
        dataLocation = '../data/Data1/';
        imuFile = strcat(dataLocation, 'sensor_data_still.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 2.3;
        DATA_TYPE = 'ROS';
        CAM_RES = '640x480';
    case 2 % ros - stand 5 min
        dataLocation = '../data/Data2/';
        imuFile = strcat(dataLocation, 'sensor_data_5min.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 3.8263;
        DATA_TYPE = 'ROS';
        CAM_RES = '640x480';
    case 3 % ros - rotate 180
        dataLocation = '../data/Data3/';
        imuFile = strcat(dataLocation, 'sensor_data180.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 1.935;
        DATA_TYPE = 'ROS';
        CAM_RES = '640x480';
    case 4 % ros - small rotate
        dataLocation = '../data/Data4/';
        imuFile = strcat(dataLocation, 'sensor_data_table_rot.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 1.935;
        DATA_TYPE = 'ROS';
        CAM_RES = '640x480';
    %----------------------------------------------------------------------
    case 5 % stand
        dataLocation = '../data/Data5/';
        imuFile = strcat(dataLocation, 'sensor_data_stand.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 6 % bending forward
        dataLocation = '../data/Data6/';
        imuFile = strcat(dataLocation, 'sensor_data_1bending.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 7 % 3 bending
        dataLocation = '../data/Data7/';
        imuFile = strcat(dataLocation, 'sensor_data_3bending.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 8 % move left right
        dataLocation = '../data/Data8/';
        imuFile = strcat(dataLocation, 'sensor_data_leftright.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 9 % move backward + forward
        dataLocation = '../data/Data9/';
        imuFile = strcat(dataLocation, 'sensor_data_backforth.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 10 % move backward + forward (2)
        dataLocation = '../data/Data10/';
        imuFile = strcat(dataLocation, 'sensor_data.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;%-0.009;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 11 % kitchen
        dataLocation = '../data/Data11/';
        imuFile = strcat(dataLocation, 'sensor_data_kitchen.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 12 % kitchen 2
        dataLocation = '../data/Data12/';
        imuFile = strcat(dataLocation, 'sensor_data_kitchen2.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 13 % corridor
        dataLocation = '../data/Data13/';
        imuFile = strcat(dataLocation, 'sensor_data_corridor.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 14 % u-turn
        dataLocation = '../data/Data14/';
        imuFile = strcat(dataLocation, 'sensor_data_uturn.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 15 % stair
        dataLocation = '../data/Data15/';
        imuFile = strcat(dataLocation, 'sensor_data_stair.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 16 % room - 1 turn
        dataLocation = '../data/Data16/';
        imuFile = strcat(dataLocation, 'sensor_data_room.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 17 % long room part 1
        dataLocation = '../data/Data17/';
        imuFile = strcat(dataLocation, 'sensor_data_roomp1.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 18 % long room part 2
        dataLocation = '../data/Data18/';
        imuFile = strcat(dataLocation, 'sensor_data_roomp2.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 19 % long room part 3
        dataLocation = '../data/Data19/';
        imuFile = strcat(dataLocation, 'sensor_data_roomp3.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 20 % long room all
        dataLocation = '../data/Data20/';
        imuFile = strcat(dataLocation, 'sensor_data.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 21 % long room all 2
        dataLocation = '../data/Data21/';
        imuFile = strcat(dataLocation, 'sensor_data.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 22 % room - 3 turns
        dataLocation = '../data/Data22/';
        imuFile = strcat(dataLocation, 'sensor_data_3turns.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 23 % room - 5 turns
        dataLocation = '../data/Data23/';
        imuFile = strcat(dataLocation, 'sensor_data_5turns.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 24 % corridor 2 - fewer features (looking straight)
        dataLocation = '../data/Data24/';
        imuFile = strcat(dataLocation, 'sensor_data_corridor2_lessfeat.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 25 % corridor 3 - more features (looking left and right to find features)
        dataLocation = '../data/Data25/';
        imuFile = strcat(dataLocation, 'sensor_data_corridor3_morefeat.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 26 % stair 2
        dataLocation = '../data/Data26/';
        imuFile = strcat(dataLocation, 'sensor_data_stair2.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 27 % kitchen 3
        dataLocation = '../data/Data27/';
        imuFile = strcat(dataLocation, 'sensor_data_kitchen3.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 28 % blank hallway
        dataLocation = '../data/Data28/';
        imuFile = strcat(dataLocation, 'sensor_data_blankhallway.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
    case 29 % long hallway
        dataLocation = '../data/Data29/';
        imuFile = strcat(dataLocation, 'sensor_data_longhallway.txt');
        visionLocation = strcat(dataLocation, 'RGBD/');
        TIME_DIFF = 0;
        DATA_TYPE = 'ONI';
        CAM_RES = '320x240';
end