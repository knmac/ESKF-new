#!/usr/bin/python
import time, sys, os, numpy
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image

import ImageFile

def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
    all = []
    left_files = []
    right_files = []
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg', '.ppm']:
                    if 'left' in f or 'left' in path:
                        left_files.append( os.path.join( path, f ) )
                    elif 'right' in f or 'right' in path:
                        right_files.append( os.path.join( path, f ) )
                    all.append( os.path.join( path, f ) )
    return all, left_files, right_files

def CreateMonoBag(imgs,bagname):
    '''Creates a bag file with camera images'''
    bag = rosbag.Bag(bagname, 'w')
    imgs = sorted(imgs)

    try:
        for i in range(len(imgs)):
            print("Adding %s" % imgs[i])
            fp = open( imgs[i], "r" )
            p = ImageFile.Parser()

            rgb_file = imgs[i]
            llim = rgb_file.rfind('/')
            rlim = rgb_file.rfind('.')
            rgb_ext = rgb_file[rlim:]
            msec = rgb_file[llim+1:rlim]
            sec = float(msec) / 1000 # msec to sec

            while 1:
                s = fp.read(1024)
                if not s:
                    break
                p.feed(s)

            im = p.close()

            # Stamp = rospy.rostime.Time.from_sec(time.time())
            Stamp = rospy.rostime.Time.from_sec(sec)
            Img = Image()
            Img.header.stamp = Stamp
            Img.width = im.size[0]
            Img.height = im.size[1]
            Img.encoding = "rgb8"
            Img.header.frame_id = "camera"
            Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
            Img.data = Img_data

            bag.write('camera/rgb/image_color', Img, Stamp)

            #####
            d_file = rgb_file.replace(rgb_ext, '.txt')
            print("Adding %s" % d_file)
            fid = open(d_file, 'r')
            raw = fid.readlines()
            fid.close()
            #depth = numpy.reshape(raw, (im.size[1], im.size[0]))

            Img_depth = Image()
            Img_depth.header.stamp = Stamp
            Img_depth.width = im.size[0]
            Img_depth.height = im.size[1]
            Img_depth.encoding = "rgb8"
            Img_depth.header.frame_id = "camera"
            #Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
            Img_depth.data = raw
            bag.write('camera/depth/image', Img, Stamp)
    finally:
        bag.close()       


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    all_imgs, left_imgs, right_imgs = GetFilesFromDir(args[0])
    if len(all_imgs) <= 0:
        print("No images found in %s" % args[0])
        exit()

    # create bagfile with mono camera image stream
    CreateMonoBag(all_imgs, args[1])

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateBag( sys.argv[1:])
    else:
        print( "Usage: img2bag imagedir bagfilename")