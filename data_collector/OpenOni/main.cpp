#include "OpenNI.h"
#include "Device.h"
#include "XnLib.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include <sstream>

using namespace cv;
using namespace openni;
using namespace std;

std::string num2string(float num)
{
	std::ostringstream convert;
	convert << num;
	return convert.str();
}

int main(int argc, char **argv) {
    //all variables needed
    openni::Device device_;         
    openni::VideoStream depth_;      
    openni::VideoStream color_;    
    openni::Status rc_ = STATUS_OK;
    openni::VideoFrameRef depthf_;
    openni::VideoFrameRef colorf_;
    openni::PlaybackControl* playback_;
    uint64_t timestamp;
    uint64_t epoch=0;
    openni::OpenNI::initialize ();
    
    char *cstr;
    if(argc != 2){
        printf("no file name specified\n");
        return -1;
    }
    else{
        cstr = argv[1];
        int length = strlen(cstr);
        char temp[50];
        strcpy(temp, cstr);
        temp[13] = '\0';
        epoch = strtoull(temp,NULL,10);
    }

    //const char *cstr = "1436559886562.oni";
    device_.open(cstr);
    
    depth_.create(device_, openni::SENSOR_DEPTH);
    depth_.start();
    
    device_.setImageRegistrationMode (openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    color_.create(device_, openni::SENSOR_COLOR);
    color_.start();
    
   
    
    playback_ = device_.getPlaybackControl();
    int colorFrameNumber = playback_->getNumberOfFrames(color_);
    int depthFrameNumber = playback_->getNumberOfFrames(depth_);
    int frameNumber;
    playback_->setSpeed(-1);
   
    
    frameNumber = colorFrameNumber <= depthFrameNumber ? colorFrameNumber : depthFrameNumber;
     Mat timestamps = Mat::zeros(frameNumber, 2, CV_64F);
    
    cout << "Frame number: " << frameNumber << endl;       
        
    for (int i = 0; i < frameNumber; i++)
    {
        Mat frame;
        
        // get color frame
        rc_ = color_.readFrame(&colorf_);
        if (rc_ != STATUS_OK) {
            cout << "error with depth read" << endl;
            break;
        }            
        // get depth frame
        rc_ = depth_.readFrame(&depthf_);
        if (rc_ != STATUS_OK){
            cout << "error with depth read" << endl;
            break;
        }
        
        cout << "Frame: " << colorf_.getFrameIndex() << " - " << depthf_.getFrameIndex() <<"/" << frameNumber << endl;
        // get timestamp
        timestamp = colorf_.getTimestamp();
        timestamps.at<double>(i,0) = timestamp/1000.0;
                
        //printf("%llu microseconds\n", timestamp);
        timestamp = epoch + timestamp/1000;
        char fileName[50];
        sprintf(fileName, "data/rgb/%llu.jpg",timestamp);
        
        // write image       
        const openni::RGB888Pixel* imageBuffer = (const openni::RGB888Pixel*)colorf_.getData();
        frame.create(colorf_.getHeight(), colorf_.getWidth(), CV_8UC3);
        memcpy( frame.data, imageBuffer, 3*colorf_.getHeight()*colorf_.getWidth()*sizeof(uint8_t) );
        cv::cvtColor(frame, frame, CV_BGR2RGB); //this will put colors right
        imwrite(fileName, frame);
        
        // write depth

        //printf("%d %d\n", depthf_.getHeight(), depthf_.getWidth());
        const uint16_t *depthRaw = (const uint16_t *) depthf_.getData();
        
        
        //cout << "Dimension: " << depthf_.getWidth() << ' ' << depthf_.getHeight() << endl;

        cv::Mat depthMap = cv::Mat::zeros(depthf_.getHeight(), depthf_.getWidth(), CV_16UC1);
        depthMap.data = (uchar*)depthRaw;
        
        sprintf(fileName, "data/depth_raw/%llu.yml",timestamp);
        
        timestamp = depthf_.getTimestamp();
        timestamps.at<double>(i,1) = timestamp/1000.0;
        
        cv::FileStorage fsDepth(fileName, cv::FileStorage::WRITE);
        fsDepth << "Depth" << depthMap;
        fsDepth.release();
        depthMap.release();
        frame.release();
    }
    
    int irregularity_count = 0;
    int *irr_index;
    irr_index = (int*) malloc(sizeof(int));
    
    Mat diffPeriod = Mat::zeros(frameNumber,1,CV_64F);
    Mat diffRGBD = timestamps.col(0) - timestamps.col(1);
    
    int depthIrregular=0;
    
    for (int i = 1; i < frameNumber; i++)
    {
        double diff = timestamps.at<double>(i,0) - timestamps.at<double>(i-1,0);
        diffPeriod.at<double>(i,0) = diff;
        
        double imgTimestamp = timestamps.at<double>(i,0);
        double depthTimestamp = timestamps.at<double>(i,1);
        double diffDepthImg = imgTimestamp - depthTimestamp;
            
        if(abs(diff) > 35){
            irr_index[irregularity_count] = i;
            irregularity_count++;
            irr_index = (int*) realloc(irr_index, (irregularity_count+1)*sizeof(int));
            
            cout << i << " "<< diff << " " << imgTimestamp << " "<< depthTimestamp << " " << diffDepthImg << endl;
        }
        
        if(abs(diffDepthImg) > 3){
            depthIrregular++;
            cout << "depth irregular: " << depthIrregular << " " << diffDepthImg << endl;
        }
    }
    free(irr_index);
    
    double maxDiffPeriod=0, minDiffPeriod=0;
    double maxDiffRGBD =0, minDiffRGBD=0;
    minMaxLoc(diffPeriod,&minDiffPeriod,&maxDiffPeriod, NULL,NULL,noArray());
    minMaxLoc(diffRGBD,&minDiffRGBD,&maxDiffRGBD, NULL,NULL,noArray());
    Scalar meanPeriod = mean(diffPeriod, noArray());
    Scalar meanRGBD = mean(diffRGBD, noArray());
    
    
    cout << "irregularity count: " << irregularity_count << endl;
    cout << "Max period diff: " << maxDiffPeriod << " Min period diff: " << minDiffPeriod << " Mean period: " << meanPeriod.val[0] << endl;
    cout << "Max RBG-D diff: " << maxDiffRGBD << " Min RBG-D diff: " << minDiffRGBD << " Mean RBG-D diff: " << meanRGBD.val[0] << endl;
    
    long long int startTime = epoch + timestamps.at<double>(0,0);
    long long int stopTime = epoch + timestamps.at<double>(frameNumber-1,0);    
    cout << "start: " << startTime << " stop: " << stopTime << " for frames: " << frameNumber << endl;
    
    if(frameNumber != depthFrameNumber){        
        cout << "number of frames are different" << endl;
        cout << "number of images: " << colorFrameNumber << " " << "number of depth: " << depthFrameNumber << endl;
    }
    
    
   
    playback_->~PlaybackControl();
    color_.stop();
    depth_.stop();
    color_.destroy();
    depth_.destroy();
    depthf_.release();
    colorf_.release();
    device_.close();
   
    openni::OpenNI::shutdown ();
    return 0;
}

