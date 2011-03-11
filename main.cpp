#include "libfreenect.hpp"
#include "kinectdevice.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace cv;
using namespace std;


int main(int argc, char **argv) {
    bool die(false);

    // Initialize the buffers
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf  (Size(640,480),CV_8UC3);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat bigMat(Size(1280,480),CV_8UC3,Scalar(0));

    // Kinect device
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

    // Generate windows and start the feed
    namedWindow("rgb and orange",CV_WINDOW_AUTOSIZE);
    //namedWindow("depth",CV_WINDOW_AUTOSIZE);
    device.startVideo();
    device.startDepth();

    bool orangeFilter = true;
    int  key;
    double freenect_angle = 0;
    int iter = 0;
    bool findTriangle = false;
    // As long as the program is alive...
    while (!die) {
        ++iter;
        if( iter == 10 ) findTriangle = true;
        // get frames and push to screen
    	device.getVideo(rgbMat);

        // Triangle detection
        if( iter == 0 ){
           device.setOwnMat();
        }else{
           device.accumOwnMat();
        }
        
        // Get only the orange pixels
        device.filterOrange(ownMat);
        if (orangeFilter){
           device.contourImg();    // contour the image
           Point cMass;
           device.getDetectCM(cMass);
           circle(rgbMat, cMass, 60, Scalar(0,0,255),5);
           findTriangle = false;
           iter = -1;
        }
    	device.getDepth(depthMat);
        device.depthViewColor(depthf);
    	//depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
        Rect leftROI(   Point(0,0),rgbMat.size());
        Rect rightROI(Point(640,0),rgbMat.size());
        Mat leftSide  = bigMat( leftROI);
        Mat rightSide = bigMat(rightROI);
        rgbMat.copyTo( leftSide);
        ownMat.copyTo(rightSide);
        imshow("rgb and orange", bigMat); 
        //imshow("depth",depthf);
        key = cvWaitKey(10);
	if(key == 27){
          cvDestroyWindow("rgb and orange");
          //cvDestroyWindow("depth");
          device.stopVideo();
          device.stopDepth();
          break;
        }
        switch(key){
           case 'f':
              cout << "Toggle Orange Filter " << !orangeFilter << endl;
              orangeFilter = !orangeFilter;
             
           case 'w':
              freenect_angle++;
              if (freenect_angle > 30){
                 freenect_angle = 30;
              }
              device.setTiltDegrees(freenect_angle);
              break; 
           case 'x': 
              freenect_angle--;
              if (freenect_angle < -30){
                 freenect_angle = -30;
              }
              device.setTiltDegrees(freenect_angle);
              break;
           case 's': 
              freenect_angle = 0;
              device.setTiltDegrees(freenect_angle);
              break;
        }
    }
    // Shut down
    return 0;
}
