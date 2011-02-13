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
    Mat depthf  (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat rgbCor(Size(640,480),CV_8UC3,Scalar(0));
    Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));

    // Kinect device
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

    // Generate windows and start the feed
    namedWindow("rgb",CV_WINDOW_AUTOSIZE);
    namedWindow("depth",CV_WINDOW_AUTOSIZE);
    device.startVideo();
    device.startDepth();

    // As long as the program is alive...
    while (!die) {
        // get frames and push to screen
    	device.getVideo(rgbMat, rgbCor);
    	device.getDepth(depthMat);
        cv::imshow("rgb", rgbMat); 
    	depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
        cv::imshow("depth",rgbCor);
	if(cvWaitKey(30) >= 0){
          cvDestroyWindow("rgb");
          cvDestroyWindow("depth");
          break;
        }
    }
    // Shut down
    device.stopVideo();
    device.stopDepth();
    return 0;
}
