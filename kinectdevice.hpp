#include "libfreenect.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace cv;
using namespace std;

// This controls the threads
class Mutex {
   public:
      Mutex() {
         pthread_mutex_init( &m_mutex, NULL );
      }
      void lock() {
         pthread_mutex_lock( &m_mutex );
      }
      void unlock() {
         pthread_mutex_unlock( &m_mutex );
      } 
   private:
      pthread_mutex_t m_mutex;
};

// This controls the Kinect
class MyFreenectDevice : public Freenect::FreenectDevice {
   public:
        // Constructor
      MyFreenectDevice(freenect_context *_ctx, int _index)
        : Freenect::FreenectDevice(_ctx, _index),         // Device Constructor
         m_buffer_depth(FREENECT_DEPTH_11BIT_SIZE),      // depth buffer
         m_buffer_rgb(FREENECT_VIDEO_RGB_SIZE),          // RGB buffer
         m_gamma(2048),                                  // gamma
         depthMat(Size(640,480),CV_16UC1),               // Depth matrix 
         rgbMat(Size(640,480),CV_8UC3,Scalar(0)),        // RGB matrix
         ownMat(Size(640,480),CV_8UC3,Scalar(0)),         // own matrix
         m_new_rgb_frame(false),                         // new RGB frame
         m_new_depth_frame(false)                        // new depth frame
      {
           // Fill the gamma array
         for( unsigned int i = 0 ; i < 2048 ; i++) 
         {
            const float k1 = 1.1863;
            const float k2 = 2852.5;
            const float k3 = 0.1236;
            //float v = i/2048.0;
            //v = std::pow(v, 3)* 6;
            const float v = k3*tanf(i/k2 + k1);
            m_gamma[i] = v;//*6*256;
         }
      }
      // Do not call directly even in child
      void VideoCallback(void* _rgb, uint32_t timestamp) {
          //std::cout << "RGB callback" << std::endl;
          m_rgb_mutex.lock();
          uint8_t* rgb = static_cast<uint8_t*>(_rgb); 
          rgbMat.data = rgb;
          m_new_rgb_frame = true;
          m_rgb_mutex.unlock();
      }
      // Do not call directly even in child
      void DepthCallback(void* _depth, uint32_t timestamp) {
          //std::cout << "Depth callback" << std::endl;
          m_depth_mutex.lock();
          uint16_t* depth = static_cast<uint16_t*>(_depth);
          depthMat.data = (uchar*) depth;
          m_new_depth_frame = true;
          m_depth_mutex.unlock();
      }

      bool getVideo(Mat& output) {
         m_rgb_mutex.lock();
         if(m_new_rgb_frame) {
            cvtColor(rgbMat, output, CV_RGB2BGR);
            m_new_rgb_frame = false;
            m_rgb_mutex.unlock();
            return true;
         } else {
            m_rgb_mutex.unlock();
            return false;
         }
      }

      void setOwnMat( void ) {
         m_rgb_mutex.lock();
         cvtColor(rgbMat, ownMat, CV_RGB2BGR);
         m_rgb_mutex.unlock();
      }

      bool getDepth(Mat& output) {
         m_depth_mutex.lock();
         if(m_new_depth_frame) {
            depthMat.copyTo(output);
            m_new_depth_frame = false;
            m_depth_mutex.unlock();
            return true;
         } else {
            m_depth_mutex.unlock();
            return false;
         }
      }
      
      void getBinary(Mat& output){
         Mat gray;
         cvtColor(ownMat, gray, CV_BGR2GRAY);
         GaussianBlur(gray, gray, Size(7,7), 1.5, 1.5);
         Canny(gray, gray, 0, 30, 3);
         dilate(gray, output, Mat());
         output = output > 128;
      } 

      void getContour( Mat& output){
         Mat dst = Mat::zeros(output.size(), CV_8UC1);
         Mat gray, tmpMat;
         getBinary(gray);
         getBinary(tmpMat);
         
         // Contour info
         vector<vector<Point> > contours;
         vector<Vec4i> hierarchy;
         findContours(gray, contours, hierarchy,
                      CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
         if( contours.size() > 0 ){
            int idx = 0;
            while( idx < (int)contours.size() ){
               vector<Point> approx;
               approxPolyDP(Mat(contours[idx]), approx, 5, true);
               double area1 = contourArea(approx);
               if( area1 > 0.0 && approx.size()==3 ){
                  Scalar color(255,255,255);
                  fillConvexPoly(dst, contours[idx].data(), 
                                 contours[idx].size(), color);
               }
               idx++;
            }
            
            dst.copyTo(output);
         } 
         else 
         {
           tmpMat.copyTo(output);
           cout << "No Contours" << endl;
         }
      }
 
      void contourImg(Mat& output)
      {
         setOwnMat();
         // convert to HSV
         //Mat tmpImg;
         //cvtColor(ownMat, tmpImg, CV_BGR2HSV);
         //Mat orangeImg(ownMat.size(), CV_8UC1);
         //inRange(tmpImg, Scalar(0,0,0), Scalar(255,130,255), orangeImg);
         //ownMat.setTo(Scalar(0,0,0),orangeImg);
         
         Mat contour, mask;
         getBinary(contour);
         getContour(contour);
         setOwnMat();
         inRange(contour, Scalar(0), Scalar(254), mask);
         ownMat.setTo(Scalar(0,0,0),mask);
         ownMat.copyTo(output);
         Scalar matSum = mean(ownMat,mask);
         cout << matSum[0] << " " << matSum[1] << " " << matSum[2] << endl;
         Scalar depSum = mean(depthMat,mask);
         int depIdx = (int)depSum[0] - 1;
         if( depIdx >= 0 ) cout << "Depth: " << m_gamma[depIdx] << endl;
         cvWaitKey(100);
      }

   private:
      std::vector<uint8_t> m_buffer_depth;
      std::vector<uint8_t> m_buffer_rgb;
      std::vector<float> m_gamma;
      Mat depthMat;
      Mat rgbMat;
      Mat ownMat;
      Mutex m_rgb_mutex;
      Mutex m_depth_mutex;
      bool m_new_rgb_frame;
      bool m_new_depth_frame;
};


