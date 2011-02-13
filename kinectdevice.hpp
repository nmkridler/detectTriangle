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
            float v = i/2048.0;
            v = std::pow(v, 3)* 6;
            m_gamma[i] = v*6*256;
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

      bool getVideo(Mat& output, Mat& corners) {
         m_rgb_mutex.lock();
         if(m_new_rgb_frame) {
            // Run Harris Corner Detection
            cvtColor(rgbMat, output, CV_RGB2BGR);
            getCorners(corners);            
            m_new_rgb_frame = false;
            m_rgb_mutex.unlock();
            return true;
         } else {
            m_rgb_mutex.unlock();
            return false;
         }
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

      void getCorners(Mat& output){
         // Create a corner image and a grayscale
         Mat cornerImg32(rgbMat.rows, rgbMat.cols, CV_32FC1);
         Mat gray;
         cv::cvtColor(rgbMat, gray, CV_RGB2GRAY);
         cornerHarris(gray, cornerImg32, 3, 3, 0.04);

         // Get the min/max
	 double minVal = 0;
         double maxVal = 0;
         minMaxLoc( cornerImg32, &minVal, &maxVal, NULL, NULL);
         double scale = 255.0/(maxVal - minVal);
         double shift = -minVal*scale;
         Size elemSize(3,3);
         Mat structElem = getStructuringElement( MORPH_RECT, elemSize);
         dilate( cornerImg32, cornerImg32, structElem );
         convertScaleAbs(cornerImg32, output, scale, shift);
      }
   private:
      std::vector<uint8_t> m_buffer_depth;
      std::vector<uint8_t> m_buffer_rgb;
      std::vector<uint16_t> m_gamma;
      Mat depthMat;
      Mat rgbMat;
      Mat ownMat;
      Mutex m_rgb_mutex;
      Mutex m_depth_mutex;
      bool m_new_rgb_frame;
      bool m_new_depth_frame;
};


