#ifndef __KINECTDEVICE_H__
#define __KINECTDEVICE_H__

#include "libfreenect.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <boost/shared_ptr.hpp>

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
// OpenKinect.org is responsible for most of this
class KinectDevice : public Freenect::FreenectDevice {
   public:
      // Constructor
      KinectDevice(freenect_context *_ctx, int _index);
      
      // Do not call directly even in child
      void VideoCallback(void* _rgb, uint32_t timestamp);

      // Do not call directly even in child
      void DepthCallback(void* _depth, uint32_t timestamp);

      // RGB Video getter 
      bool getVideo(Mat & output);

      // Depth getter
      bool getDepth(Mat & output);
      
      // Convert depthMat into color
      void depthViewColor(Mat & output);

   protected:
      std::vector<float> m_gamma;
      Mat depthMat;
      Mat rgbMat;

   private:
      std::vector<uint8_t> m_buffer_depth;
      std::vector<uint8_t> m_buffer_rgb;
      Mutex m_rgb_mutex;
      Mutex m_depth_mutex;
      bool m_new_rgb_frame;
      bool m_new_depth_frame;
};

typedef boost::shared_ptr<KinectDevice> KinectDevicePtr;

#endif
