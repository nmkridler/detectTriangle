#ifndef __KINECTDEVICE_HPP__
#define __KINECTDEVICE_HPP__

#include "libfreenect.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "detection.hpp"

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
      MyFreenectDevice(freenect_context *_ctx, int _index);
           // Do not call directly even in child
      void VideoCallback(void* _rgb, uint32_t timestamp);

      // Do not call directly even in child
      void DepthCallback(void* _depth, uint32_t timestamp);

      bool getVideo(Mat& output);

      void setOwnMat( void );

      bool getDepth(Mat& output);

      //###############################################################
      // depthViewColor
      //
      //   convert depthMat into color
      //
      //###############################################################
      void depthViewColor(Mat& output);

      //###############################################################
      // getBinary
      //
      //   turn ownMat into a binary image
      //
      //###############################################################
      void getBinary(Mat& output);

      //###############################################################
      // getContour
      //
      //   get a list of contours for this frame
      //
      //###############################################################
      void getContour();
 
      //###############################################################
      // processContour
      //
      //   determine if the contour is a detection
      //   add to detection list
      //
      //###############################################################
      void processContour( const vector<Point> &contour );
 
      //###############################################################
      // initializeDetection
      //
      //   initialize the detection
      // 
      //###############################################################
      void initializeDetection( Detection& newDetection);
     
      //###############################################################
      // resetDetections
      //
      //   set everything to missed
      // 
      //###############################################################
      void resetDetections();
    
      //###############################################################
      // reduceDetections
      //
      //   cut out missed detections
      // 
      //###############################################################
      void reduceDetections();
  
      //###############################################################
      // outputDetections
      //
      //   create a mask with all of the detections
      // 
      //###############################################################
      void outputDetections(Mat& output);
     
      //###############################################################
      // validTriangle
      //
      //   check all the vertices to see if they are within the triangle
      // 
      //###############################################################
      bool validTriangle( const vector<Point> &contour, 
                          const vector<Point> &triangle);
       
      //###############################################################
      // centerOfMass
      //
      //   determine the object's center of mass
      // 
      //###############################################################
      void centerOfMass( const vector<Point> &contour, Point &cMass);
   
      //###############################################################
      // pixelDepth
      //
      //   determine the depth in meters
      // 
      //###############################################################
      float pixelDepth( Point &vertex );
     
      //###############################################################
      // reduceContour
      //
      //   reduces the contour to the three vertices furthest from 
      //   the center of mass
      // 
      //###############################################################
      void reduceContour( const vector<Point> &contour, vector<Point> &newTri);

       // Filter the feed to get orange only 
      void filterOrange( Mat& output);
    
      // Set the color mean and standard deviation of the object
      void contourColor( Detection &newDetection);
 
      void contourImg(Mat& output);
     

   private:
      std::vector<uint8_t> m_buffer_depth;
      std::vector<uint8_t> m_buffer_rgb;
      std::vector<float> m_gamma;
      vector<Detection> m_triangle;
      Mat depthMat;
      Mat rgbMat;
      Mat ownMat;
      Mutex m_rgb_mutex;
      Mutex m_depth_mutex;
      bool m_new_rgb_frame;
      bool m_new_depth_frame;
};

#endif
