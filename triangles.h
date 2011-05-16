#ifndef __TRIANGLES_H__
#define __TRIANGLES_H__

#include "kinectdevice.h"
#include <iostream>
#include <vector>
#include <list>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "detection.h"
#include "stats.h"
#include "filters.h"

using namespace cv;
using namespace std;

// This controls the Kinect
class Triangles : public MyFreenectDevice
{
   public:
      // Constructor
      Triangles(freenect_context *_ctx, int _index);

      //###############################################################
      // getContour
      //
      //   get a list of contours for this frame
      //
      //###############################################################
      void getContour();
 
      //###############################################################
      // processDetection
      //
      //   determine if the contour is a detection
      //   add to detection list
      //
      //###############################################################
      void processDetection( Detection &newDetection);
 
      //###############################################################
      // initializeDetection
      //
      //   initialize the detection
      // 
      //###############################################################
      float contourScore( vector<Point> &contour);

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
      void outputDetections();
     
      // Center of mass getter
      void getDetectCM( vector<Point> &cMass ) const;
 
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

      // Set the color mean and standard deviation of the object
      float contourColor( vector<Point> &contour);
 
      void contourImg();
    
      bool foundTarget(); 
     
   private:
      list<Detection>      m_triangle;
      vector<Point>        m_cMass;
      bool                 m_foundTarget;
};

#endif
