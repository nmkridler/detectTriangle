#ifndef __TRIANGLES_H__
#define __TRIANGLES_H__

#include "kinectdevice.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "detection.h"

using namespace cv;
using namespace std;

// This controls the Kinect
class Triangles : public MyFreenectDevice
{
   public:
      // Constructor
      Triangles(freenect_context *_ctx, int _index);

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
  
      // Center of mass getter
      void getDetectCM( Point &cMass ) const;
 
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
 
      void contourImg();
    
      bool foundTarget(); 

   private:
      vector<Detection>  m_triangle;
      Point              m_cMass;
      bool               m_foundTarget;
};

#endif
