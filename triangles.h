#ifndef __TRIANGLES_H__
#define __TRIANGLES_H__

#include "kinectdevice.h"
#include <list>
#include "detection.h"
#include "stats.h"
#include "filters.h"
#include <boost/shared_ptr.hpp>

using namespace cv;
using namespace std;
using namespace Orange;

// This controls the Kinect
class Triangles : public MyFreenectDevice
{
   public:
      // Constructor
      Triangles(freenect_context *ctx, int index);

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
     
      //###############################################################
      // Center of mass getter
      // 
      //###############################################################
      vector<cv::Point> const & getDetectCM() const { return m_cMass; }
 
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

      //###############################################################
      // Set the color mean and standard deviation of the object
      // 
      //###############################################################
      float contourColor( vector<Point> &contour);
 
      //###############################################################
      // Extract the image contours
      // 
      //###############################################################
      void contourImg();
    
      //###############################################################
      // Extract the image contours
      // 
      //###############################################################
      bool const &foundTarget(); 
     
   private:
      list<Detection>      m_triangle;    ///< List of detections
      vector<Point>        m_cMass;       ///< Detection locations
      bool                 m_foundTarget; ///< Find anything?
};

// Shared Pointer
typedef boost::shared_ptr<Triangles> TrianglesPtr;
#endif
