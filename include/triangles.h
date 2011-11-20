#ifndef __TRIANGLES_H__
#define __TRIANGLES_H__

#include <cv.h>
#include <math.h>
#include <list>
#include <detection.h>
#include <stats.h>
#include <filters.h>
#include <boost/shared_ptr.hpp>

using namespace cv;
using namespace std;
using namespace Orange;

// This does the detection process
class Triangles : public Detection
{
public:
   // Constructor
   Triangles(Settings const & settings);

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
   void processDetection( Contact &newDetection);
 
   //###############################################################
   // initializeDetection
   //
   //   initialize the detection
   //
   //###############################################################
   float contourScore( vector<Point> &contour);
     
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
   // Process a frame of data
   //
   //###############################################################
   void processFrame(cv::Mat const & rgb, cv::Mat const & depth);
private:

   cv::Mat m_frame;
   cv::Mat m_depth;

   static const float k1 = 1.1863;
   static const float k2 = 2842.5;
   static const float k3 = 0.1236;
     
};

// Shared Pointer
typedef boost::shared_ptr<Triangles> TrianglesPtr;
#endif
