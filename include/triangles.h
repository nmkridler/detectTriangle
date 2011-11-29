#ifndef __TRIANGLES_H__
#define __TRIANGLES_H__

#include <cv.h>
#include <math.h>
#include <list>
#include <detection.h>
#include <stats.h>
#include <filters.h>
#include <boost/shared_ptr.hpp>
#include <constants.h>


// This does the detection process
class Triangles : public Detection
{
public:
   // Constructor
   Triangles(Settings const & settings, cv::Size const & frameSize);

   //###############################################################
   // getContour
   //
   //   get a list of contours for this frame
   //
   //###############################################################
   void getContour();
 
   //###############################################################
   // Calculate a score
   //###############################################################
   double contourScore( std::vector<cv::Point> const & triangle,
   		                Feature                      & features);

   //###############################################################
   // pixelDepth
   //###############################################################
   double pixelDepth( cv::Point const & vertex );

   //###############################################################
   // reduceContour
   //###############################################################
   void reduceContour( std::vector<cv::Point> const & contour,
   		               std::vector<cv::Point>       & newTri);

   //###############################################################
   // Set the color mean and standard deviation of the object
   //###############################################################
   void contourColor( std::vector<cv::Point> const & contour,
   		              Feature                      & features);
 

   //###############################################################
   // Process a frame of data
   //###############################################################
   void processFrame(cv::Mat const & rgb, cv::Mat const & depth);

private:

   cv::Mat m_frame;
   cv::Mat m_depth;
   Feature m_target;
   cv::Mat m_inverse;
   static const float k1 = 1.1863;
   static const float k2 = 2842.5;
   static const float k3 = 0.1236;
     
};

// Shared Pointer
typedef boost::shared_ptr<Triangles> TrianglesPtr;
#endif
