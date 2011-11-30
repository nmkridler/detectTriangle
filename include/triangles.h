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
   double pixelDepth( std::vector<cv::Point> const & contour );

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

   //###############################################################
   // Calculate the covariance matrix from the training data
   //###############################################################
   void calcInvCovariance();


private:

   cv::Mat                  m_frame;       // RGB Frame
   cv::Mat                  m_depth;       // Depth frame
   Feature                  m_target;      // Target means
   cv::Mat                  m_inverse;     // Inverse covariance matrix

   // Constants for depth calculation
   static const float k1 = 1.1863;
   static const float k2 = 2842.5;
   static const float k3 = 0.1236;
     
};

// Shared Pointer
typedef boost::shared_ptr<Triangles> TrianglesPtr;
#endif
