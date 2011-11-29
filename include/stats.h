#ifndef __STATS_H__
#define __STATS_H__

#include <iostream>
#include <vector>
#include <cmath>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <constants.h>


namespace Stats
{
   //###############################################################
   // validTriangle
   //###############################################################
   bool validTriangle( std::vector<cv::Point> const & contour,
                       std::vector<cv::Point> const & triangle);

   //###############################################################
   // centerOfMass
   //###############################################################
   void centerOfMass( std::vector<cv::Point> const & contour, cv::Point & cMass);
 
   
   //###############################################################
   // Transform to metric space
   //###############################################################
   void pixelToMetric( std::vector<cv::Point>   const & pixelVertex,
                       std::vector<double>      const & distance,
                       std::vector<cv::Point3d>       & metricVertex );

   //###############################################################
   // area of a triangle
   //###############################################################
   double triangleArea( cv::Vec3d const & u, cv::Vec3d const & v);

   //###############################################################
   // Triangle shape ( sides / angles)
   //###############################################################
   void shape( std::vector<cv::Point3d>  const  & xyz,
		       double                           & perimeter,
		       double                           & angle,
		       double                           & area);

   //###############################################################
   // Similarity metric
   //###############################################################
   double similarity(Feature const & u, Feature const & v, cv::Mat const & iCovar);
 
}



#endif
