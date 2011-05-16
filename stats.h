#ifndef __STATS_H__
#define __STATS_H__

#include <iostream>
#include <vector>
#include <cmath>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "constants.h"

using namespace std;
using namespace cv;
namespace Stats
{
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

   // Calculate a color score
   float colorScore( Scalar &cMean, Scalar &cStdDev );
 
   void pixelToMetric( vector<Point> & pixelVertex, 
                       vector<float> & distance,
                       vector<Point3f> & metricVertex );
   
   float triangleArea( Vec3f &u, Vec3f &v);
   float areaError( float const &area);
   float perimeterError( float const &perimeter);
   bool  shapeScore( vector<Point3f> &xyz, float &score);
 
}



#endif
