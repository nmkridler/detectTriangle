#include "stats.h"

using namespace cv;

//###############################################################
// validTriangle
//
//   check all the vertices to see if they are within the triangle
// 
//###############################################################
bool Stats::validTriangle( const vector<Point> &contour, 
                           const vector<Point> &triangle)
{
   bool foundStray = false;
   for( unsigned int idx = 0; idx < contour.size(); ++idx)
   {
      if( pointPolygonTest(Mat(triangle),contour[idx],false) < 0.0)
         foundStray = true;
   }
   return !foundStray;
}
//###############################################################
// centerOfMass
//
//   determine the object's center of mass
// 
//###############################################################
void Stats::centerOfMass( const vector<Point> &contour, Point &cMass)
{
   // Calculate the moments       
   Moments cMoments = moments(Mat(contour));
   cMass.x = (int)(cMoments.m10/cMoments.m00);
   cMass.y = (int)(cMoments.m01/cMoments.m00);
}

float Stats::colorScore( Scalar &cMean, Scalar &cStdDev)
{
   float colorScore = 0.0;
   colorScore += abs((float)(cMean[0] - KinectConstants::TARGET_COLOR[0]))*
                     (float)cStdDev[0];
   colorScore += abs((float)(cMean[1] - KinectConstants::TARGET_COLOR[1]))*
                     (float)cStdDev[1];
   colorScore += abs((float)(cMean[2] - KinectConstants::TARGET_COLOR[2]))*
                     (float)cStdDev[2];

   return colorScore;
}

