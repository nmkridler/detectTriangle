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
   float totalColor = 0.0;
   colorScore += abs((float)(cMean[0] - KinectConstants::TARGET_COLOR[0]))*
                     (float)cStdDev[0];
   //totalColor += KinectConstants::TARGET_COLOR[0];
   totalColor += KinectConstants::TARGET_COLOR[0]*(float)cStdDev[0];
   colorScore += abs((float)(cMean[1] - KinectConstants::TARGET_COLOR[1]))*
                     (float)cStdDev[1];
   //totalColor += KinectConstants::TARGET_COLOR[1];
   totalColor += KinectConstants::TARGET_COLOR[1]*(float)cStdDev[1];
   colorScore += abs((float)(cMean[2] - KinectConstants::TARGET_COLOR[2]))*
                     (float)cStdDev[2];
   //totalColor += KinectConstants::TARGET_COLOR[2];
   totalColor += KinectConstants::TARGET_COLOR[2]*(float)cStdDev[2];
   
   //float stdScore = (float)cStdDev[1] + (float)cStdDev[2];
   return colorScore/totalColor;
}

void Stats::pixelToMetric( vector<Point> & pixelVertex, 
                           vector<float> & distance,
                           vector<Point3f> & metricVertex )
{
   // Make sure metricVertex is empty
   if( !metricVertex.empty() ) metricVertex.clear();

   // loop through the vertices
   for( unsigned int idx = 0; idx < pixelVertex.size(); ++idx)
   {
      float xFact = distance[idx]/(KinectConstants::fx_d);
      float yFact = distance[idx]/(KinectConstants::fy_d);
      Point3f xyzPt((pixelVertex[idx].x - KinectConstants::cx_d)*xFact, 
                    (pixelVertex[idx].y - KinectConstants::cy_d)*yFact,
                     distance[idx]);
      metricVertex.push_back(xyzPt);
   }
} 

float Stats::triangleArea(Vec3f &u, Vec3f &v)
{
   Vec3f w = u.cross(v);
   return 0.5*sqrt(w.dot(w));
}

float Stats::areaError( float const &area)
{
   float diff = area - KinectConstants::TARGET_AREA_METERS;
   return 100.0*sqrt(diff*diff)/KinectConstants::TARGET_AREA_METERS;
}

float Stats::perimeterError( float const &perimeter)
{
   float diff = perimeter - KinectConstants::TARGET_PERIM;
   return 100.0*sqrt(diff*diff)/KinectConstants::TARGET_PERIM;  
}
// Determine if it has a right angle
bool Stats::shapeScore( vector<Point3f> &xyz, float &score)
{
   // Side lengths
   score = 1000000.0;
   if( xyz.size() != 3 ) return false;
   // Combos: 0,1 - 1,2 - 2,0
   Vec3f u(xyz[1].x - xyz[0].x,
           xyz[1].y - xyz[0].y,
           xyz[1].z - xyz[0].z);
       
   Vec3f v(xyz[2].x - xyz[1].x,
           xyz[2].y - xyz[1].y,
           xyz[2].z - xyz[1].z);

   Vec3f w(xyz[2].x - xyz[0].x,
           xyz[2].y - xyz[0].y,
           xyz[2].z - xyz[0].z);

   // Angles (u,v), (u,w), (v,w)
   float uLength = sqrt(u.dot(u));
   float vLength = sqrt(v.dot(v));
   float wLength = sqrt(w.dot(w));

   vector<float> angles;
   angles.push_back(abs(acos(u.dot(w)/(uLength*wLength))));   
   for( unsigned int i = 0; i < 3; ++i) u[i] = -u[i];
   angles.push_back(abs(acos(u.dot(v)/(uLength*vLength))));
   for( unsigned int i = 0; i < 3; ++i)
   {
      v[i] = -v[i];
      w[i] = -w[i];
   }
   angles.push_back(abs(acos(w.dot(v)/(vLength*wLength))));   
   bool rightAngle = false;
   unsigned int idx = 0;
   float ninety = KinectConstants::fPi/2.0;
   float closestTo90 = 100;
   float idx90 = 0;
   while( !rightAngle && idx != angles.size() )
   {
      float rightError = abs(angles[idx] - ninety)/ninety;
      if( rightError < closestTo90)
      {
         closestTo90 = rightError;
         idx90 = idx;
      }
      ++idx;
   }
   //cout << angles[idx90]*radeg << "," << closestTo90 << endl;
   if( closestTo90 < 0.08) rightAngle = true;
   // Calculate area and perimeter
   float perimeter = uLength + vLength + wLength;
   float area = Stats::triangleArea(u,w);
   //cout << "area: " << area << "," << perimeter << endl;
   score = (Stats::areaError(area) + Stats::perimeterError(perimeter) +
           closestTo90*100.0)/3.0;
   return rightAngle;
}
