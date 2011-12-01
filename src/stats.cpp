#include <stats.h>

bool Stats::validTriangle( std::vector<cv::Point> const & contour,
                           std::vector<cv::Point> const & triangle)
{
   bool foundStray = false;
   for( unsigned int idx = 0; idx < contour.size(); ++idx)
   {
      if( fabs(cv::pointPolygonTest(cv::Mat(triangle),contour[idx],true)) > 10)
         foundStray = true;
   }
   return !foundStray;
}

void Stats::centerOfMass( std::vector<cv::Point> const & contour, cv::Point & cMass)
{
   // Calculate the moments       
   cv::Moments cMoments = cv::moments(cv::Mat(contour));
   cMass.x = static_cast<int>(cMoments.m10/cMoments.m00);
   cMass.y = static_cast<int>(cMoments.m01/cMoments.m00);
}

void Stats::pixelToMetric( std::vector<cv::Point>   const & pixelVertex,
                           std::vector<double>      const & distance,
                           std::vector<cv::Point3d>       & metricVertex )
{
   // Make sure metricVertex is empty
   if( !metricVertex.empty() ) metricVertex.clear();

   // loop through the vertices
   for( unsigned int idx = 0; idx < pixelVertex.size(); ++idx)
   {

      double xFact = distance[idx]/(Kinect::fx_d);
      double yFact = distance[idx]/(Kinect::fy_d);
      cv::Point3d xyzPt((static_cast<double>(pixelVertex[idx].x) - Kinect::cx_d)*xFact,
                        (static_cast<double>(pixelVertex[idx].y) - Kinect::cy_d)*yFact,
                         distance[idx]);
      metricVertex.push_back(xyzPt);
   }

} 
//###############################################################
// area of a triangle
//###############################################################
double Stats::triangleArea( cv::Vec3d const & u, cv::Vec3d const & v)
{
   cv::Vec3d w = u.cross(v);
   return 0.5*sqrt(w.dot(w));
}

//###############################################################
// Triangle shape ( sides / angles)
//###############################################################
void Stats::shape( std::vector<cv::Point3d>  const  & xyz,
	     	       double                           & perimeter,
		           double                           & angle,
		           double                           & area)
{
   // Set the perimeter to 0
   perimeter = 0.;
   angle  = 0.;
   area   = 0.;
   if( xyz.size() == 3 )
   {
      // Combos: 0,1 - 1,2 - 2,0
      cv::Vec3d u(xyz[1].x - xyz[0].x,
                  xyz[1].y - xyz[0].y,
                  xyz[1].z - xyz[0].z);
       
      cv::Vec3d v(xyz[2].x - xyz[1].x,
                  xyz[2].y - xyz[1].y,
                  xyz[2].z - xyz[1].z);

      cv::Vec3d w(xyz[2].x - xyz[0].x,
                  xyz[2].y - xyz[0].y,
                  xyz[2].z - xyz[0].z);

      // Angles (u,v), (u,w), (v,w)
      double uLength = sqrt(u.dot(u));
      double vLength = sqrt(v.dot(v));
      double wLength = sqrt(w.dot(w));

      perimeter = uLength + vLength + wLength;

      // Determine all of the angles
      std::vector<double> angles;
      angles.push_back(std::abs(acos(u.dot(w)/(uLength*wLength))));
      for( size_t i = 0; i < 3; ++i) u[i] *= -1.;
      angles.push_back(std::abs(acos(u.dot(v)/(uLength*vLength))));
      for( size_t i = 0; i < 3; ++i)
      {
         v[i] *= -1;
         w[i] *= -1;
      }
      angles.push_back(std::abs(acos(w.dot(v)/(vLength*wLength))));

      // Figure out which one is the max
      angle = angles[0];
      for( size_t i = 1; i < 3; ++i)
      {
    	  if(angles[i] > angle) angle = angles[i];
      }
      angle *= (180./Kinect::fPi);
      area = Stats::triangleArea(u,w);
   }

}

double Stats::similarity(Feature const & u, Feature const & v)
{
#if 1
   for(size_t i = 0; i < 6; ++i) std::cout << u[i] << ",";
#endif
   double normU = 0;
   double normV = 0;
   double dotUV = 0;
   for( size_t i = 0; i < 6; ++i)
   {
	   normU += (u[i]*u[i]);
	   normV += (v[i]*v[i]);
	   dotUV += (u[i]*v[i]);
   }
   return dotUV/(sqrt(normV)*sqrt(normU));
}

double Stats::overlap(Contact const & boxA, Contact const & boxB)
{
	if( boxA.position.x > boxB.position.x + boxB.dims.x ||
	    boxB.position.x > boxA.position.x + boxA.dims.x ||
	    boxA.position.y > boxB.position.y + boxB.dims.y ||
	    boxB.position.y > boxA.position.y + boxA.dims.y ) return 0.;

	// The bounding boxes overlap, determine the area of overlap
	double width  = static_cast<double>(std::min(boxA.position.x + boxA.dims.x,
			                                     boxB.position.x + boxB.dims.x) -
			                            std::max(boxA.position.x,boxB.position.x));
	double height = static_cast<double>(std::min(boxA.position.y + boxA.dims.y,
			                                     boxB.position.y + boxB.dims.y) -
			                            std::max(boxA.position.y,boxB.position.y));

	double totalArea = static_cast<double>(boxA.dims.y*boxA.dims.x + boxB.dims.x*boxB.dims.y) - width*height;

	// Amount of overlap
	return (width*height)/totalArea;

}
