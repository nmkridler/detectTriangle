#include <triangles.h>



// Constructor
Triangles::Triangles(Settings const & settings, cv::Size const & frameSize) :
Detection(settings,frameSize)
{
	// Set the target model
	m_target.assign(6,0);
	m_target[0] = 0.5*m_settings.dimensions.x*m_settings.dimensions.y;
	m_target[1] = m_settings.dimensions.x + m_settings.dimensions.y + m_settings.dimensions.z;
	m_target[2] = m_settings.color.x;
	m_target[3] = m_settings.color.y;
	m_target[4] = m_settings.color.z;
	m_target[5] = 90.;

}

void Triangles::processFrame(cv::Mat const & rgb, cv::Mat const & depth)
{
   // Create mats to hold the contours and a mask
   cv::Mat contour, mask;
   rgb.copyTo(m_frame);
   depth.copyTo(m_depth);
   m_list.clear();

   // Create contours
   getContour();
}

//###############################################################
// getContour
//
//   get a list of contours for this frame
//
//###############################################################
void Triangles::getContour(){
   // Create a destination array
   cv::Mat gray,orange;                            // Initialize some Mats
   Filters::filterOrange(m_frame,m_settings.HSVMIN,m_settings.HSVMAX);
   Filters::binaryFilter(m_frame,gray);     // Convert rgb data to grayscale
    
   // Contour info
   std::vector<std::vector<cv::Point> > contours;
   cv::findContours(gray, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

   // If contours exist, find the triangles
   if( contours.size() > 0 ){
      // Initialize area and indices
      for( size_t idx = 0; idx < contours.size(); ++idx )
      {
         std::vector<cv::Point> approx;
         std::vector<cv::Point> approxTriangle;
         cv::approxPolyDP(cv::Mat(contours[idx]), approx, 5, true);
         if( approx.size() > 2 )
         {
            double areaIdx = cv::contourArea(approx);
            if( areaIdx > 0)
            {
               reduceContour( approx, approxTriangle);
         
               // Make sure it's a valid triangle
               if( Stats::validTriangle(approx,approxTriangle)){

                  // Triangle center of mass
				  cv::Point triangleCenter(0,0);
				  Stats::centerOfMass(approxTriangle,triangleCenter);

				  // Create a new detection
            	  Contact newContact;
            	  Feature newFeature(6,0);
            	  newContact.position = triangleCenter - cv::Point(m_boxSize.x/2,m_boxSize.y/2);
            	  newContact.dims     = m_boxSize;
                  newContact.score    = contourScore(approxTriangle,newFeature);
                  newContact.valid    = false;

                  // Add to the list
                  if( newContact.score > 0 ){
                	  m_list.push_back(newContact);
                  }

               }
            } // End if positive area 
         } // End if > 2 sides
      }
   } 
}


//###############################################################
// contourScore
//
//   initialize the detection - this returns a score
// 
//###############################################################
double Triangles::contourScore( std::vector<cv::Point> const & triangle,
		                        Feature                      & features)
{
   // To calculate the score we need the distance
   double depth = pixelDepth(triangle);

   // Depth is outside Kinect operating bounds
   if( depth <= 0.6 || depth >= 10. ) return -1;
   std::vector<double> distance(3,depth);

   // Get a color score
   contourColor( triangle, features);

   // Create a vector in metric space
   std::vector<cv::Point3d> xyzTriangle;
   Stats::pixelToMetric(triangle, distance, xyzTriangle);

   // Now we can get a shape score
   Stats::shape(xyzTriangle,features[1],features[5],features[0]);

   return Stats::similarity(features,m_target);
}

      

//###############################################################
// pixelDepth
//
//   determine the depth in meters
// 
//###############################################################
double Triangles::pixelDepth( std::vector<cv::Point> const & contour )
{

   cv::Mat contourMask = cv::Mat::zeros(m_frame.size(), CV_8UC1);
   cv::Scalar color(255);

   // Create a mask of a filled contour
   cv::fillConvexPoly(contourMask, contour.data(),
                      contour.size(), color);

   // Get the center of mass
   cv::Scalar depthMean;
   cv::Scalar depthStd;
   cv::meanStdDev(m_depth, depthMean, depthStd, contourMask);

   return k3*tan(depthMean[0]/k2 + k1);

}

//###############################################################
// reduceContour
//
//   reduces the contour to the three vertices furthest from 
//   the center of mass
// 
//###############################################################
void Triangles::reduceContour( std::vector<cv::Point> const & contour,
		                       std::vector<cv::Point>       & newTri)
{
   // Calculate the moments
   cv::Point centMass(0,0);
   Stats::centerOfMass( contour, centMass );

   // Get the 3 vertices furthest from the center
   std::vector<int> cDist;
   std::vector<unsigned int>::iterator jIdx;
   std::vector<unsigned int>::iterator kIdx;
   std::vector<unsigned int> distIdx;

   // Compute distance to the center
   for( unsigned int idx = 0; idx < contour.size(); ++idx)
   {
          
      int dist = sqrt( pow(contour[idx].x - centMass.x,2) +
                       pow(contour[idx].y - centMass.y,2) );
      cDist.push_back(dist);
        
      jIdx = distIdx.begin();
      kIdx = distIdx.end();
      bool foundInsert = false;
      while( jIdx < kIdx && !foundInsert )
      {
         if( dist > cDist.at(*jIdx) )
         {
            distIdx.insert(jIdx,idx);
            foundInsert = true;
         }
         ++jIdx;
      }
      if( foundInsert == false ) distIdx.push_back(idx);
       
   }
   newTri.push_back(contour[distIdx[0]]);
   newTri.push_back(contour[distIdx[1]]);
   newTri.push_back(contour[distIdx[2]]);
}

// Color of the contour
void Triangles::contourColor( std::vector<cv::Point> const & contour,
		                      Feature                      & features)
{
   cv::Mat contourMask = cv::Mat::zeros(m_frame.size(), CV_8UC1);
   cv::Scalar color(255,255,255);
 
   // Create a mask of a filled contour
   cv::fillConvexPoly(contourMask, contour.data(),
                      contour.size(), color);

   // Get the center of mass
   cv::Mat tmpImg;
   cv::cvtColor(m_frame, tmpImg, CV_BGR2HSV);

   cv::Scalar contourMean;
   cv::Scalar contourStd;
   cv::meanStdDev(tmpImg, contourMean, contourStd, contourMask);

   features[2] = contourMean[0]; // hue
   features[3] = contourMean[1]; // saturation
   features[4] = contourMean[2]; // value

}

