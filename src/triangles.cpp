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
   m_frame.copyTo(orange);
   Filters::filterOrange(orange,m_settings.HSVMIN,m_settings.HSVMAX);
   Filters::binaryFilter(orange,gray);     // Convert rgb data to grayscale
    
   // Contour info
   std::vector<std::vector<cv::Point> > contours;
   cv::findContours(gray, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

   // If contours exist, find the triangles
   if( contours.size() > 0 ){
      // Initialize area and indices
      for( size_t idx = 0; idx < contours.size(); ++idx )
      {
         std::vector<cv::Point> approx;
         std::vector<cv::Point> cluster;
         std::vector<cv::Point> approxTriangle;
         cv::approxPolyDP(cv::Mat(contours[idx]), approx, 5, true);
         if( approx.size() > 2 )
         {
            double areaIdx = cv::contourArea(approx);
            if( areaIdx > 0)
            {
               clusterContour(approx, cluster);
               if( cluster.size() < 3 ) continue;
               reduceContour(approx,approxTriangle);
               //if( reduceContour( cluster, approxTriangle) ){
         
               // Make sure it's a valid triangle
               if( Stats::validTriangle(approx,approxTriangle)){

                  // Triangle center of mass
                  cv::Point triangleCenter(0,0);
                  Stats::centerOfMass(approx,triangleCenter);

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
                     std::cout << newContact.score << std::endl;
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
   bool invalid = false;
   // Calculate the distance to the object
   std::vector<double> distance;
   for( size_t i = 0; i < 3; ++i)
   {
      double sum = 0.;
      double count = 0.;
      for( int j = -5; j < 6; ++j )
      {
         for( int k = -5; k < 6; ++k )
         {
            double ptDepth = pixelDepth(triangle[i] + cv::Point(j,k) + cv::Point(20,10));
            if( ptDepth > 0 && ptDepth < 1000. )
            {
               sum += ptDepth;
               count++;
            }
         }
      }
      if(count > 0)
      {
         distance.push_back( k3*tanf((sum/count)/k2 + k1));
      } else
      {
         invalid = true;
         distance.push_back(0.);
      }
 
      std::cout << distance.back() << ",";
   }

   Feature model = m_target;
   // Depth is outside Kinect operating bounds
   if( invalid )
   {
      features[0] = 1000.;
      features[1] = 1000.;
      features[5] = 1000.;
      model[0] = 0.;
      model[1] = 0.;
      model[5] = 0.;
   } else
   {

      // Create a vector in metric space
      std::vector<cv::Point3d> xyzTriangle;
      Stats::pixelToMetric(triangle, distance, xyzTriangle);

      // Now we can get a shape score
      Stats::shape(xyzTriangle,features[1],features[5],features[0]);
      if( fabs(features[5] - 90.) > 5.0 ) return -1;
      if( features[0] < 0.01 ) return -1;
   }
   // Get a color score
   contourColor( triangle, features);

   return Stats::similarity(features,model);
}

      

//###############################################################
// pixelDepth
//
//   determine the depth in meters
// 
//###############################################################
double Triangles::averageDepth( std::vector<cv::Point> const & contour )
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
  
   return k3*tanf(depthMean[0]/k2 + k1);

}
double Triangles::pixelDepth( cv::Point const & point)
{
   if( point.y >= 480 || point.y < 0 || point.x < 0 || point.x >= 640 ) return 0.;
   double depthIdx = static_cast<double>(m_depth.at<uint16_t>(point.y,point.x));
   return depthIdx;
   //return k3*tanf(depthIdx/k2 + k1);
}
void Triangles::clusterContour( std::vector<cv::Point> const & contour,
	                        std::vector<cv::Point>       & newTri)
{
   // create some lists
   std::list<cv::Point> cList;
   for( size_t k = 0; k < contour.size(); ++k )
      cList.push_back(contour[k]);
 
   // Loop through the contour
   std::list<cv::Point>::iterator iter = cList.begin();
   while( iter != cList.end()  )
   {
      int count = 0;
      int x = iter->x;
      int y = iter->y;
      std::list<cv::Point>::iterator j = iter;
      ++j;
      // Calculate the distance between all of the points
      while( j != cList.end() )
      {
         cv::Point diff = cv::Point( abs( iter->x - j->x),
                                     abs( iter->y - j->y) );
         if( diff.x < 10 && diff.y < 10 )
         { 
            ++count;
            x += j->x;
            y += j->y;
            // Delete this entry
            j = cList.erase(j);
         } else ++j;
         
      }
      if( count > 0 )
      {
         newTri.push_back(cv::Point(x/count,y/count));
         iter = cList.erase(iter);
      } else
      {
         newTri.push_back(*iter);
         ++iter;
      } 
      
   }
#if 0
   if( newTri.size() > 2 )
   {
      cv::Mat contourMask = cv::Mat::zeros(m_frame.size(), CV_8UC1);
      cv::Scalar color(255);
      // Create a mask of a filled contour
      cv::fillConvexPoly(contourMask, newTri.data(),
                         newTri.size(), color);
      cv::namedWindow("test",1);
      cv::flip(contourMask,contourMask,0);
      cv::imshow("test",contourMask);
   
      cv::Mat test;
      m_depth.copyTo(test);
      cv::namedWindow("blah",1);
      cv::flip(test,test,0);
      double minVal = 0;
      double maxVal = 0;
      cv::minMaxLoc(test,&minVal,&maxVal);
      double scale = 255.0/(maxVal - minVal);
      double shift = -minVal*scale;
      cv::Mat tmp;
      cv::convertScaleAbs(test,tmp,scale,shift);
      cv::imshow("blah",tmp);
      std::cout << newTri.size() << std::endl;
      cv::waitKey(0);
   }
#endif
}
bool Triangles::reduceContour( std::vector<cv::Point> const & contour,
	                       std::vector<cv::Point>       & newTri)
{
   // Calculate the moments
   cv::Point centMass(0,0), newCM(0,0);
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
   Stats::centerOfMass( newTri, newCM );
   // Calculate the change in center of mass
   cv::Point diff( abs(centMass.x - newCM.x), abs(centMass.y - newCM.y));
   if( diff.x > 5 || diff.y > 5 ) return false;

#if 0
   // Create a mask of a filled contour
   cv::fillConvexPoly(contourMask, newTri.data(),
                      newTri.size(), color);
   cv::namedWindow("test",1);
   cv::imshow("test",contourMask);
   cv::waitKey(0);
#endif
   return true;
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
#if 0
   cv::namedWindow("test",1);
   cv::flip(contourMask,contourMask,0);
   cv::imshow("test",contourMask);
   cv::waitKey(0);
#endif
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

