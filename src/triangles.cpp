#include <triangles.h>
#include <constants.h>


// Constructor
Triangles::Triangles(Settings const & settings, cv::Size const & frameSize) :
Detection(settings,frameSize)
{
}

void Triangles::processFrame(cv::Mat const & rgb, cv::Mat const & depth)
{
   // Create mats to hold the contours and a mask
   Mat contour, mask;
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
   Mat gray;                                // Initialize some Mats
   Filters::binaryFilter(m_frame,gray);     // Convert rgb data to grayscale
    
   // Contour info
   vector<vector<Point> > contours;
   findContours(gray, contours, 
                CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

   // If contours exist, find the triangles
   if( contours.size() > 0 ){
      // Initialize area and indices
      for( unsigned int idx = 0; idx < contours.size(); ++idx )
      {
         vector<Point> approx;
         vector<Point> approxTriangle;
         approxPolyDP(Mat(contours[idx]), approx, 5, true);
         if( approx.size() > 2 )
         {
            double areaIdx = contourArea(approx);
            if( areaIdx > 0)
            {
               reduceContour( approx, approxTriangle);
         
               // Make sure it's a valid triangle
               if( Stats::validTriangle(approx,approxTriangle)){

                  // Triangle center of mass
				  Point triangleCenter(0,0);
				  Stats::centerOfMass(approxTriangle,triangleCenter);

				  // Create a new detection
            	  Contact newContact;
            	  newContact.position = triangleCenter - cv::Point(50,50);
            	  newContact.dims     = cv::Point(100,100);
                  newContact.score    = 1./contourScore(approxTriangle);
                  newContact.valid    = false;

				  if( m_tracking && overlap(newContact) > 0.6)
				  {
					  newContact.valid = true;
				  }
                  // Add to the list
                  m_list.push_back(newContact);

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
double Triangles::contourScore( vector<Point> &triangle )
{
   // To calculate the score we need the distance
   vector<float> distance;
   for( unsigned int idx = 0; idx < triangle.size(); idx++)
   {
      distance.push_back(pixelDepth(triangle[idx]));
   }
  
   // Get a color score
   float cScore = contourColor( triangle );

   // Create a vector in metric space
   vector<Point3f> xyzTriangle;
   Stats::pixelToMetric(triangle, distance, xyzTriangle);

   // Now we can get a shape score
   float triangleScore = 0;
   if( Stats::shapeScore( xyzTriangle, triangleScore) )
   {
      double score = static_cast<double>(triangleScore + 100.0*cScore)/2.0;
      if( score > 0 ) return 1./score;

   }
   return 1000.;
}

      

//###############################################################
// pixelDepth
//
//   determine the depth in meters
// 
//###############################################################
float Triangles::pixelDepth( Point &vertex )
{
   // Get the depth value for this vertex
   double x = ((static_cast<double>(vertex.x) - 46.)/586.)*640.;
   double y = ((static_cast<double>(vertex.y) - 37.)/436.)*480.;

   vertex.x = std::max(static_cast<int>(x),0);
   vertex.y = std::max(static_cast<int>(y),0);
   if( vertex.x == 0 || vertex.y == 0 ) return 0.0f;

   float depIdx = static_cast<float>(m_depth.at<uint16_t>(vertex));
   return k3*tan(depIdx/k2 + k1);

}

//###############################################################
// reduceContour
//
//   reduces the contour to the three vertices furthest from 
//   the center of mass
// 
//###############################################################
void Triangles::reduceContour( const vector<Point> &contour, vector<Point> &newTri)
{
   // Calculate the moments
   Point centMass(0,0);      
   Stats::centerOfMass( contour, centMass );

   // Get the 3 vertices furthest from the center
   vector<int> cDist;
   vector<unsigned int>::iterator jIdx;
   vector<unsigned int>::iterator kIdx;
   vector<unsigned int> distIdx;
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
float Triangles::contourColor( vector<Point> &contour)
{
   Mat contourMask = Mat::zeros(m_frame.size(), CV_8UC1);
   Scalar color(255,255,255);
 
   // Create a mask of a filled contour
   fillConvexPoly(contourMask, contour.data(), 
                  contour.size(), color);

   // Get the center of mass
   Mat tmpImg;
   cvtColor(m_frame, tmpImg, CV_BGR2HSV);

   Scalar contourMean;
   Scalar contourStd;
   meanStdDev(tmpImg, contourMean, contourStd, contourMask);

   // Set the standard deviation and the mean
   return Stats::colorScore( contourMean, contourStd);

}


