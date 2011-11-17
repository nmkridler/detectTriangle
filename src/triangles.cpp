#include <triangles.h>
#include <constants.h>

// Sorting function
bool rankDetection( Contact &first, Contact &second)
{
   if( first.score > second.score )
   {
      return false;
   } else return true;
}   

// Constructor
Triangles::Triangles(int const & maxDet) :
Detection(maxDet)
{
}

void Triangles::processFrame(cv::Mat const & rgb, cv::Mat const & depth)
{
   // Create mats to hold the contours and a mask
   Mat contour, mask;
   rgb.copyTo(m_frame);
   depth.copyTo(m_depth);

   // Create contours
   getContour();

   // Create the contour image
   if( m_list.size() > 0)
   {
      resetDetections();
      reduceDetections();
      outputDetections();

   }

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
                  // Get the detection score
                  float cScore = contourScore(approxTriangle);
                  // If it's a valid triangle it's a detection
                  if( cScore < TARGET_SCORE_THRESHOLD && cScore > 0)
                  { 
               
                     // Triangle center of mass
                     Point triangleCenter(0,0);
                     Stats::centerOfMass(approxTriangle,triangleCenter);
                     Contact newContact;
                     newContact.position = triangleCenter;
                     newContact.score = cScore;
                     processDetection(newContact);
                  }
               }
            } // End if positive area 
         } // End if > 2 sides
      }
   } 
}

//###############################################################
// processContour
//
//   determine if the contour is a detection
//   add to detection list
//
//###############################################################
void Triangles::processDetection( Contact &newDetection )
{
   // Get the center of mass
   Point triangleCenter = newDetection.position;
   
   // Check the size of the track list
   if (m_list.size() == 0){
      m_list.push_back(newDetection);
   }  // If we haven't seen it before at it to the list
   else{
      // Loop through the detections
      // determine the distance to the center of mass
      bool foundMatch = false;
      ContactList::iterator tracks = m_list.begin();
      while( tracks != m_list.end() && !foundMatch)
      {
         
         // get the center of mass
         Point detectMass = tracks->position;
         // Calculate the distance
         int dist = sqrt( pow(detectMass.x - triangleCenter.x,2) +
                          pow(detectMass.y - triangleCenter.y,2));
          
         // Take everything within 10 pixels to be the same
         if( dist < TARGET_RELATED_DIST )
         {
            tracks->misses=0;
            foundMatch = true;
         }
  
         tracks++;
      } // end loop over detections
      if(  !foundMatch ){
         m_list.push_back(newDetection);
      }
   } // End check for existing
   // Make sure we don't have too many detections
   while( m_list.size() > 10 )
   {
      m_list.pop_back();
   }
}

//###############################################################
// contourScore
//
//   initialize the detection - this returns a score
// 
//###############################################################
float Triangles::contourScore( vector<Point> &triangle )
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
      return (triangleScore + 100.0*cScore)/2.0;
   } else return -1;
}


//###############################################################
// resetDetections
//
//   set everything to missed
// 
//###############################################################
void Triangles::resetDetections()
{
   ContactList::iterator tracks;
   for( tracks = m_list.begin(); tracks != m_list.end(); ++tracks)
   {
      tracks->misses++;
   }
}

//###############################################################
// reduceDetections
//
//   cut out missed detections
// 
//###############################################################
void Triangles::reduceDetections()
{
   ContactList::iterator tracks = m_list.begin();
   while( tracks != m_list.end() )
   {
     // Check the miss Count
     if( tracks->misses > MAX_MISS_THRESH )
     {
         tracks = m_list.erase(tracks);
     } else ++tracks;
   } 
}    

//###############################################################
// outputDetections
//
//   create a list of the centers
// 
//###############################################################
void Triangles::outputDetections()
{
   // Sort the tracks
   m_list.sort(rankDetection);

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
   Mat flipRGB;
   Mat tmpImg;
   flip(m_frame, flipRGB,0);
   cvtColor(flipRGB, tmpImg, CV_RGB2HSV);
    
   Scalar contourMean;
   Scalar contourStd;
   meanStdDev(tmpImg, contourMean, contourStd, contourMask);

   // Set the standard deviation and the mean
   return Stats::colorScore( contourMean, contourStd);
}


