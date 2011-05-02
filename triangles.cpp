#include "triangles.h"
#include "constants.h"

// Constructor
Triangles::Triangles(freenect_context *_ctx, int _index)
        : MyFreenectDevice(_ctx, _index)         // Device Constructor
{
}

//###############################################################
// getBinary
//
//   turn ownMat into a binary image
//
//###############################################################
void Triangles::getBinary(Mat& output){
   Mat gray;
   // At this point ownMat should be the Sobel Filtered image
   cvtColor(ownMat, gray, CV_BGR2GRAY);
   //GaussianBlur(gray, gray, Size(11,11), 1.5, 1.5);
   //Canny(gray, gray, 0, 30, 3);
   dilate(gray, output, Mat());
   output = output > 128;
} 

//###############################################################
// getContour
//
//   get a list of contours for this frame
//
//###############################################################
void Triangles::getContour(){
   // Create a destination array
   Mat gray;    // Initialize some Mats
   getBinary(gray);     // Convert rgb data to grayscale
    
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
         double areaIdx = contourArea(approx);
         if( approx.size() > 2 && areaIdx > 0)
         {
            reduceContour( approx, approxTriangle);
         
            // Create a mask
            if( Stats::validTriangle(approx,approxTriangle)){
               processContour(approxTriangle);
            }
         }
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
void Triangles::processContour( vector<Point> &approxTriangle )
{
   // Triangle center of mass
   Point triangleCenter(0,0);
   Stats::centerOfMass(approxTriangle,triangleCenter);

   // Make a new detection object
   Detection newDetection(approxTriangle);
   if (m_triangle.size() == 0){
      initializeDetection(newDetection);
      m_triangle.push_back(newDetection);
      uint64_t lastIdx = m_triangle.size();
      m_triangle[lastIdx-1].addHit(); 
   }  // If we haven't seen it before at it to the list
   else{
      // Loop through the detections
      // determine the distance to the center of mass
      unsigned int dIdx = 0;
      bool foundMatch = false;
      while( dIdx < m_triangle.size() && !foundMatch)
      {
         // get the center of mass
         Point detectMass(0,0);
         m_triangle[dIdx].getCentMass(detectMass);
         // Calculate the distance
         int dist = sqrt( pow(detectMass.x - triangleCenter.x,2) +
                          pow(detectMass.y - triangleCenter.y,2));
          
         // Take everything within 10 pixels to be the same
         if( dist < TARGET_RELATED_DIST )
         {
            foundMatch = true;
            initializeDetection(m_triangle[dIdx]);
            m_triangle[dIdx].addHit();
                
         }
         ++dIdx;
      } // end loop over detections
      if(  !foundMatch ){
         initializeDetection(newDetection);
         newDetection.addHit();
         m_triangle.push_back(newDetection);
         uint64_t lastIdx = m_triangle.size();
         m_triangle[lastIdx-1].addHit(); 
          
      }
   } // End check for existing
   // Make sure we don't have too many detections
   while( m_triangle.size() > MAX_DETECTIONS )
   {
      m_triangle.pop_back();
   }
}

//###############################################################
// initializeDetection
//
//   initialize the detection
// 
//###############################################################
void Triangles::initializeDetection( Detection& newDetection)
{
   // Set the center of mass
   Point triangleCenter(0,0);
   vector<Point> triangle;
   newDetection.getVertices(triangle);

   // Initialize with valid and zero misses
   newDetection.setValid(true);
   newDetection.resetMissCount();

   // Create a vector of floats to hold the distances
   vector<float> distance;
   for( unsigned int idx = 0; idx < triangle.size(); idx++)
   {
      distance.push_back(pixelDepth(triangle[idx]));
   }
   newDetection.setDistance(distance);

   // Set the mean and standard deviation of the color
   contourColor(newDetection);
}
//###############################################################
// resetDetections
//
//   set everything to missed
// 
//###############################################################
void Triangles::resetDetections()
{
   for( unsigned int idx = 0; idx < m_triangle.size(); ++idx)
   {
      m_triangle[idx].addMiss();
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
   vector<Detection> newList;
   for( unsigned int idx = 0; idx < m_triangle.size(); ++idx)
   {
     // Check the hit Count
     if( m_triangle[idx].getMissCount() > MAX_MISS_THRESH )
         m_triangle[idx].setValid(false);

     if( !m_triangle[idx].isRightTriangle())
        m_triangle[idx].setValid(false);
              
     // Add only if it's a valid detection
     if( m_triangle[idx].validDetection())
         newList.push_back(m_triangle[idx]);
   } 
   m_triangle.clear();
   m_triangle = newList;
}    

//###############################################################
// outputDetections
//
//   create a list of the centers
// 
//###############################################################
void Triangles::outputDetections()
{

   // Loop over the detections
   m_cMass.clear();
   for( unsigned int dIdx = 0; dIdx < m_triangle.size(); ++dIdx)
   {
      Point cMass;
      m_triangle[dIdx].getCentMass(cMass);
      m_cMass.push_back(cMass);

   }
     
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
   uint16_t depIdx = depthMat.at<uint16_t>(vertex);
   return m_gamma[depIdx];
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
void Triangles::contourColor( Detection &newDetection)
{
   vector<Point> contour;
   newDetection.getVertices(contour);
   Mat contourMask = Mat::zeros(rgbMat.size(), CV_8UC1);
   Scalar color(255,255,255);

   // Create a mask of a filled contour
   fillConvexPoly(contourMask, contour.data(), 
                  contour.size(), color);

   Scalar contourMean;
   Scalar contourStd;
   Mat tmpImg;
   cvtColor(rgbMat, tmpImg, CV_RGB2HSV);
   meanStdDev(tmpImg, contourMean, contourStd, contourMask);

   //cout << "Mean: ";
   //cout << contourMean[0] << " " << contourMean[1] << " " << contourMean[2];
   //cout << endl;
   //cout << "Sig: ";
   //cout << contourStd[0] << " " << contourStd[1] << " " << contourStd[2];
   //cout << endl;
   // Set the standard deviation and the mean
   float cScore = Stats::colorScore( contourMean, contourStd);
   newDetection.setColorScore( cScore );

}


void Triangles::contourImg()
{
   // Create mats to hold the contours and a mask
   Mat contour, mask;

   // Create contours
   getContour();

   // Create the contour image
   if( m_triangle.size() > 0){

      resetDetections();
      reduceDetections();
      outputDetections();

   }
   else m_foundTarget = false;
}

void Triangles::getDetectCM( vector<Point> &cMass) const
{
   cMass = m_cMass;
}

bool Triangles::foundTarget() 
{
   if( m_triangle.size() > 0 ) m_foundTarget = true;
   return m_foundTarget;
}

