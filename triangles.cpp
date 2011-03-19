#include "triangles.h"
#include "constants.h"

// Constructor
Triangles::Triangles(freenect_context *_ctx, int _index)
        : MyFreenectDevice(_ctx, _index)         // Device Constructor
{

   // Create a triangle
   Point a0(298, 98), a1(344,102), a2(341,125);
           
   // First triangle
   vector<Point> initTriangle; 
   initTriangle.push_back(a0);
   initTriangle.push_back(a1);
   initTriangle.push_back(a2);
   Detection firstDetection(initTriangle);
   m_triangle.push_back(initTriangle);


}

//###############################################################
// getBinary
//
//   turn ownMat into a binary image
//
//###############################################################
void Triangles::getBinary(Mat& output){
   Mat gray;
   cvtColor(ownMat, gray, CV_BGR2GRAY);
   GaussianBlur(gray, gray, Size(11,11), 1.5, 1.5);
   Canny(gray, gray, 0, 30, 3);
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
      unsigned int idx = 0;
      while( idx < contours.size() ){
         vector<Point> approx;
         approxPolyDP(Mat(contours[idx]), approx, 5, true);
         double areaIdx = contourArea(approx);
         // Create a mask
         if( areaIdx > TARGET_PIXEL_THRESH && approx.size() == 3 ){
            processContour(approx);
         }
         idx++;
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
void Triangles::processContour( const vector<Point> &contour )
{
   // Approximate the object with a triangle
   vector<Point> approxTriangle;
   //reduceContour( contour, approxTriangle );
   approxTriangle = contour;
   Point contourCenter(0,0);
   Point triangleCenter(0,0);
   centerOfMass(contour,contourCenter);
   centerOfMass(approxTriangle,triangleCenter);
   if( validTriangle(contour, approxTriangle) )
   {
      Detection newDetection(approxTriangle);
      if (m_triangle.size() == 0){
         newDetection.addHit();
         initializeDetection(newDetection);
         m_triangle.push_back(newDetection);
      }
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
            if( dist < TARGET_RELATED_DIST ){
               foundMatch = true;
               newDetection.addHit();
               initializeDetection(newDetection);
               m_triangle[dIdx] = newDetection;
                
            }
            ++dIdx;
         } // end loop over detections
         if(  !foundMatch ){
            newDetection.addHit();
            initializeDetection(newDetection);
            m_triangle.push_back(newDetection);
            
         }
      } // End check for existing
   } // End check for valid
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
   centerOfMass(triangle,triangleCenter);
   newDetection.setCentMass(triangleCenter);

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
//   create a mask with all of the detections
// 
//###############################################################
void Triangles::outputDetections(Mat& output)
{
   Mat dst = Mat::zeros(output.size(), CV_8UC1);
   Scalar color(255);

   float minScore = 0;
   unsigned int minIdx = 0;
   // Loop over the detections
   for( unsigned int dIdx = 0; dIdx < m_triangle.size(); ++dIdx)
   {
       //if( dIdx == 0 ){
       //   m_triangle[dIdx].getCentMass(m_cMass);
       //   cout << "Area: " << m_triangle[dIdx].getArea() << endl;
       //}
       //vector<Point> newObject;
       //m_triangle[dIdx].getVertices(newObject);
       //fillConvexPoly(dst, newObject.data(), 
       //               newObject.size(), color);
       float score = m_triangle[dIdx].getScore();
       if( dIdx == 0 ) minScore = score;
      
       if( score < minScore ){
          minScore = score;
          minIdx   = dIdx;
       }
            
   }
   m_triangle[minIdx].getCentMass(m_cMass);
   dst.copyTo(output); // Copy to output
     
}
      
//###############################################################
// validTriangle
//
//   check all the vertices to see if they are within the triangle
// 
//###############################################################
bool Triangles::validTriangle( const vector<Point> &contour, 
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
void Triangles::centerOfMass( const vector<Point> &contour, Point &cMass)
{
   // Calculate the moments       
   Moments cMoments = moments(Mat(contour));
   cMass.x = (int)(cMoments.m10/cMoments.m00);
   cMass.y = (int)(cMoments.m01/cMoments.m00);
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
   centerOfMass( contour, centMass );

   // Get the 3 vertices furthest from the center
   vector<int> cDist;
   vector<unsigned int>:: iterator jIdx;
   vector<unsigned int>:: iterator kIdx;
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
   Mat contourMask = Mat::zeros(ownMat.size(), CV_8UC1);
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
   newDetection.setMean( contourMean );
   newDetection.setStdDev( contourStd );

}

// Filter the feed to get orange only 
void Triangles::filterOrange( Mat& output)
{

  // convert to HSV
   Mat tmpImg;
   cvtColor(ownMat, tmpImg, CV_BGR2HSV);

   // Create a mask for the orange color
   Mat orangeImg = Mat::zeros(tmpImg.size(), CV_8UC1);
   Mat orangeMsk(ownMat.size(), CV_8UC1);
   inRange(tmpImg, HSV_LOWER, HSV_UPPER, orangeMsk);

   // Dilate the mask and set to 255
   dilate(orangeMsk,orangeMsk,Mat());
   orangeImg.setTo(Scalar(255),orangeMsk);

   // Find the non orange pixels and set to 0
   inRange(orangeImg,Scalar(0),Scalar(254),orangeMsk);
   erode(orangeMsk,orangeMsk,Mat());
   dilate(orangeMsk,orangeMsk,Mat());

   ownMat.setTo(Scalar(0,0,0),orangeMsk);
   
   ownMat.copyTo(output);
}

void Triangles::contourImg()
{
   // Create mats to hold the contours and a mask
   Mat contour, mask;

   // Create contours
   getContour();

   // Create the contour image
   cvtColor(ownMat,contour,CV_BGR2GRAY);
   resetDetections();
   reduceDetections();
   outputDetections(contour);
 
}

void Triangles::getDetectCM( Point &cMass) const
{
   cMass = m_cMass;
}

bool Triangles::foundTarget() 
{
   if( m_triangle.size() > 0 ) m_foundTarget = true;
   return m_foundTarget;
}

