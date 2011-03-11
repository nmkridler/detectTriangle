#include "kinectdevice.hpp"
#include "constants.hpp"

// Constructor
MyFreenectDevice::MyFreenectDevice(freenect_context *_ctx, int _index)
        : Freenect::FreenectDevice(_ctx, _index),         // Device Constructor
         m_buffer_depth(FREENECT_DEPTH_11BIT_SIZE),      // depth buffer
         m_buffer_rgb(FREENECT_VIDEO_RGB_SIZE),          // RGB buffer
         m_gamma(2048),                                  // gamma
         depthMat(Size(640,480),CV_16UC1),               // Depth matrix 
         rgbMat(Size(640,480),CV_8UC3,Scalar(0)),        // RGB matrix
         ownMat(Size(640,480),CV_8UC3,Scalar(0)),        // own matrix
         m_new_rgb_frame(false),                         // new RGB frame
         m_new_depth_frame(false)                        // new depth frame
{
     // Fill the gamma array
   for( unsigned int i = 0 ; i < 2048 ; i++) 
   {
      const float k1 = 1.1863;
      const float k2 = 2852.5;
      const float k3 = 0.1236;
      const float v = k3*tanf(i/k2 + k1);
      m_gamma[i] = v;
   }

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

// Do not call directly even in child
void MyFreenectDevice::VideoCallback(void* _rgb, uint32_t timestamp) {
    m_rgb_mutex.lock();
    uint8_t* rgb = static_cast<uint8_t*>(_rgb); 
    rgbMat.data = rgb;
    m_new_rgb_frame = true;
    m_rgb_mutex.unlock();
}

// Do not call directly even in child
void MyFreenectDevice::DepthCallback(void* _depth, uint32_t timestamp) {
    m_depth_mutex.lock();
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    depthMat.data = (uchar*) depth;
    m_new_depth_frame = true;
    m_depth_mutex.unlock();
}

bool MyFreenectDevice::getVideo(Mat& output) {
   m_rgb_mutex.lock();
   if(m_new_rgb_frame) {
      cvtColor(rgbMat, output, CV_RGB2BGR);
      m_new_rgb_frame = false;
      m_rgb_mutex.unlock();
      return true;
   } else {
      m_rgb_mutex.unlock();
      return false;
   }
}

void MyFreenectDevice::setOwnMat( void ) {
   m_rgb_mutex.lock();
   cvtColor(rgbMat, ownMat, CV_RGB2BGR);
   m_rgb_mutex.unlock();
}

void MyFreenectDevice::accumOwnMat( void ) {
   Mat tmpMat;
   m_rgb_mutex.lock();
   cvtColor(rgbMat, tmpMat, CV_RGB2BGR);
   m_rgb_mutex.unlock();
   ownMat += tmpMat;
}
bool MyFreenectDevice::getDepth(Mat& output) {
   m_depth_mutex.lock();
   if(m_new_depth_frame) {
      depthMat.copyTo(output);
      m_new_depth_frame = false;
      m_depth_mutex.unlock();
      return true;
   } else {
      m_depth_mutex.unlock();
      return false;
   }
}

//###############################################################
// depthViewColor
//
//   convert depthMat into color
//
//###############################################################
void MyFreenectDevice::depthViewColor(Mat& output) 
{ 
    
   // pointer to output data 
   unsigned char *depth_mid = output.data; 
   for (unsigned int i = 0; i < 640*480 ; i++) { 
      int lb = ((short *)depthMat.data)[i] % 256; 
      int ub = ((short *)depthMat.data)[i] / 256; 
      switch (ub) { 
         case 0: 
            depth_mid[3*i+2] = 255; 
            depth_mid[3*i+1] = 255-lb; 
            depth_mid[3*i+0] = 255-lb; 
            break; 
         case 1: 
            depth_mid[3*i+2] = 255; 
            depth_mid[3*i+1] = lb; 
            depth_mid[3*i+0] = 0; 
            break; 
         case 2: 
            depth_mid[3*i+2] = 255-lb; 
            depth_mid[3*i+1] = 255; 
            depth_mid[3*i+0] = 0; 
            break; 
         case 3: 
            depth_mid[3*i+2] = 0; 
            depth_mid[3*i+1] = 255; 
            depth_mid[3*i+0] = lb; 
            break; 
         case 4: 
            depth_mid[3*i+2] = 0; 
            depth_mid[3*i+1] = 255-lb; 
            depth_mid[3*i+0] = 255; 
            break; 
         case 5: 
            depth_mid[3*i+2] = 0; 
            depth_mid[3*i+1] = 0; 
            depth_mid[3*i+0] = 255-lb; 
            break; 
         default: 
            depth_mid[3*i+2] = 0; 
            depth_mid[3*i+1] = 0; 
            depth_mid[3*i+0] = 0; 
            break; 
      } 
   } 
}
 
//###############################################################
// getBinary
//
//   turn ownMat into a binary image
//
//###############################################################
void MyFreenectDevice::getBinary(Mat& output){
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
void MyFreenectDevice::getContour(){
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
void MyFreenectDevice::processContour( const vector<Point> &contour )
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
         if( !foundMatch ){
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
void MyFreenectDevice::initializeDetection( Detection& newDetection)
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
void MyFreenectDevice::resetDetections()
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
void MyFreenectDevice::reduceDetections()
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
void MyFreenectDevice::outputDetections(Mat& output)
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
bool MyFreenectDevice::validTriangle( const vector<Point> &contour, 
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
void MyFreenectDevice::centerOfMass( const vector<Point> &contour, Point &cMass)
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
float MyFreenectDevice::pixelDepth( Point &vertex )
{
   m_depth_mutex.lock();
   // Get the depth value for this vertex
   uint16_t depIdx = depthMat.at<uint16_t>(vertex);
   m_depth_mutex.unlock();
   return m_gamma[depIdx];
}

//###############################################################
// reduceContour
//
//   reduces the contour to the three vertices furthest from 
//   the center of mass
// 
//###############################################################
void MyFreenectDevice::reduceContour( const vector<Point> &contour, vector<Point> &newTri)
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
void MyFreenectDevice::contourColor( Detection &newDetection)
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
void MyFreenectDevice::filterOrange( Mat& output)
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

void MyFreenectDevice::contourImg()
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

void MyFreenectDevice::getDetectCM( Point &cMass) const
{
   cMass = m_cMass;
}

