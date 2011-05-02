#include "filters.h"

using namespace cv;

void Filters::equalizeRGB(Mat &output)
{
   // Initialize the channels
   Mat rMat(output.size(),CV_8UC1);
   Mat gMat(output.size(),CV_8UC1);
   Mat bMat(output.size(),CV_8UC1);
   Mat rOut(output.size(),CV_8UC1);
   Mat gOut(output.size(),CV_8UC1);
   Mat bOut(output.size(),CV_8UC1);

   // Make a vector and split
   vector<Mat> outMat;
   outMat.push_back(bMat); 
   outMat.push_back(gMat); 
   outMat.push_back(rMat); 
   split(output, outMat );

   // Equalize and copy to output
   equalizeHist(rMat,rOut);
   equalizeHist(gMat,gOut);
   equalizeHist(bMat,bOut);
   merge(outMat,output);
}

// Filter the feed to get orange only 
void Filters::filterOrange( Mat& output)
{
    
   // convert RGB to HSV
   Mat tmpHSV;
   cvtColor(output, tmpHSV, CV_BGR2HSV);

   // Create a mask for the orange color
   Mat orangeImg = Mat::zeros(tmpHSV.size(), CV_8UC1);
   Mat orangeMsk(tmpHSV.size(), CV_8UC1);
   inRange(tmpHSV, KinectConstants::HSV_LOWER, 
           KinectConstants::HSV_UPPER, orangeMsk);

   // Dilate the mask and set to 255
   erode(orangeMsk,orangeMsk,Mat(),Point(-1,-1),5);
   orangeImg.setTo(Scalar(255),orangeMsk);

   // Find the non orange pixels and set to 0
   inRange(orangeImg,Scalar(0),Scalar(254),orangeMsk);

   erode(orangeMsk,orangeMsk,Mat(),Point(-1,-1),30);
   output.setTo(Scalar(255,255,255),orangeMsk);
   //GaussianBlur(output, output, Size(11,11), 5.5, 5.5);
}

