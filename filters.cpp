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
   outMat.clear();
   outMat.push_back(bOut); 
   outMat.push_back(gOut); 
   outMat.push_back(rOut); 
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

   // dilate(erode)
   morphologyEx(orangeMsk,orangeMsk,MORPH_OPEN,Mat());
   morphologyEx(orangeMsk,orangeMsk,MORPH_CLOSE,Mat());
   orangeImg.setTo(Scalar(255),orangeMsk);

   // Find the non orange pixels and set to 0
   inRange(orangeImg,Scalar(0),Scalar(254),orangeMsk);
   output.setTo(Scalar(0,0,0),orangeMsk);
}
void Filters::binaryFilter(Mat const &input, Mat &output)
{
   cvtColor(input, output, CV_BGR2GRAY);
   morphologyEx(output,output,MORPH_CLOSE,Mat());
   output = output > 200;
}
