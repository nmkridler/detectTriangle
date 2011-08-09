#include "filters.h"

// Filter the feed to get orange only 
void Filters::filterOrange( cv::Mat& output)
{
   // convert RGB to HSV
   cv::Mat tmpHSV;
   cv::cvtColor(output, tmpHSV, CV_BGR2HSV);

   // Create a mask for the orange color
   cv::Mat orangeImg = cv::Mat::zeros(tmpHSV.size(), CV_8UC1);
   cv::Mat orangeMsk(tmpHSV.size(), CV_8UC1);
   cv::inRange(tmpHSV, Filters::HSV_LOWER,Filters::HSV_UPPER, orangeMsk);

   // dilate(erode)
   cv::morphologyEx(orangeMsk,orangeMsk,cv::MORPH_OPEN,cv::Mat());
   cv::morphologyEx(orangeMsk,orangeMsk,cv::MORPH_CLOSE,cv::Mat());
   orangeImg.setTo(cv::Scalar(255),orangeMsk);

   // Find the non orange pixels and set to 0
   cv::inRange(orangeImg,cv::Scalar(0),cv::Scalar(254),orangeMsk);
   output.setTo(cv::Scalar(0,0,0),orangeMsk);
}
void Filters::binaryFilter(cv::Mat const &input, cv::Mat &output)
{
   cv::cvtColor(input, output, CV_BGR2GRAY);
   cv::morphologyEx(output,output,cv::MORPH_CLOSE,cv::Mat());
   output = output > 200;
}
