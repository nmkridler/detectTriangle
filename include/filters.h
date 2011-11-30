#ifndef __FILTERS_H__
#define __FILTERS_H__

#include <cv.h>
#include <highgui.h>
#include <constants.h>
#include <vector>
namespace Filters
{
   static const cv::Scalar   HSV_LOWER(-1,100,100);
   static const cv::Scalar   HSV_UPPER(15,256,256);

   // Filter for orange
   void filterOrange(cv::Mat &output, cv::Scalar const & hsvmin, cv::Scalar const & hsvmax);

   // Binary filter
   void edgeDetection(cv::Mat const &input, cv::Mat &output);

   // Binary filter
   void binaryFilter(cv::Mat const &input, cv::Mat &output);

   void randWarpROI(cv::Mat    const & input,
   		            cv::Point  const & corner,
   		            cv::Point  const & size,
   		            cv::Mat & output );

   double randHalf();
}
#endif
