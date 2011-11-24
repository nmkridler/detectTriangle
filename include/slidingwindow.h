#ifndef __SLIDINGWINDOW_H__
#define __SLIDINGWINDOW_H__

#include <iostream>
#include <cv.h>
#include <constants.h>
#include <boost/shared_ptr.hpp>
#include <settings.h>
#include <vector>
#include <detection.h>


class SlidingWindow : public Detection
{
public:
   // Constructor
   SlidingWindow(Settings const & settings,
		         cv::Size const & frameSize);

   // Destructor
   ~SlidingWindow(){}

   void initialize(cv::Vec4d const & boundingBox );

   // Process a frame of data
   void processFrame(cv::Mat const & rgb, cv::Mat const & depth);

   double overlap( cv::Vec4d const & boxA,
		           cv::Vec4d const & boxB);

private:
   cv::Size m_size;

   cv::Mat  m_gray;
   cv::Mat  m_integral;
};

typedef boost::shared_ptr<SlidingWindow> SlidingWindowPtr;
#endif
