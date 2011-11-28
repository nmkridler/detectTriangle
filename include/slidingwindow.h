#ifndef __SLIDINGWINDOW_H__
#define __SLIDINGWINDOW_H__

#include <iostream>
#include <cv.h>
#include <constants.h>
#include <boost/shared_ptr.hpp>
#include <settings.h>
#include <vector>
#include <detection.h>
#include <classifier.h>

class SlidingWindow : public Detection
{
public:
   // Constructor
   SlidingWindow(Settings const & settings,
		         cv::Size const & frameSize);

   // Destructor
   ~SlidingWindow(){}

   // Process a frame of data
   void processFrame(cv::Mat const & rgb, cv::Mat const & depth);

   void generateBoundingBoxes(cv::Mat const & integral);


};

typedef boost::shared_ptr<SlidingWindow> SlidingWindowPtr;
#endif
