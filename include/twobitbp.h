#ifndef __TWOBITBP_H__
#define __TWOBITBP_H__

#include <iostream>
#include <cv.h>
#include <constants.h>
#include <boost/shared_ptr.hpp>
#include <settings.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

class TwoBitBP
{
public:
   // Constructor
   TwoBitBP(cv::Point2f const & scale);

   // Destructor
   ~TwoBitBP(){}

   // Calculate the sum over a patch
   float pixelSum(cv::Mat     const & image,
		          cv::Point2f const & pos,
		          cv::Point2f const & size);

   // Train the classifier with a single training patch
   int test(cv::Mat   const & image,
		    cv::Point const & patchPt,
		    cv::Point const & patchDims);

protected:

   cv::Point2f  m_position;  // Percentage position (x,y)
   cv::Point2f  m_scale;     // Percentage scale (width,height)

};

typedef boost::shared_ptr<TwoBitBP> TwoBitBPPtr;
#endif
