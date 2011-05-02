#ifndef __FILTERS_H__
#define __FILTERS_H__

#include <cv.h>
#include <highgui.h>
#include "constants.h"

using namespace cv;

namespace Filters
{
   // Equalize the image
   void equalizeRGB(Mat &output);   

   // Filter for orange
   void filterOrange(Mat &output);

}
#endif
