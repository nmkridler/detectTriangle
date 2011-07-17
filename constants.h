#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include <cv.h>

using namespace cv;
//
// The constants used to convert from pixel,depth to XYZ
// can be found at
// nicolas.burrus.name/index.php/Research/KinectCalibration
//
namespace KinectConstants
{
   const float  fPi    =  3.14159265;

   // Constants for RGB conversion
   const double fx_rgb =  5.2921508098293293e+02;
   const double fy_rgb =  5.2556393630057437e+02;
   const double cx_rgb =  3.2894272028759258e+02;
   const double cy_rgb =  2.6748068171871557e+02;
   const double k1_rgb =  2.6451622333009589e-01;
   const double k2_rgb = -8.3990749424620825e-01;
   const double p1_rgb = -1.9922302173693159e-03;
   const double p2_rgb =  1.4371995932897616e-03;
   const double k3_rgb =  9.1192465078713847e-01;

   // Constants for depth conversion
   const double fx_d   =  5.9421434211923247e+02;
   const double fy_d   =  5.9104053696870778e+02;
   const double cx_d   =  3.3930780975300314e+02;
   const double cy_d   =  2.4273913761751615e+02;
   const double k1_d   = -2.6386489753128833e-01;
   const double k2_d   =  9.9966832163729757e-01;
   const double p1_d   = -7.6275862143610667e-04;
   const double p2_d   =  5.0350940090814270e-03;
   const double k3_d   = -1.3053628089976321e+00;

   // Thresholds
   const double       TARGET_PIXEL_THRESH = 400.0;
   const double       TARGET_AREA_METERS  = 0.033;
   const double       TARGET_PERIM        = 0.89;
   const int          TARGET_RELATED_DIST = 10;
   const unsigned int MAX_DETECTIONS      = 100;
   const unsigned int MAX_MISS_THRESH     = 10;
   const Scalar       HSV_LOWER(-1,100,100);
   const Scalar       HSV_UPPER(15,256,256);
   const Scalar       TARGET_COLOR(8,190,185);
   const float        TARGET_SCORE_THRESHOLD = 15.0;
}

#endif
