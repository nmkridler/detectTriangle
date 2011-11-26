#ifndef __FERN_H__
#define __FERN_H__

#include <iostream>
#include <cv.h>
#include <constants.h>
#include <boost/shared_ptr.hpp>
#include <settings.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <twobitbp.h>
#include <algorithm>
#include <math.h>

class Fern
{
public:
   // Constructor
   Fern(int         const & nodeNum,
        cv::Point2f const & scale);

   // Destructor
   ~Fern(){}

   // Train the classifier with a single training patch
   void train( cv::Mat   const & image,
		       cv::Point       & patchPt,
		       cv::Point       & patchDims,
		       int       const & patchClass);

   // Classify a given patch
   double classify( cv::Mat   const & image,
		           cv::Point       & patchPt,
		           cv::Point       & patchDims);

   // Process a frame of data
   int getLeafIndex(cv::Mat    const & image,
		            cv::Point        & patchPt,
		            cv::Point        & patchDims );


protected:

   int                       m_numNodes;      // Number of nodes
   std::vector<double>       m_posteriors;    // Precomputed posterior likelihoods
   std::vector<int>          m_positive;      // Number of positive patches in each node
   std::vector<int>          m_negative;      // Number of negative patches in each node

   std::vector<TwoBitBPPtr>  mp_twoBitBP;     // Feature nodes

};

typedef boost::shared_ptr<Fern> FernPtr;
#endif
