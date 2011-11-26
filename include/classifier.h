#ifndef __CLASSIFIER_H__
#define __CLASSIFIER_H__

#include <iostream>
#include <cv.h>
#include <constants.h>
#include <boost/shared_ptr.hpp>
#include <settings.h>
#include <vector>
#include <fern.h>

class Classifier
{
public:
   // Constructor
   Classifier(int         const & numFerns,
		      int         const & numNodes,
		      cv::Point2f const & scale);

   // Destructor
   ~Classifier(){}

   // Train the classifier with a single training patch
   void train( cv::Mat   const & image,
		       cv::Point       & patchPt,
		       cv::Point       & patchDims,
		       int       const & patchClass);

   // Classify a given patch
   double classify( cv::Mat   const & image,
		            cv::Point       & patchPt,
		            cv::Point       & patchDims);

protected:

   int                   m_numFerns;   // Number of Ferns

   std::vector<FernPtr>  mp_ferns;     // Fern classifiers

};

typedef boost::shared_ptr<Classifier> ClassifierPtr;
#endif
