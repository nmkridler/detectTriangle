#include <classifier.h>

// Constructor
Classifier::Classifier(int         const & numFerns,
	                   int         const & numNodes,
	                   cv::Point2f const & scale) :
m_numFerns(numFerns)
{
	// Add to the list of nodes
	for( int i = 0; i < m_numFerns; ++i)
	{
		mp_ferns.push_back(FernPtr(new Fern(numNodes,scale)));
	}
}


// Train the classifier with a single training patch
void Classifier::train( cv::Mat   const & image,
		                cv::Point       & patchPt,
		                cv::Point       & patchDims,
		                int       const & patchClass)
{
   for(int i = 0; i < m_numFerns; ++i)
   {
	   mp_ferns[i]->train(image,patchPt,patchDims,patchClass);
   }
}

// Classify a given patch
double Classifier::classify( cv::Mat   const & image,
                             cv::Point       & patchPt,
                             cv::Point       & patchDims)
{
	double sum = 0.;
	for( int i = 0; i < m_numFerns; ++i)
	{
		sum += mp_ferns[i]->classify(image,patchPt,patchDims);
	}
	return sum / static_cast<double>(m_numFerns);
}

void Classifier::warpTrainPositive( cv::Mat   const & image,
		                            cv::Point       & patchPt,
		                            cv::Point       & patchDims)
{
   cv::Mat warp;
   // Warp a bunch of times and train
   for( int tIdx = 0; tIdx < 20; ++tIdx)
   {
	   Filters::randWarpROI(image,patchPt,patchDims,warp);
	   train(warp,patchPt,patchDims,1);
   }

}

// Train the classifier with a single training patch
void Classifier::restart()
{
   for(int i = 0; i < m_numFerns; ++i)
   {
	   mp_ferns[i]->restart();
   }
}
