#include <fern.h>

// Constructor
Fern::Fern(int         const & nodeNum,
                       cv::Point2f const & scale) :
m_numNodes(nodeNum)
{
    // Create the number of leafNodes
	int leafNodes = static_cast<int>(pow(4.0f,m_numNodes));

	m_positive.assign(leafNodes,0);
	m_negative.assign(leafNodes,0);
	m_posteriors.assign(leafNodes,0.);

	// Add to the list of nodes
	for( int i = 0; i < m_numNodes; ++i)
	{
		mp_twoBitBP.push_back(TwoBitBPPtr(new TwoBitBP(scale)));
	}
}


// Train the classifier with a single training patch
void Fern::train( cv::Mat   const & image,
		          cv::Point       & patchPt,
		          cv::Point       & patchDims,
		          int       const & patchClass)
{
	// Determine the leaf this patch belongs to
	int leaf = getLeafIndex(image,patchPt,patchDims);

	// Increment the appropriate leaf
	if( patchClass == 0)
	{
		m_negative[leaf]++;
	}
	else
	{
		m_positive[leaf]++;
	}

	// Compute the posterior likelihood of a positive class
	if( m_positive[leaf] > 0)
	{
		m_posteriors[leaf] = static_cast<double>(m_positive[leaf]) /
				             static_cast<double>(m_positive[leaf] + m_negative[leaf]);
	}
}

// Classify a given patch
double Fern::classify( cv::Mat   const & image,
                       cv::Point       & patchPt,
                       cv::Point       & patchDims)
{
   return m_posteriors[getLeafIndex(image,patchPt,patchDims)];
}

// Process a frame of data
int Fern::getLeafIndex(cv::Mat    const & image,
					   cv::Point        & patchPt,
					   cv::Point        & patchDims )
{
	// Make sure the patch sizes don't exceed the image size
	cv::Size imgSize = image.size() - cv::Size(1,1);
	patchPt.x = std::max(std::min(patchPt.x,imgSize.width-2),0);
	patchPt.y = std::max(std::min(patchPt.y,imgSize.height-2),0);

	patchDims.x = std::min(patchDims.x,imgSize.width - patchPt.x);
	patchDims.y = std::min(patchDims.y,imgSize.height - patchPt.y);

	// Apply all tests to find the leaf index this patch falls into
	int leaf = 0;
	for( int i = 0; i < m_numNodes; ++i)
	{
		leaf = leaf | (mp_twoBitBP[i]->test(image,patchPt,patchDims) << i*2);
	}
	return leaf;
}

void Fern::restart()
{
	m_posteriors.assign(m_leafNodes,0.);
	m_positive.assign(m_leafNodes,0);
	m_negative.assign(m_leafNodes,0);
}
