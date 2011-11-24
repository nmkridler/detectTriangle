#include <twobitbp.h>
float TwoBitBP::pixelSum(cv::Mat     const & image,
		                 cv::Point2f const & pos,
		                 cv::Point2f const & size)
{
	// The input image should be an integral image so that
	// it's easy to calculate the summed area of four points
	// sum = I(x,y) + I(x+w,y+h) - I(x+w,y) - I(x,y+h)
	return image.at<float>(pos.x,pos.y) +
		   image.at<float>(pos.x + size.x, pos.y + size.y) -
		   image.at<float>(pos.x + size.x, pos.y) -
		   image.at<float>(pos.x, pos.y + size.y);
}

TwoBitBP::TwoBitBP( cv::Point2f const & scale )
{
	// Generate randome scales between minScale and maxScale
	// This assumes the scale variable is ordered as
	// scale.x = minScale
	// scale.y = maxScale
	m_scale.x = (scale.y - scale.x)*static_cast<float>(rand()) /
			    static_cast<float>(RAND_MAX) + scale.x;
	m_scale.y = (scale.y - scale.x)*static_cast<float>(rand()) /
			    static_cast<float>(RAND_MAX) + scale.x;

	// Generate random position between 0 and width/heigh
	m_position.x = (1.0f - m_scale.x)*static_cast<float>(rand()) /
			       static_cast<float>(RAND_MAX);
	m_position.y = (1.0f - m_scale.y)*static_cast<float>(rand()) /
			       static_cast<float>(RAND_MAX);

}

int TwoBitBP::test(cv::Mat   const & image,
		           cv::Point const & patchPt,
		           cv::Point const & patchDims)
{
	// Cast the input dimensions as floats
	cv::Point2f floatDims;
	floatDims.x = static_cast<float>(patchDims.x);
	floatDims.y = static_cast<float>(patchDims.y);

	// Compute the properties of the test rectangles relative to the
	// size and position of the patch
	cv::Point pos;
	cv::Point scale;
	pos.x   = static_cast<int>(m_position.x*floatDims.x) + patchPt.x;
	pos.y   = static_cast<int>(m_position.y*floatDims.y) + patchPt.y;
	scale.x = static_cast<int>(m_scale.x*floatDims.x*0.5f);
	scale.y = static_cast<int>(m_scale.y*floatDims.y*0.5f);

	// Double the height
	scale.y *= 2;
	float left     = pixelSum(image,pos,scale);
	float right    = pixelSum(image,pos + cv::Point(scale.x,0),scale );

	// Double the width and undo the height scaling
	scale.x *= 2;
	scale.y /= 2;
	float top      = pixelSum(image,pos,scale);
	float bottom   = pixelSum(image,pos + cv::Point(0,scale.y),scale );

    return (left > right ? 0 : 2) + ( top > bottom ? 0 : 1);
}
