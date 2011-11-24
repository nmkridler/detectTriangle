
#include "slidingwindow.h"

// Constructor for the detection
SlidingWindow::SlidingWindow( Settings const & settings,
		                      cv::Size const &frameSize) :
Detection(settings),
m_size(frameSize)
{
}

double SlidingWindow::overlap(cv::Vec4d const & boxA,
		                      cv::Vec4d const & boxB)
{
	if( boxA[0] > boxB[0] + boxB[2] ||
	    boxB[0] > boxA[0] + boxA[2] ||
	    boxA[1] > boxB[1] + boxB[3] ||
	    boxB[1] > boxA[1] + boxA[3] ) return 0.;

	// The bounding boxes overlap, determine the area of overlap
	double width  = std::min(boxA[0] + boxA[2], boxB[0] + boxB[2]) -
			        std::max(boxA[0],boxB[0]);
	double height = std::min(boxA[1] + boxA[3], boxB[1] + boxB[3]) -
			        std::max(boxA[1],boxB[1]);

	// Amount of overlap
	return (width*height)/(boxA[2]*boxA[3] + boxB[2]*boxB[3] - width*height);

}

// Process a frame of data
void SlidingWindow::processFrame(cv::Mat const & rgb, cv::Mat const & depth)
{
	// Convert to gray scale
	cv::cvtColor(rgb,m_gray,CV_BGR2GRAY);

	// Convert the image to an integral image
    cv::integral(m_gray,m_integral,CV_32F);

}
