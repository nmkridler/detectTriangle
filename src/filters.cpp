#include <filters.h>

// Filter the feed to get orange only 
void Filters::filterOrange( cv::Mat& output, cv::Scalar const & hsvmin, cv::Scalar const &hsvmax)
{
   // convert RGB to HSV
   cv::Mat tmpHSV;
   cv::cvtColor(output, tmpHSV, CV_BGR2HSV);

   // Create a mask for the orange color
   cv::Mat orangeImg = cv::Mat::zeros(tmpHSV.size(), CV_8UC1);
   cv::Mat orangeMsk(tmpHSV.size(), CV_8UC1);
   cv::inRange(tmpHSV,hsvmin,hsvmax, orangeMsk);

   // dilate(erode)
   cv::morphologyEx(orangeMsk,orangeMsk,cv::MORPH_OPEN,cv::Mat());
   cv::morphologyEx(orangeMsk,orangeMsk,cv::MORPH_CLOSE,cv::Mat());
   orangeImg.setTo(cv::Scalar(255),orangeMsk);

   // Find the non orange pixels and set to 0
   cv::inRange(orangeImg,cv::Scalar(0),cv::Scalar(254),orangeMsk);
   output.setTo(cv::Scalar(0,0,0),orangeMsk);
}

void Filters::edgeDetection(cv::Mat const &input, cv::Mat &output)
{
   // Canny edge detector
   cv::cvtColor(input,output,CV_BGR2GRAY);
   cv::GaussianBlur( output, output, cv::Size(9, 9), 2, 2 );
   cv::Canny( output, output, 50, 200, 3 );
}

void Filters::binaryFilter(cv::Mat const &input, cv::Mat &output)
{
   // Filter out the orange first
   cv::Mat tmpMat;
   input.copyTo(tmpMat);

   // Do the edge detection
   edgeDetection(tmpMat,output);

   // Make a binary mask
   output = output > 200;


}

void Filters::randWarpROI(cv::Mat    const & input,
		                  cv::Point  const & corner,
		                  cv::Point  const & size,
		                  cv::Mat & output )
{
	// Center of the patch
	cv::Point2f center(static_cast<float>(corner.x + size.x/2 - 1),
			           static_cast<float>(corner.y + size.y/2 - 1));

	// Create a random angle centered at 20 degrees
	double angle = 20.*randHalf();
	double scale = 1. - 0.02*randHalf();

	// Create the rotation matrix
	cv::Mat rotate = getRotationMatrix2D(center,angle,scale);
	rotate.at<double>(0,2) += 0.02*randHalf()*static_cast<double>(size.x);
	rotate.at<double>(1,2) += 0.02*randHalf()*static_cast<double>(size.y);

    // warp the image
    cv::warpAffine(input,output,rotate,input.size());

    // Add some white noise
	cv::Rect ROI(corner,size);
	cv::Mat  patch = output(ROI);
	cv::Mat  noise(patch.size(),patch.type(),cv::Scalar(0));
	cv::randn(noise,cv::Scalar(0),cv::Scalar(1));
	cv::Mat  tmp;
	cv::addWeighted(patch,1.,noise,5.,0.,tmp);
	tmp.copyTo(patch);
}

double Filters::randHalf()
{
	return static_cast<double>(rand())/static_cast<double>(RAND_MAX) - 0.5;
}
