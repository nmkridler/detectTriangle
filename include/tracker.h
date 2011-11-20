#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <iostream>
#include <cv.h>
#include <constants.h>
#include <boost/shared_ptr.hpp>
#include <settings.h>
#include <vector>
#include <ctype.h>

struct TrackerSettings
{
	cv::TermCriteria   criteria;
	std::vector<uchar> status;
	std::vector<float> err;
	cv::Size           windowSize;
};
class Tracker
{
public:
	// Constructor
	Tracker();

	// Destructor
	~Tracker(){}

	// Get the updated location
	void update(cv::Mat const &frame,
	            Points  const &in,
			    Points        &out);

private:
    // Tracker Options
    TrackerSettings  m_settings;

    // Grayscale images
    cv::Mat          m_gray;
    cv::Mat          m_prevGray;

};
#endif
