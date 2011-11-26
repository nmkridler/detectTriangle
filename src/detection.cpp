
#include "detection.h"

// Constructor for the detection
Detection::Detection( Settings const & settings ) :
m_settings(settings),
m_tracking(false)
{
	m_boxSize.x = 60;
	m_boxSize.y = 60;
	m_classifier.reset( new Classifier(10,5,cv::Point2f(0.1,0.5)));
}
void Detection::setTrackBox( Contact const & box)
{
	m_track    = box;
	m_boxSize  = box.dims;
    m_tracking = true;
}
