#include <tracker.h>

Tracker::Tracker()
{
	m_settings.criteria.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;
	m_settings.criteria.maxCount = 20;
	m_settings.criteria.epsilon  = 0.03;
	m_settings.windowSize.width  = 31;
	m_settings.windowSize.height = 31;
}

void Tracker::update(cv::Mat const &frame,
	                 Points  const &in,
			         Points        &out)
{
	// Convert the input to grayscale
	cv::cvtColor(frame,m_gray,CV_BGR2GRAY);

	// Clear the status and err settings
	m_settings.status.clear();
	m_settings.err.clear();

	// Calculate the flow
	if( m_prevGray.empty() ) m_gray.copyTo(m_prevGray);
	cv::calcOpticalFlowPyrLK(m_prevGray, m_gray, in, out, m_settings.status,
			                 m_settings.err, m_settings.windowSize,
	                         3, m_settings.criteria, 0, 0, 0.001);

	// Remove any points that were not found
	size_t i,k;
	for(i = k = 0; i < out.size(); ++i)
	{
		if(!m_settings.status[i]) continue;
		out[k++] = out[i];
	}
	out.resize(k);

	// Swap the images
	cv::swap(m_prevGray,m_gray);
}
