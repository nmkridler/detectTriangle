#include <tracker.h>

Tracker::Tracker()
{
	m_settings.criteria.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;
	m_settings.criteria.maxCount = 20;
	m_settings.criteria.epsilon  = 0.03;
	m_settings.windowSize.width  = 4;
	m_settings.windowSize.height = 4;
}


void Tracker::update(cv::Mat     const & frame,
	                 Contact     const & in,
			         Contact           & out)
{

	// Clear the status and err settings
	m_settings.status.clear();
	m_settings.err.clear();

	Points prevPts, nextPts;
	float xStep = in.dims.x/11.;
	float yStep = in.dims.y/11.;

	// Distribute points over the bounding box
	for( int x = 1; x < 11; ++x)
	{
		for( int y = 1; y < 11; ++y)
		{
			float xInc = static_cast<float>(x)*xStep;
			float yInc = static_cast<float>(y)*yStep;
			cv::Point2f thisPt(static_cast<float>(in.position.x) + xInc,
					       static_cast<float>(in.position.y) + yInc);
			prevPts.push_back(thisPt);

		}
	}

	// Calculate the flow
	if( m_prevGray.empty() ) frame.copyTo(m_prevGray);
	cv::calcOpticalFlowPyrLK(m_prevGray, frame, prevPts, nextPts,
			                 m_settings.status, m_settings.err,
			                 m_settings.windowSize, 3, m_settings.criteria,
			                 0, 0, 0.001);

	// Determine the displacement of any points that were
	// successfully tracked
	std::vector<float> xDist;
	std::vector<float> yDist;
	for(size_t i = 0; i < nextPts.size(); ++i)
	{
		if(!m_settings.status[i])
		{
		   xDist.push_back( nextPts[i].x - prevPts[i].x );
		   yDist.push_back( nextPts[i].y - prevPts[i].y );
		}
	}

	// Sort the vectors
	std::sort(xDist.begin(),xDist.end());
	std::sort(yDist.begin(),yDist.end());
    size_t idx = xDist.size()/2;

    cv::Point2f medianDist(xDist[idx],yDist[idx]);

    // Check the scale
    std::vector<float> scales;
    for( int i = 0; i < 100; ++i)
    {
    	for( int j = i + 1; j < 100; ++j)
    	{
    		if(m_settings.status[i] && m_settings.status[j])
    		{
    			cv::Point2f dPrev(prevPts[j].x - prevPts[i].x,
    			                  prevPts[j].y - prevPts[i].y);
    			cv::Point2f dNext(nextPts[j].x - nextPts[i].x,
    			                  nextPts[j].y - nextPts[i].y);
    			float sPrev = cv::norm(dPrev);
    			float sNext = cv::norm(dNext);
    			if( sPrev != 0 && sNext != 0)
    			{
    				scales.push_back(sNext/sPrev);
    			}
    		}
    	}
    }

    // Sort
    float scale = 0.0f;

    if( !scales.empty() )
    {
       std::sort(scales.begin(),scales.end());
       idx = scales.size()/2;
       scale = (scales[idx] + scales[idx+1])/2.0f - 1.0f;
    }

    cv::Point2f offset( static_cast<float>(in.dims.x)*scale/2.0f,
    		            static_cast<float>(in.dims.y)*scale/2.0f);

    // Calculate offsets
    out.position.x = in.position.x - static_cast<int>(offset.x - medianDist.x);
    out.position.y = in.position.y - static_cast<int>(offset.y - medianDist.y);
    out.dims.x     = in.dims.x + static_cast<int>(offset.x*2.0f);
    out.dims.y     = in.dims.y + static_cast<int>(offset.y*2.0f);

}

void Tracker::setPrevious(cv::Mat const & frame)
{
	frame.copyTo(m_prevGray);
}
