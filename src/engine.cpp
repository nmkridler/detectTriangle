#include <engine.h>

Engine::Engine(Settings const & settings) :
m_settings(settings)
{
	// Create the kinect device
    m_device.reset(&freenect.createDevice<KinectDevice>(0));
    m_device->startVideo();
    m_device->startDepth();

    // Initialize the dual frame and the summation frame
    m_out = Mat::zeros(Size(1280,480),CV_8UC3);
    if(!m_device->getVideo(m_rgb))
       m_rgb = Mat::zeros(Size(640,480),CV_8UC3);

    if(!m_device->getDepth(m_depthRaw))
    {
       m_depthRaw = Mat::zeros(Size(640,480),CV_16UC1);
       m_depth    = Mat::zeros(Size(640,480),CV_8UC3);
    } else m_device->depthViewColor(m_depth);

    // Create the detection
    m_detector.reset( new Triangles(m_settings) );

    // Create the display module
    m_display.reset( new Display(1280,480) );
    update();

    // Initialize the events list
    m_events.clear();
}

void Engine::setOutput()
{
    // copy into the output
    Rect leftROI(    Point(0,0),m_rgb.size());
    Rect rightROI( Point(640,0),m_rgb.size());
    Mat  leftSide  = m_out(leftROI);
    Mat  rightSide = m_out(rightROI);

    // Get the list of outputs and copy them onto the left side
    // Output the detections
    Points::iterator contacts = m_events.begin();

    m_rgb.copyTo(leftSide);

    if( !m_events.empty())
    {
		cv::circle(leftSide, *contacts, 60, cv::Scalar(0,0,255),5);
		++contacts;
		while( contacts != m_events.end() )
		{
		   cv::circle(leftSide, *contacts, 60, cv::Scalar(255,0,0),5);
		   ++contacts;
		}
    }

    m_depth.copyTo(rightSide);

}
void Engine::update()
{

    if(m_device->getVideo(m_rgb) && m_device->getDepth(m_depthRaw))
    {
       m_device->depthViewColor(m_depth);

       // Run the detector
       m_detector->processFrame(m_rgb,m_depthRaw);

       // Run the tracker
       m_events.clear();
       if( !m_detector->getPositions().empty())
          m_tracker.update(m_rgb,m_detector->getPositions(),m_events);

       setOutput();
    }
    m_display->update(m_out);
}

