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
    m_detector.reset( new Triangles(m_settings.m_maxDetections) );

    // Create the display module
    m_display.reset( new Display(1280,480) );
    update();

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
    ContactList detections = m_detector->getDetections();
    ContactList::iterator contacts = detections.begin();
    m_rgb.copyTo(leftSide);
    if( detections.size() > 0)
    {
		cv::circle(leftSide, contacts->position, 60, cv::Scalar(0,0,255),5);
		++contacts;
		while( contacts != detections.end() )
		{
		   cv::circle(leftSide, contacts->position, 60, cv::Scalar(255,0,0),5);
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

       setOutput();
    }
    m_display->update(m_out);
}
