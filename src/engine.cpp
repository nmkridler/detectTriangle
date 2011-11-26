#include <engine.h>

Engine::Engine(Settings const & settings) :
m_settings(settings),
m_reInit(false),
m_confidence(1.),
m_filterToggle(0)
{
	// Initialize the bounding box
	m_box.set = false;

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
    m_detector.reset( new SlidingWindow(m_settings,m_rgb.size()) );

    // Create the display module
    m_display.reset( new Display(1280,480) );
    update();

}

void Engine::setBox( BoundingBox const & box )
{
	m_box     = box;
	m_box.set = true;
	m_reInit  = true;
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
    //Points::iterator contacts = m_events.begin();

    m_rgb.copyTo(leftSide);
#if 0
    if( !m_events.empty())
    {

		cv::circle(leftSide, *contacts, 60, cv::Scalar(0,0,255),5);
		++contacts;
		while( contacts != m_events.end() )
		{
		   cv::circle(leftSide, *contacts, 60, cv::Scalar(255,0,0),5);
		   ++contacts;
		}

		cv::rectangle(leftSide,m_box.lowerLeft,m_box.lowerLeft+m_box.size,
				      Scalar(0,0,255),5);
    }
#endif
    if( m_box.set )
    {
		cv::rectangle(leftSide,m_box.lowerLeft,m_box.lowerLeft+m_box.size,
				      Scalar(0,0,255),5);
    }
    switch(m_filterToggle){
        case DEPTH:
           m_depth.copyTo(rightSide);
           break;
        case CONTOURS:
           m_rgb.copyTo(rightSide);
           Filters::binaryFilter(rightSide,m_mask);
           rightSide.setTo(cv::Scalar(255,255,255),m_mask);
           break;
        case ORANGE:
           m_rgb.copyTo(rightSide);
           Filters::filterOrange(rightSide);
           break;
        default: break;
    }


}
void Engine::update()
{

    if(m_device->getVideo(m_rgb) && m_device->getDepth(m_depthRaw))
    {
       m_device->depthViewColor(m_depth);

       // Create an integral image
 	   cv::cvtColor(m_rgb,m_gray,CV_BGR2GRAY);
 	   cv::integral(m_gray,m_integral,CV_32F);

       // Train the classifier based on user selection
       if( m_reInit && m_box.set )
       {

    	  m_detector->classifier()->train(m_integral,m_box.lowerLeft, m_box.size,1);
    	  m_contact.position = m_box.lowerLeft;
    	  m_contact.dims     = m_box.size;
    	  m_contact.score    = 0.;
    	  m_contact.valid    = true;
    	  m_detector->setTrackBox(m_contact);
    	  m_reInit = false;

       }

       if( !m_reInit && m_box.set)
       {
    	  Contact newContact;

    	  if( m_confidence > 0.1)
    	  {
    		  m_tracker.update(m_rgb,m_contact,newContact);
    		  // Determine the score for the new contact
    		  newContact.score = m_detector->classifier()->classify(m_integral,
    				                                                newContact.position,
    				                                                newContact.dims);
              std::cout << newContact.score << std::endl;
    		  m_detector->setTrackBox(newContact);
    		  m_detector->processFrame(m_rgb,m_depthRaw);
    		  m_box.set = true;
    	  } else
    	  {
              m_detector->processFrame(m_rgb,m_depthRaw);
              m_tracker.setPrevious(m_rgb);
              newContact.score = 0.1;
              m_box.set = false;
    	  }

    	  // Get the greatest detected patch confidence
          ContactList contacts = m_detector->getDetections();
          double maxScore = 0;
          size_t maxIdx   = 0;
          for( size_t idx = 0; idx < contacts.size(); ++idx)
          {
        	  if( contacts[idx].score > maxScore)
        	  {
        		  maxScore = contacts[idx].score;
        		  maxIdx   = idx;
        	  }
          }

          if( maxScore > newContact.score && maxScore > 0.8)
          {
        	  newContact = contacts[maxIdx];
          } else if( newContact.score > maxScore && m_confidence > 0.8)
          {
        	  // Loop over the detections
        	  for( size_t idx = 0; idx < contacts.size(); ++idx)
        	  {
        		  if( contacts[idx].valid)
        		  {
        			  m_detector->classifier()->train(m_integral,
        					                          contacts[idx].position,
        					                          contacts[idx].dims,1);
        		  } else
        		  {
        			  m_detector->classifier()->train(m_integral,
        					                          contacts[idx].position,
        					                          contacts[idx].dims,0);
        		  }
        	  }
          }
          // set confidence
          m_confidence = newContact.score;
		  m_box.lowerLeft = newContact.position;
		  m_box.size      = newContact.dims;
          m_contact    = newContact;

       }


       setOutput();
    }
    m_display->update(m_out);
}

