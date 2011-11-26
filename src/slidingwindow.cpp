
#include "slidingwindow.h"

// Constructor for the detection
SlidingWindow::SlidingWindow( Settings const & settings,
		                      cv::Size const &frameSize) :
Detection(settings),
m_size(frameSize)
{
	std::cout << m_size.x << "," << m_size.y << std::endl;
}

double SlidingWindow::overlap(Contact const & box)
{
	if( box.position.x     > m_track.position.x + m_track.dims.x ||
	    m_track.position.x > box.position.x + box.dims.x         ||
	    box.position.y     > m_track.position.y + m_track.dims.y ||
	    m_track.position.y > box.position.y + box.dims.y ) return 0.;

	// The bounding boxes overlap, determine the area of overlap
	double width  = static_cast<double>(std::min(box.position.x + box.dims.x,
			                            m_track.position.x + m_track.dims.x) -
			                            std::max(box.position.x,m_track.position.x));
	double height = static_cast<double>(std::min(box.position.y + box.dims.y,
			                            m_track.position.y + m_track.dims.y) -
			                            std::max(box.position.y,m_track.position.y));

	double totalArea = static_cast<double>(box.dims.y*box.dims.x + m_track.dims.x*m_track.dims.y);
	// Amount of overlap
	return (width*height)/totalArea;

}

// Process a frame of data
void SlidingWindow::processFrame(cv::Mat const & rgb, cv::Mat const & depth)
{
	// Convert to gray scale
	cv::cvtColor(rgb,m_gray,CV_BGR2GRAY);

	// Convert the image to an integral image
    cv::integral(m_gray,m_integral,CV_32F);

    // Loop over all of the boxes
    generateBoundingBoxes();

}

void SlidingWindow::generateBoundingBoxes()
{
   cv::Point currentSize;
   cv::Point2d doubleBox;
   doubleBox.x = static_cast<double>(m_boxSize.x);
   doubleBox.y = static_cast<double>(m_boxSize.y);
   // Create a list of bounding boxes to search through
   for( int scaleIdx = 0; scaleIdx < 6; ++scaleIdx)
   {
	   // Calculate the scale for this index
	   double scale = static_cast<double>(scaleIdx)/5.0 + 0.5;

	   // Determine the x values
       currentSize.x = static_cast<int>(scale*doubleBox.x);
       int xMax  = m_size.x - currentSize.x;
       int xStep = xMax / 29;

       if( xStep <= 0)
       {
    	   xMax  = 0;
    	   xStep = 1;
       }

       // Loop over the x-values and generate the y-values
       for( int x = 0; x <= xMax; x += xStep)
       {
          currentSize.y = static_cast<int>(scale*doubleBox.y);
          int yMax  = m_size.y - currentSize.y;
          int yStep = yMax / 29;
          if( yStep <= 0 )
          {
        	  yMax  = 0;
        	  yStep = 1;
          }

          // Loop over y and build the boxes
          for( int y = 0; y <= yMax; y += yStep)
          {
        	  // Create a new contact
        	  Contact newBox;
        	  newBox.position.x = x;
        	  newBox.position.y = y;
        	  newBox.dims.x     = currentSize.x;
        	  newBox.dims.y     = currentSize.y;

        	  // Calculate the score
#if 0
        	  newBox.score      = m_classifier->classify(m_integral,
        			                                     newBox.position,
        			                                     newBox.dims);
#endif

        	  newBox.valid      = false;
        	  if( m_tracking && overlap(newBox) > 0.6)
        	  {
        		  newBox.score = 1.0;
        		  newBox.valid = true;
        	  }

              if( newBox.score > 0.5 ||  newBox.valid )
              {
        	     m_list.push_back(newBox);
              }
          }

       }
   }
}


