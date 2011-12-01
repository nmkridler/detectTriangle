
#include "detection.h"

// Constructor for the detection
Detection::Detection( Settings const & settings,
		              cv::Size const &frameSize) :
m_settings(settings),
m_tracking(false),
m_size(frameSize)
{
	m_boxSize.x = 100;
	m_boxSize.y = 100;
	m_classifier.reset( new Classifier(10,5,cv::Point2f(0.1,0.5)));
}
void Detection::setTrackBox( Contact const & box)
{
	m_track    = box;
	m_boxSize  = box.dims;
    m_tracking = true;
}
double Detection::overlap(Contact const & box)
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

	double totalArea = static_cast<double>(box.dims.y*box.dims.x + m_track.dims.x*m_track.dims.y) - width*height;

	// Amount of overlap
	return (width*height)/totalArea;

}

void Detection::trainNegative(cv::Mat const & integral)
{

   cv::Point currentSize;
   cv::Point2d doubleBox;
   doubleBox.x = static_cast<double>(m_boxSize.x);
   doubleBox.y = static_cast<double>(m_boxSize.y);

   // Create a list of bounding boxes to search through
   for( int scaleIdx = 0; scaleIdx < 5; ++scaleIdx)
   {
	   // Calculate the scale for this index
	   double scale = static_cast<double>(scaleIdx)/4.0 + 0.5;

	   // Determine the x values
       currentSize.x = static_cast<int>(scale*doubleBox.x);
       int xMax  = m_size.x - currentSize.x;
       int xStep = xMax / 19;

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
          int yStep = yMax / 19;
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

        	  if(overlap(newBox) ==  0.)
        	  {
        		  m_classifier->train(integral,newBox.position, newBox.dims,0);
        	  }

          }

       }
   }

}
