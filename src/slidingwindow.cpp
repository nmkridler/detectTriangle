
#include "slidingwindow.h"

// Constructor for the detection
SlidingWindow::SlidingWindow( Settings const & settings,
		                      cv::Size const &frameSize) :
Detection(settings,frameSize)
{
}

// Process a frame of data
void SlidingWindow::processFrame(cv::Mat const & rgb, cv::Mat const & depth)
{
    // Loop over all of the boxes
    generateBoundingBoxes(rgb);
}

void SlidingWindow::generateBoundingBoxes(cv::Mat const & integral)
{
   m_list.clear();
   cv::Point currentSize;
   cv::Point2d doubleBox;
   doubleBox.x = static_cast<double>(m_boxSize.x);
   doubleBox.y = static_cast<double>(m_boxSize.y);
   if( doubleBox.x == 0 || doubleBox.y == 0)
   {
	   doubleBox.x = 60;
	   doubleBox.y = 60;
   }

   if( doubleBox.x >= 40 && doubleBox.y >= 40)
   {
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
				  newBox.score      = m_classifier->classify(integral,
															 newBox.position,
															 newBox.dims);

				  newBox.valid      = false;
				  if( m_tracking && overlap(newBox) > 0.6)
				  {
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
}

