#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <iostream>
#include <cv.h>
#include <constants.h>
#include <boost/shared_ptr.hpp>
#include <settings.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>

struct Contact
{
	cv::Point   position;
	double      score;
	int         misses;
};

typedef std::vector<Contact> ContactList;

class Detection 
{
public:
   // Constructor
   Detection(Settings const & settings);

   // Destructor
   ~Detection(){}

   // Getter for the contact list
   ContactList const & getDetections() const { return m_list; }

   // Getter for the contact list
   Points      const & getPositions() const { return m_positions; }

   // Process a frame of data
   virtual void processFrame(cv::Mat const & rgb, cv::Mat const & depth)=0;


protected:
   Settings        m_settings;       // Detector settings
   ContactList     m_list;           // Contact List
   Points          m_positions;      // Contact positions
};

typedef boost::shared_ptr<Detection> DetectionPtr;
#endif
