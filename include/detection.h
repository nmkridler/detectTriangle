#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <iostream>
#include <cv.h>
#include <constants.h>
#include <boost/shared_ptr.hpp>
#include <settings.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <classifier.h>

struct Contact
{
	cv::Point   position;
	cv::Point   dims;
	double      score;
	bool        valid;
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

   ClassifierPtr const & classifier() const {return m_classifier;}

   void setTrackBox(Contact const & box);


protected:
   Settings        m_settings;       // Detector settings
   ContactList     m_list;           // Contact List
   Points          m_positions;      // Contact positions
   ClassifierPtr   m_classifier;     // Classifier
   Contact         m_track;          // Top detection

   bool            m_tracking;
   cv::Point       m_boxSize;
};

typedef boost::shared_ptr<Detection> DetectionPtr;
#endif
