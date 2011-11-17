#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <iostream>
#include <cv.h>
#include <constants.h>
#include <boost/shared_ptr.hpp>
#include <list>

struct Contact
{
	cv::Point position;
	double    score;
	int       misses;
};

typedef std::list<Contact> ContactList;

class Detection 
{
public:
   // Constructor
   Detection(int const & maxDet);

   // Destructor
   ~Detection(){}

   // Getter for the contact list
   ContactList const & getDetections() const { return m_list; }

   // Process a frame of data
   virtual void processFrame(cv::Mat const & rgb, cv::Mat const & depth)=0;


protected:
   int             m_maxDetections;  //
   ContactList     m_list;           // Contact List

};

typedef boost::shared_ptr<Detection> DetectionPtr;
#endif
