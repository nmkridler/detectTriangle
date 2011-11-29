#ifndef __ENGINE_H__
#define __ENGINE_H__

#include <cv.h>
#include <highgui.h>
#include <libfreenect.h>
#include <kinectdevice.h>
#include <display.h>
#include <settings.h>
#include <filters.h>
#include <triangles.h>
#include <tracker.h>
#include <slidingwindow.h>
#include <detection.h>
#include <constants.h>
enum FilterToggle
{
	DEPTH,
	ORANGE,
	CONTOURS
};


/////////////////////////////////////////////////////////////////////////////
// Engine
//
// Controls the various pieces of the detection process
//
/////////////////////////////////////////////////////////////////////////////
class Engine
{
public:
    // Constructor with settings
    Engine( Settings const & settings);

    // Destructor
    ~Engine()
    {
    	m_device->stopVideo();
    	m_device->stopDepth();
    }

    // update
    void update();

    // Set the output
    void setOutput();

    // Access to the camera
    KinectDevicePtr const &device() const {return m_device;}

    // Toggle the images
    void toggleFilter(int const & key ){ m_filterToggle = key;}

    // Set bounding box
    void setBox( BoundingBox const & box);

    void tldInitialize();

    void tldUpdate();

    void topTriangle();

private:

    // Run settings
    Settings     m_settings;

    // Number of misses
    int          m_missCount;

    // data frames
    cv::Mat      m_rgb;
    cv::Mat      m_depth;
    cv::Mat      m_depthRaw;
    cv::Mat      m_out;
    cv::Mat      m_mask;
    cv::Mat      m_gray;
    cv::Mat      m_integral;

    Contact                m_contact;

    bool                   m_reInit;
    bool                   m_initialized;
    double                 m_confidence;
    BoundingBox            m_box;

    Freenect::Freenect     freenect;          // Freenect device

    KinectDevicePtr        m_device;          // Get the video frames

    DetectionPtr           m_detector;        // Detection module

    DisplayPtr             m_display;         // Output to screen

    Tracker                m_tracker;         // Optical flow tracker

    int                    m_filterToggle;

};
#endif
