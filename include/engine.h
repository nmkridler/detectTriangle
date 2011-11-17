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
    ~Engine(){}

    // update
    void update();

    void setOutput();

private:
    // Run settings
    Settings     m_settings;

    // data frames
    cv::Mat      m_rgb;
    cv::Mat      m_depth;
    cv::Mat      m_depthRaw;
    cv::Mat      m_out;

    Freenect::Freenect     freenect;          // Freenect device

    KinectDevicePtr        m_device;           // Get the video frames

    DetectionPtr           m_detector;        // Detection module

    DisplayPtr             m_display;         // Output to screen


};
#endif
