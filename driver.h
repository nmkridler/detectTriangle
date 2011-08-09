#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <cv.h>
#include <highgui.h>
#include "libfreenect.h"
#include "kinectdevice.h"
#include "triangles.h"
#include "filters.h"
#include "glsobel.h"
#include "constants.h"

using namespace cv;

// This class is the driver for the orange triangle detection.
class driver : public GLSobel
{
public: // methods
    driver(int const & w, int const & h);

    ~driver()
    {
       device->stopVideo();
       device->stopDepth();
    }

    // Pulls a frame and runs it through GLUT
    void update();

    // Accumulate frames
    void accumulate();

    // Sobel Filter
    void shader();

    // Run the triangle detection portion of the code
    void runDetect();

    // Current status
    void showStatus();
  
    // Set the kinect tilt angle
    void setTilt(double &tiltAngle);

    // Set whether or not you want to see the depth field
    void setDepthFlag();

    // Set whether or not you want to see the orange field
    void setOrangeFlag();

    // Set whether or not you want to see the input field
    void resetFlags();
    
    // Create the input texture
    void createImgTexture(); 
 
    // Create the output texture
    void createOutTexture(); 
protected: // data

    Freenect::Freenect freenect;          // Freenect device
    TrianglesPtr       device;            // The kinect device and 
                                          // triangle detection object
    Mat                frame;             // The filtered frame
    Mat                dualFrame;         // The output image 
                                          // (left rgb, right out)
    Mat                orangeFrame;       // The filtered orange RGB frame
    Mat                rgbFrame;          // The RGB frame
    Mat                sumFrame;          // The Accumulated frame
    unsigned int       m_frameCount;      // Frame count
    bool               m_filterOrange;    // Show orange filter?
    bool               m_showDepth;       // Show depth?
    bool               m_findTriangles;   // Find triangles?
private:
    static unsigned int const  FRAMES_PER_STACK = 1;  
};
#endif

