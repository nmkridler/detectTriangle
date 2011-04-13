#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <cv.h>
#include <highgui.h>
#include "libfreenect.h"
#include "kinectdevice.h"
#include "triangles.h"
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#define GLEW_STATIC 1
#include <GL/glew.h>
#include <GL/glut.h>

using namespace cv;

// This class encapsulates all of the GPGPU functionality of the example.
class driver
{
public: // methods
    driver(int w, int h);

    ~driver(){};

    // This method updates the texture by rendering the geometry (a teapot 
    // and 3 rotating tori) and copying the image to a texture.  
    // It then renders a second pass using the texture as input to an edge 
    // detection filter.  It copies the results of the filter to the texture.
    // The texture is used in HelloGPGPU::display() for displaying the 
    // results.
    void update();
 
    void display();

    void setTilt(double &tiltAngle);

    void cleanUp();

protected: // data
    int           _iWidth, _iHeight; // The dimensions of our array
    
    unsigned int  _iTexture;         // The texture used as a data array

    GLhandleARB   _programObject;    // the program used to update
    GLhandleARB   _fragmentShader;

    GLint         _texUnit;          // a parameter to the fragment program

    //GLhandleARB   _normPrgObject;    // the program used to update
    //GLhandleARB   _normFrgShader;
    Freenect::Freenect freenect;
    Triangles*     device;
    Mat            frame;
};
#endif

