#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#define GLEW_STATIC 1
#include <GL/glew.h>
#include <GL/glut.h>
#include <boost/shared_ptr.hpp>

using namespace cv;

// This class is an interface to a GL shader
// In this case it's a sobel filter
class Display
{
public: // methods
    // Constructor
    Display(int const & w, int const & h);

    // Destructor
    virtual ~Display(){}

    // Setup the texture
    void setupTexture();

    // Update the texture
    void update(Mat const &input);


protected: // data
    int                m_iWidth;
    int                m_iHeight;          // The dimensions of our array

    unsigned int       m_iTexture;      // The texture used as a data array

};

typedef boost::shared_ptr<Display> DisplayPtr;
#endif
