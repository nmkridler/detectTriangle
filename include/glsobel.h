#ifndef __GLSOBEL_H__
#define __GLSOBEL_H__

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#define GLEW_STATIC 1
#include <GL/glew.h>
#include <GL/glut.h>


using namespace cv;

// This class is an interface to a GL shader
// In this case it's a sobel filter
class GLSobel
{
public: // methods
    // Constructor
    GLSobel(int const & w, int const & h);

    // Destructor
    virtual ~GLSobel(){}
    
    // Display the results
    void display();

    // Update the texture
    virtual void update()=0;

    // Run the shader
    void shader();

    // Create a texture for the image we want to process
    virtual void createImgTexture()=0;
    
    // Create a texture for the output image
    virtual void createOutTexture()=0;

    // Initialize the shader program
    void initShader();

    // Initialize the Frame Buffer Object
    void initFBO();  
   
    // Setup the texture
    void setupTexture(const GLuint texID, int width, int height);

    // Transfer the image to a texture
    void transferToTexture(Mat const &input, GLuint texID);
    
    // Transfer to texture to an image
    void transferFromTexture(Mat& output);


protected: // data
    int                m_iWidth;         
    int                m_iHeight;          // The dimensions of our array
    unsigned int       m_outputTex;        // Output texture
    unsigned int       m_iTexture[2];      // The texture used as a data array
                                           // 0 - read
                                           // 1 - write 
    GLhandleARB        m_programObject;    // the program used to update
    GLhandleARB        m_fragmentShader;   // Fragment shader
    GLint              m_texUnit;          // a parameter to the fragment program
    GLuint             m_fbo;              // Frame buffer object
    Mat                frame;              // Mat Frame
};
#endif

