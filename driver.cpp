#include "driver.h"


// This shader performs a 9-tap Laplacian edge detection filter.
// (converted from the separate "edges.cg" file to embedded GLSL string)
static const char *edgeFragSource = {
"uniform sampler2D texUnit;"
"void main(void)"
"{"
"   const float offset = 1.0 / 512.0;"
"   vec2 texCoord = gl_TexCoord[0].xy;"
"   vec4 c  = texture2D(texUnit, texCoord);"
"   vec4 bl = texture2D(texUnit, texCoord + vec2(-offset, -offset));"
"   vec4 l  = texture2D(texUnit, texCoord + vec2(-offset,     0.0));"
"   vec4 tl = texture2D(texUnit, texCoord + vec2(-offset,  offset));"
"   vec4 t  = texture2D(texUnit, texCoord + vec2(    0.0,  offset));"
"   vec4 ur = texture2D(texUnit, texCoord + vec2( offset,  offset));"
"   vec4 r  = texture2D(texUnit, texCoord + vec2( offset,     0.0));"
"   vec4 br = texture2D(texUnit, texCoord + vec2( offset,  offset));"
"   vec4 b  = texture2D(texUnit, texCoord + vec2(    0.0, -offset));"
"   vec4 hEdge = -tl - 2.0*t - ur + bl + 2.0*b + br;"
"   vec4 vEdge =  tl + 2.0*l + bl - ur - 2.0*r - br;"
"   gl_FragColor.rgb = sqrt(dot(hEdge.rgb,hEdge.rgb) + dot(vEdge.rgb,vEdge.rgb));"
"   gl_FragColor.a = 1.0;"
"}"
};
//"   gl_FragColor = 8.0 * (c + -0.125 * (bl + l + tl + t + ur + r + br + b));"

driver::driver(int w, int h)
    : _iWidth(w),
      _iHeight(h),
      m_frameCount(0),
      m_filterOrange(false),
      m_showDepth(false),
      m_findTriangles(false)
{
    
    // Start the freenect device
    device = &freenect.createDevice<Triangles>(0);
    device->startVideo();
    device->startDepth();
    bigFrame = Mat::zeros(Size(1280,480),CV_8UC3);
    sumFrame = Mat::zeros(Size(_iWidth,_iHeight),CV_32FC3);
    if(!device->getVideo(rgbFrame))
    {
       rgbFrame = Mat::zeros(Size(_iWidth,_iHeight),CV_8UC3);
    }
    rgbFrame.copyTo(frame);
    rgbFrame.copyTo(orangeFrame);

    // Create a 2d texture for the input
    createImgTexture();
        
    // Create a 2d texture for the output
    createOutTexture();

    // Initialize the shader
    initShader();    
}

// Update
void driver::update()
{   
   // Get the RGB frame and copy to frame for processing
   if( m_findTriangles )
   { 
      // Scale the accumulated data
      double alpha = 1.0/static_cast<double>(FRAMES_PER_STACK);
      sumFrame.convertTo(frame,CV_8UC3,alpha);
  
      // Reset everything
      m_frameCount = 0;
      m_findTriangles = false;
      Mat zeroFrame = Mat::zeros(Size(_iWidth,_iHeight),CV_32FC3);
      zeroFrame.copyTo(sumFrame);

      // Equalize the image and filter for orange 
      Filters::equalizeRGB(frame);
      Filters::filterOrange(frame);   // Get only the orange in HSV

      // Run the edge detection
      shader();

      // Get the opencv frame
      runDetect();
   }
   showStatus();
   transferToTexture(bigFrame,_outputTex);
   
}

void driver::accumulate()
{
   if( device->getVideo(rgbFrame) )
   {
      m_frameCount++;
      Mat floatFrame = Mat::zeros(Size(_iWidth,_iHeight),CV_32FC3);
      rgbFrame.convertTo(floatFrame,CV_32FC3);
      cv::accumulate(floatFrame,sumFrame);
   }
   if( m_frameCount == FRAMES_PER_STACK ) m_findTriangles = true;
   update();
}
// Initialize the shader program
void driver::initShader()
{

    // GPGPU CONCEPT 2: Fragment Program = Computational Kernel.
    _programObject = glCreateProgramObjectARB();

    // Create the edge detection fragment program
    _fragmentShader = glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
    glShaderSourceARB(_fragmentShader, 1, &edgeFragSource, NULL);
    glCompileShaderARB(_fragmentShader);
    glAttachObjectARB(_programObject, _fragmentShader);

    // Link the shader into a complete GLSL program.
    glLinkProgramARB(_programObject);
    GLint progLinkSuccess;
    glGetObjectParameterivARB(_programObject, GL_OBJECT_LINK_STATUS_ARB,
            &progLinkSuccess);
    if (!progLinkSuccess)
    {
        fprintf(stderr, "Filter shader could not be linked\n");
        exit(1);
    }
    // Get location of the sampler uniform
    _texUnit = glGetUniformLocationARB(_programObject, "texUnit");

}


void driver::initFBO()
{
    // Now create a frame buffer object
    glGenFramebuffersEXT(1,&_fbo);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,_fbo);
    glMatrixMode(GL_PROJECTION);    
    glLoadIdentity();               
    gluOrtho2D(-1, 1, -1, 1);       
    glMatrixMode(GL_MODELVIEW);     
    glLoadIdentity();    
}

// Run the RGB filter
void driver::shader()
{   

    // Initialize the frame buffer object
    initFBO();

    // Attach a READ texture
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT0_EXT,
                              GL_TEXTURE_2D,_iTexture[0],0);
    // Attach a WRITE texture
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT1_EXT,
                              GL_TEXTURE_2D,_iTexture[1],0);
    glDrawBuffer(GL_COLOR_ATTACHMENT1_EXT);

    // Set up the window and bind the image to the texture
    int vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    glViewport(0, 0, _iWidth, _iHeight);
    glClear(GL_COLOR_BUFFER_BIT);

    // Transfer the frame to the read texture 
    transferToTexture(frame,_iTexture[0]);
        
    // run the edge detection filter over the geometry texture
    // Activate the edge detection filter program
    glUseProgramObjectARB(_programObject);
           
    // identify the bound texture unit as input to the filter
    glUniform1iARB(_texUnit, 0);
         
    // GPGPU CONCEPT 4: Viewport-Sized Quad = Data Stream Generator.
    glBegin(GL_QUADS);
    {            
        glTexCoord2f(0, 0); glVertex3f(-1, -1, -0.5f);
        glTexCoord2f(1, 0); glVertex3f( 1, -1, -0.5f);
        glTexCoord2f(1, 1); glVertex3f( 1,  1, -0.5f);
        glTexCoord2f(0, 1); glVertex3f(-1,  1, -0.5f);
    }
    glEnd();
      
    // disable the filter
    glUseProgramObjectARB(0);

    // Get the results
    transferFromTexture(frame);
    glDeleteFramebuffersEXT(1,&_fbo); 

    // restore the stored viewport dimensions
    glViewport(vp[0], vp[1], vp[2], vp[3]);
}

// Display the output texture
void driver::display()
{
    // Bind the filtered texture
    glBindTexture(GL_TEXTURE_2D, _outputTex);
    glEnable(GL_TEXTURE_2D);

    // render a full-screen quad textured with the results of our 
    // computation.  Note that this is not part of the computation: this
    // is only the visualization of the results.
    glBegin(GL_QUADS);
    {
        glTexCoord2f(0, 0); glVertex3f(-1, -1, -0.5f);
        glTexCoord2f(1, 0); glVertex3f( 1, -1, -0.5f);
        glTexCoord2f(1, 1); glVertex3f( 1,  1, -0.5f);
        glTexCoord2f(0, 1); glVertex3f(-1,  1, -0.5f);
    }
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void driver::showStatus()
{
    // copy into the output
    Rect leftROI(    Point(0,0),frame.size());
    Rect rightROI( Point(640,0),frame.size());
    Mat  leftSide  = bigFrame(leftROI);
    Mat  rightSide = bigFrame(rightROI);
    rgbFrame.copyTo(leftSide);
    orangeFrame.copyTo(rightSide);

}

void driver::runDetect()
{
    // Set the frame we will be doing computations on
    device->setOwnMat(frame);  // This should be the sobel output

    rgbFrame.copyTo(orangeFrame);

    // Equalize the image and 
    Filters::equalizeRGB(orangeFrame);
    //if( m_showDepth ) device->depthViewColor(orangeFrame);
    if( m_showDepth ) frame.copyTo(orangeFrame);
    if( m_filterOrange ) Filters::filterOrange(orangeFrame);

    // Run the detection process
    device->contourImg();
    vector<Point> cMass;
    device->getDetectCM(cMass);
  
    // Output the detections
    if( device->foundTarget() )
    {
       for(unsigned int dIdx = 0; dIdx < cMass.size(); dIdx++)
       {  
          circle(orangeFrame, cMass[dIdx], 60, Scalar(0,0,255),5);
       }
    }

}


// Transfer data to the texture
void driver::transferToTexture(Mat &input, GLuint texID)
{
    glBindTexture(GL_TEXTURE_2D, texID);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, input.cols, input.rows, 
                  0, GL_BGR, GL_UNSIGNED_BYTE, input.data);
   
}
// Transfer texture to the Mat
void driver::transferFromTexture( Mat &output)
{
    glReadBuffer(GL_COLOR_ATTACHMENT1_EXT); 
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    glReadPixels(0,0,_iWidth,_iHeight,GL_BGR,GL_UNSIGNED_BYTE,output.data);
}

// Set up the texture
void driver::setupTexture( const GLuint texID, int width, int height)
{
    glBindTexture(GL_TEXTURE_2D, texID);

     // set basic parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    // Set up BGR texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 
                  0, GL_BGR, GL_UNSIGNED_BYTE, 0);
  
}
// Create the input texture
void driver::createImgTexture()
{
    // create a texture
    glGenTextures(2,_iTexture);
    setupTexture(_iTexture[0], _iWidth, _iHeight); // Read texture
    transferToTexture(frame,_iTexture[0]);
    setupTexture(_iTexture[1], _iWidth, _iHeight); // Read texture
    transferToTexture(frame,_iTexture[1]);

}
// Create the output texture
void driver::createOutTexture()
{
    // create a texture
    glGenTextures(1, &_outputTex);
    setupTexture(_outputTex, 2*_iWidth, _iHeight); // Read texture
    transferToTexture(bigFrame, _outputTex);
}

// Set the kinect angle
void driver::setTilt(double &tiltAngle)
{
   device->setTiltDegrees(tiltAngle);
}

// Set the depth flag
void driver::setDepthFlag()
{
   m_showDepth = !m_showDepth;
   if( m_showDepth && m_filterOrange ) setOrangeFlag();
}

void driver::setOrangeFlag()
{
   m_filterOrange = !m_filterOrange;
   if( m_showDepth && m_filterOrange ) setDepthFlag();
}

void driver::resetFlags()
{
   m_filterOrange = false;
   m_showDepth    = false;
}


