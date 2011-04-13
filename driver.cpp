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

// This shader performs RGB normalization.
static const char *normFragSource = {
"uniform sampler2D texUnit;"
"void main(void)"
"{"
"   const float offset = 1.0 / 512.0;"
"   vec2 texCoord = gl_TexCoord[0].xy;"
"   vec4 c  = texture2D(texUnit, texCoord);"
"   float cRed = (float)c.r; "
"   float cGrn = (float)c.g; "
"   float cBlu = (float)c.b; "
"   float norm = sqrt(cRed*cRed + cGrn*cGrn + cBlu*cBlu);"
"   gl_FragColor.a = 1.0;"
"   gl_FragColor.r = c.r/norm; "
"   gl_FragColor.g = c.g/norm; "
"   gl_FragColor.b = c.b/norm; "
"}"
};


driver::driver(int w, int h)
    : _iWidth(w),
      _iHeight(h)
{
    device = &freenect.createDevice<Triangles>(0);
    // Start the freenect device
    device->startVideo();
    device->startDepth();
     
    // Create a simple 2D texture.  This example does not use
    // render to texture -- it just copies from the framebuffer to the
    // texture.
    glGenTextures(1, &_iTexture);
    glBindTexture(GL_TEXTURE_2D, _iTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    // Get webcam feed
    device->getVideo(frame);
   
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, _iWidth, _iHeight, 
    //             0, GL_RGB, GL_FLOAT, 0);
       
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, _iWidth, _iHeight, 
                 0, GL_BGR, GL_UNSIGNED_BYTE, frame.data);
    // GPGPU CONCEPT 2: Fragment Program = Computational Kernel.
    // A fragment program can be thought of as a small computational 
    // kernel that is applied in parallel to many fragments 
    // simultaneously.  Here we load a kernel that performs an edge 
    // detection filter on an image.
    _programObject = glCreateProgramObjectARB();

    // Create the edge detection fragment program
    _fragmentShader = glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
    glShaderSourceARB(_fragmentShader, 1, &normFragSource, NULL);
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


// This method updates the texture by rendering the geometry (a teapot 
// and 3 rotating tori) and copying the image to a texture.  
// It then renders a second pass using the texture as input to an edge 
// detection filter.  It copies the results of the filter to the texture.
// The texture is used in HelloGPGPU::display() for displaying the 
// results.
void driver::update()
{   
    // Get the opencv frame
    device->getVideo(frame);
#if 0
    device->setOwnMat();
    Mat orangeFrame;
    frame.copyTo(orangeFrame);
    device->filterOrange(orangeFrame);
    device->contourImg();
    vector<Point> cMass;
    device->getDetectCM(cMass);
    //frame.copyTo(orangeFrame);
    if( device->foundTarget() )
    {
       for(unsigned int dIdx = 0; dIdx < cMass.size(); dIdx++)
       {  
          circle(orangeFrame, cMass[dIdx], 60, Scalar(0,0,255),5);
       }
    }
#endif

    // store the window viewport dimensions so we can reset them,
    // and set the viewport to the dimensions of our texture
    int vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);

    // GPGPU CONCEPT 3a: One-to-one Pixel to Texel Mapping: A Data-
    //                   Dimensioned Viewport.
    glViewport(0, 0, _iWidth, _iHeight);
    glClear(GL_COLOR_BUFFER_BIT); 
    glBindTexture(GL_TEXTURE_2D, _iTexture);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, _iWidth, _iHeight, 
                 0, GL_BGR, GL_UNSIGNED_BYTE, frame.data);
#if 1
    // run the edge detection filter over the geometry texture
    // Activate the edge detection filter program
    glUseProgramObjectARB(_programObject);
           
    // identify the bound texture unit as input to the filter
    glUniform1iARB(_texUnit, 0);
         
    // GPGPU CONCEPT 4: Viewport-Sized Quad = Data Stream Generator.
    // In order to execute fragment programs, we need to generate pixels.
    // Drawing a quad the size of our viewport (see above) generates a 
    // fragment for every pixel of our destination texture. Each fragment
    // is processed identically by the fragment program. Notice that in 
    // the reshape() function, below, we have set the frustum to 
    // orthographic, and the frustum dimensions to [-1,1].  Thus, our 
    // viewport-sized quad vertices are at [-1,-1], [1,-1], [1,1], and 
    // [-1,1]: the corners of the viewport.
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

    // GPGPU CONCEPT 5: Copy To Texture (CTT) = Feedback.
    // We have just invoked our computation (edge detection) by applying 
    // a fragment program to a viewport-sized quad. The results are now 
    // in the frame buffer. To store them, we copy the data from the 
    // frame buffer to a texture.  This can then be fed back as input
    // for display (in this case) or more computation (see 
    // more advanced samples.)
    // update the texture again, this time with the filtered scene
    glBindTexture(GL_TEXTURE_2D, _iTexture);
    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, _iWidth, _iHeight);
#endif
        
    // restore the stored viewport dimensions
    glViewport(vp[0], vp[1], vp[2], vp[3]);
}

void driver::display()
{
    // Bind the filtered texture
    glBindTexture(GL_TEXTURE_2D, _iTexture);
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

void driver::setTilt(double &tiltAngle)
{
   device->setTiltDegrees(tiltAngle);
}

void driver::cleanUp()
{
   device->stopVideo();
   device->stopDepth();
}
