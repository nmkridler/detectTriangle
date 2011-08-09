#include "glsobel.h"


// This shader performs a Sobel edge detection filter.
// (Modeled after Laplacian example from GPGPU tutorial)
// The floats should be passed in as uniform, but for now constant is fine
static const char *edgeFragSource = {
"uniform sampler2D texUnit;"
"void main(void)"
"{"
"   const float hoffset = 1.0 / 640.0;"
"   const float voffset = 1.0 / 480.0;"
"   vec2 texCoord = gl_TexCoord[0].xy;"
"   vec4 bl = texture2D(texUnit, texCoord + vec2(-hoffset, -voffset));"
"   vec4 l  = texture2D(texUnit, texCoord + vec2(-hoffset,     0.0));"
"   vec4 tl = texture2D(texUnit, texCoord + vec2(-hoffset,  voffset));"
"   vec4 t  = texture2D(texUnit, texCoord + vec2(     0.0,  voffset));"
"   vec4 ur = texture2D(texUnit, texCoord + vec2( hoffset,  voffset));"
"   vec4 r  = texture2D(texUnit, texCoord + vec2( hoffset,     0.0));"
"   vec4 br = texture2D(texUnit, texCoord + vec2( hoffset,  voffset));"
"   vec4 b  = texture2D(texUnit, texCoord + vec2(     0.0, -voffset));"
"   vec4 hEdge = -tl - 2.0*t - ur + bl + 2.0*b + br;"
"   vec4 vEdge =  tl + 2.0*l + bl - ur - 2.0*r - br;"
"   gl_FragColor.rgb = sqrt(dot(hEdge.rgb,hEdge.rgb) + dot(vEdge.rgb,vEdge.rgb));"
"   gl_FragColor.a = 1.0;"
"}"
};

GLSobel::GLSobel(int const & w, int const & h)
    : m_iWidth(w),
      m_iHeight(h)
{
    // Initialize the shader
    initShader();    
}

// Initialize the shader program
void GLSobel::initShader()
{

    // Computational Kernel.
    m_programObject = glCreateProgramObjectARB();

    // Create the edge detection fragment program
    m_fragmentShader = glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
    glShaderSourceARB(m_fragmentShader, 1, &edgeFragSource, NULL);
    glCompileShaderARB(m_fragmentShader);
    glAttachObjectARB(m_programObject,m_fragmentShader);

    // Link the shader into a complete GLSL program.
    glLinkProgramARB(m_programObject);
    GLint progLinkSuccess;
    glGetObjectParameterivARB(m_programObject, GL_OBJECT_LINK_STATUS_ARB,
            &progLinkSuccess);
    if (!progLinkSuccess)
    {
        fprintf(stderr, "Filter shader could not be linked\n");
        exit(1);
    }
    // Get location of the sampler uniform
    m_texUnit = glGetUniformLocationARB(m_programObject, "texUnit");

}

// Initialize the frame buffer object
void GLSobel::initFBO()
{
    // Now create a frame buffer object
    glGenFramebuffersEXT(1,&m_fbo);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,m_fbo);
    glMatrixMode(GL_PROJECTION);    
    glLoadIdentity();               
    gluOrtho2D(-1, 1, -1, 1);       
    glMatrixMode(GL_MODELVIEW);     
    glLoadIdentity();    
}

// Display the output texture
void GLSobel::display()
{
    // Bind the filtered texture
    glBindTexture(GL_TEXTURE_2D, m_outputTex);
    glEnable(GL_TEXTURE_2D);

    // Render
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


// Set up the texture
void GLSobel::setupTexture( const GLuint texID, int width, int height)
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


// Transfer data to the texture
void GLSobel::transferToTexture(cv::Mat const &input, GLuint texID)
{
    glBindTexture(GL_TEXTURE_2D, texID);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, input.cols, input.rows, 
                  0, GL_BGR, GL_UNSIGNED_BYTE, input.data);
   
}
// Transfer texture to the Mat
void GLSobel::transferFromTexture( cv::Mat &output)
{
    glReadBuffer(GL_COLOR_ATTACHMENT1_EXT); 
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    glReadPixels(0,0,m_iWidth,m_iHeight,GL_BGR,GL_UNSIGNED_BYTE,output.data);
}

// Run the RGB filter
void GLSobel::shader()
{   
    // Initialize the frame buffer object
    initFBO();

    // Attach a READ texture
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT0_EXT,
                              GL_TEXTURE_2D,m_iTexture[0],0);
    // Attach a WRITE texture
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT1_EXT,
                              GL_TEXTURE_2D,m_iTexture[1],0);
    glDrawBuffer(GL_COLOR_ATTACHMENT1_EXT);

    // Set up the window and bind the image to the texture
    int vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    glViewport(0, 0, m_iWidth, m_iHeight);
    glClear(GL_COLOR_BUFFER_BIT);

    // Transfer the frame to the read texture 
    transferToTexture(frame,m_iTexture[0]);
        
    // run the edge detection filter over the geometry texture
    // Activate the edge detection filter program
    glUseProgramObjectARB(m_programObject);
           
    // identify the bound texture unit as input to the filter
    glUniform1iARB(m_texUnit, 0);
         
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
    glDeleteFramebuffersEXT(1,&m_fbo); 

    // restore the stored viewport dimensions
    glViewport(vp[0], vp[1], vp[2], vp[3]);
}
