#include <display.h>

Display::Display(int const & w, int const & h) :
m_iWidth(w),
m_iHeight(h)
{
	// Initialize the opengl stuff
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Create a texture
    glGenTextures(1,&m_iTexture);
    setupTexture();
}


// Set up the texture
void Display::setupTexture()
{
    glBindTexture(GL_TEXTURE_2D, m_iTexture);

     // set basic parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    // Set up BGR texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_iWidth, m_iHeight,
                  0, GL_BGR, GL_UNSIGNED_BYTE, 0);

}


// Display the output texture
void Display::update(cv::Mat const &input)
{
	// Transfer the image to a texture
    glBindTexture(GL_TEXTURE_2D, m_iTexture);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, input.cols, input.rows,
                  0, GL_BGR, GL_UNSIGNED_BYTE, input.data);

    // Bind the filtered texture
    glBindTexture(GL_TEXTURE_2D, m_iTexture);
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
