#include <kinectdevice.h>
#include <constants.h>

// Constructor
KinectDevice::KinectDevice(freenect_context *_ctx, int _index)
        : Freenect::FreenectDevice(_ctx, _index),         // Device Constructor
         m_gamma(2048),                                   // gamma
         depthMat(Size(640,480),CV_16UC1),                // Depth matrix
         rgbMat(Size(640,480),CV_8UC3,Scalar(0)),         // RGB matrix
         m_buffer_depth(FREENECT_DEPTH_11BIT),            // depth buffer
         m_buffer_rgb(FREENECT_VIDEO_RGB),                // RGB buffer
         m_new_rgb_frame(false),                          // new RGB frame
         m_new_depth_frame(false)                         // new depth frame
{
   // Fill the gamma array
   for( unsigned int i = 0 ; i < 2048 ; i++) 
   {
      const float k1 = 1.1863;
      const float k2 = 2842.5;
      const float k3 = 0.1236;
      const float v = k3*tanf((float)i/k2 + k1);
      m_gamma[i] = v;
   }

}

// Do not call directly even in child
void KinectDevice::VideoCallback(void* _rgb, uint32_t timestamp) {
    m_rgb_mutex.lock();
    uint8_t* rgb = static_cast<uint8_t*>(_rgb); 
    rgbMat.data = rgb;
    m_new_rgb_frame = true;
    m_rgb_mutex.unlock();
}

// Do not call directly even in child
void KinectDevice::DepthCallback(void* _depth, uint32_t timestamp) {
    m_depth_mutex.lock();
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    depthMat.data = (uchar*) depth;
    m_new_depth_frame = true;
    m_depth_mutex.unlock();
}

bool KinectDevice::getVideo(Mat& output) {
   m_rgb_mutex.lock();
   if(m_new_rgb_frame) {
      cvtColor(rgbMat, output, CV_RGB2BGR);
      flip(output,output,0);
      m_new_rgb_frame = false;
      m_rgb_mutex.unlock();
      return true;
   } else {
      m_rgb_mutex.unlock();
      return false;
   }
}

bool KinectDevice::getDepth(Mat& output) {
   m_depth_mutex.lock();
   if(m_new_depth_frame) {
      depthMat.copyTo(output);
      flip(output,output,0);
      m_new_depth_frame = false;
      m_depth_mutex.unlock();
      return true;
   } else {
      m_depth_mutex.unlock();
      return false;
   }
}

//###############################################################
// depthViewColor
//
//   convert depthMat into color
//
//###############################################################
void KinectDevice::depthViewColor(Mat& output)
{ 
    
   // pointer to output data 
   unsigned char *depth_mid = output.data; 
   for (unsigned int i = 0; i < 640*480 ; i++) { 
      int lb = ((short *)depthMat.data)[i] % 256; 
      int ub = ((short *)depthMat.data)[i] / 256; 
      switch (ub) { 
         case 0: 
            depth_mid[3*i+2] = 255; 
            depth_mid[3*i+1] = 255-lb; 
            depth_mid[3*i+0] = 255-lb; 
            break; 
         case 1: 
            depth_mid[3*i+2] = 255; 
            depth_mid[3*i+1] = lb; 
            depth_mid[3*i+0] = 0; 
            break; 
         case 2: 
            depth_mid[3*i+2] = 255-lb; 
            depth_mid[3*i+1] = 255; 
            depth_mid[3*i+0] = 0; 
            break; 
         case 3: 
            depth_mid[3*i+2] = 0; 
            depth_mid[3*i+1] = 255; 
            depth_mid[3*i+0] = lb; 
            break; 
         case 4: 
            depth_mid[3*i+2] = 0; 
            depth_mid[3*i+1] = 255-lb; 
            depth_mid[3*i+0] = 255; 
            break; 
         case 5: 
            depth_mid[3*i+2] = 0; 
            depth_mid[3*i+1] = 0; 
            depth_mid[3*i+0] = 255-lb; 
            break; 
         default: 
            depth_mid[3*i+2] = 0; 
            depth_mid[3*i+1] = 0; 
            depth_mid[3*i+0] = 0; 
            break; 
      } 
   } 
   flip(output,output,0);

}

