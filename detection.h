#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <iostream>
#include <vector>
#include <cmath>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "constants.h"
#include "stats.h"

using namespace cv;
using namespace std;
using namespace KinectConstants;

class Detection 
{
   public:
      // Constructor
      Detection(){}

      Detection( Point &cMass, float &score);

      // Misses
      void  addMiss(){ m_missCount++;}
      void  resetMisses(){ m_missCount = 0;}

      // Setters
      void  setCentMass( Point &centMass ) { m_centMass = centMass;}
      void  setScore( float &score ){ m_score = score;}

      // Getters
      void  getCentMass( Point &centMass ) const {centMass = m_centMass;}
      float getScore() const { return m_score; }
      unsigned int getMissCount() const {return m_missCount;}
      // Destructor
      ~Detection(){}
   private:
      Point           m_centMass;        // Center of mass 
      unsigned int    m_score;           // Score
      unsigned int    m_missCount;       // Number of misses
};

#endif
