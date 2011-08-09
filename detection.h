#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <iostream>
#include <cv.h>
#include "constants.h"

class Detection 
{
   public:
      // Constructor
      Detection(){}

      Detection( cv::Point const &cMass, float const &score);

      // Misses
      void  addMiss(){ m_missCount++;}
      void  resetMisses(){ m_missCount = 0;}

      // Setters
      void  setCentMass( cv::Point &centMass ) { m_centMass = centMass;}
      void  setScore( float const &score ){ m_score = score;}

      // Getters
      cv::Point const & getCentMass()  const { return m_centMass;}
      float     const & getScore()     const { return m_score; }
      size_t    const & getMissCount() const {return m_missCount;}

      // Destructor
      ~Detection(){}
   private:
      cv::Point       m_centMass;        // Center of mass 
      float           m_score;           // Score
      size_t          m_missCount;       // Number of misses
};

#endif
