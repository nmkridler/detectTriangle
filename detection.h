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

      Detection( vector<Point> &contact);

      // Getters
      // Get distance to vertices
      void getDistance( vector<float> &distance ) const;

      // Center of mass
      void getCentMass( Point &centMass ) const;

      // Get Vertices
      void getVertices( vector<Point> &contact ) const;

      // Get Valid
      bool validDetection() const;

      // Get Lengths
      void getLengths( vector<float> &length) const;

      // Get hit count
      unsigned int getMissCount() const;

      unsigned int getHitCount() const;


      // Increment hit count
      void addMiss();
      void addHit();
      // Setters
      void setDistance( vector<float> &distance );
    
      // Set center of mass
      void setCentMass( Point &centMass );

      // Set valid
      void setValid( bool valid );

      // Set hit count to zero
      void resetMissCount();

      // Convert Pixel(X,Y),depth to xyz
      void pixelToMetric();
   
      // Determine if it has a right angle
      void setRightAngle();
    
      // Set side lengths
      void setSideLength();
     
      // Set color score
      void setColorScore( float &colorScore );

      // Get right angle flag
      bool isRightTriangle(){ return m_rightAngle; }

      float getScore();
      // Destructor
      ~Detection(){}
   private:
      vector<Point>   m_contact;         // Vertices of object
      vector<Point3f> m_xyz;
      vector<float>   m_distance;        // Distance to each vertex in meters
      vector<float>   m_sideLength;      // Length of each side
      Point           m_centMass;        // Center of mass 
      float           m_area;            // Area of the object
      bool            m_valid;           // valid detection?
      bool            m_rightAngle;      // Is it a right angle?
      unsigned int    m_missCount;       // Number of misses
      float           m_colorScore; 
      float           m_perimeter;          
      unsigned int    m_hitCount;
};

#endif
