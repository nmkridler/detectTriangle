#ifndef __DETECTION_HPP__
#define __DETECTION_HPP__

#include <iostream>
#include <vector>
#include <cmath>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "constants.hpp"

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

      // Get Color
      void getColor( Scalar &color ) const;
      
      // Get Area
      void getArea( float &area) const;

      // Get Vertices
      void getVertices( vector<Point> &contact ) const;

      // Get Valid
      bool validDetection() const;

      // Get hit count
      unsigned int getMissCount() const;

      // Increment hit count
      void addMiss();

      // Setters
      void setDistance( vector<float> &distance );
    
      // Set Color
      void setColor( Scalar &color );

      // Set Area
      void setArea( float &area );

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
     
      // Get right angle flag
      bool isRightTriangle(){ return m_rightAngle; }
      // Destructor
      ~Detection(){}
   private:
      vector<Point>   m_contact;         // Vertices of object
      vector<Point3f> m_xyz;
      vector<float>   m_distance;        // Distance to each vertex in meters
      vector<float>   m_sideLength;      // Length of each side
      Scalar          m_color;           // Color of the object
      Point3f         m_normalVector;    // Normal to the object
      Point           m_centMass;        // Center of mass 
      float           m_area;            // Area of the object
      bool            m_valid;           // valid detection?
      bool            m_rightAngle;      // Is it a right angle?
      unsigned int    m_missCount;       // Number of misses 

};

#endif
