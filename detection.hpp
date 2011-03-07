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

      Detection( vector<Point> &contact): m_contact(contact),
                                          m_valid(false),
                                          m_rightAngle(false){}

      // Getters
      // Get distance to vertices
      void getDistance( vector<float> &distance ){ distance = m_distance;}

      // Center of mass
      void getCentMass( Point &centMass ){ centMass = m_centMass; }

      // Get Color
      void getColor( Scalar &color ){ color = m_color; }
      
      // Get Area
      void getArea( float &area){ area = m_area; }

      // Get Vertices
      void getVertices( vector<Point> &contact ){ contact = m_contact; }

      // Get Valid
      bool validDetection(){return m_valid;}

      // Get hit count
      unsigned int getMissCount(){return m_missCount;}

      // Increment hit count
      void addMiss(){ m_missCount++;}

      // Setters
      void setDistance( vector<float> &distance )
      {
         m_distance = distance;
         pixelToMetric();
      }

      // Set Color
      void setColor( Scalar &color ){m_color = color;}

      // Set Area
      void setArea( float &area ){ m_area = area; }

      // Set center of mass
      void setCentMass( Point &centMass ){ m_centMass = centMass;}

      // Set valid
      void setValid( bool valid ){ m_valid = valid; }

      // Set hit count to zero
      void resetMissCount(){ m_missCount = 0;}

      // Convert Pixel(X,Y),depth to xyz
      void pixelToMetric()
      {
         // loop through the vertices
         for( unsigned int idx = 0; idx < m_contact.size(); ++idx)
         {
            float xFact = m_distance[idx]/((float)fx_d);
            float yFact = m_distance[idx]/((float)fy_d);
            Point3f xyzPt(xFact*((float)m_contact[idx].x - (float)cx_d), 
                          yFact*((float)m_contact[idx].y - (float)cy_d),
                          m_distance[idx]);
            m_xyz.push_back(xyzPt);
         }
         setRightAngle();
         setSideLength();
      } 

      // Determine if it has a right angle
      void setRightAngle()
      {
         m_rightAngle = false;
         // Calculate the angle for each vertex
         unsigned int idx = 0;
         while( idx < m_contact.size() )
         {
            vector<Point3f> u;
            for( unsigned int jdx = 0; jdx < m_contact.size(); ++jdx)
            {
               if( jdx != idx){
                  //cout << m_xyz[jdx].x - m_xyz[idx].x << " ";
                  //cout << m_xyz[jdx].y - m_xyz[idx].y << " ";
                  //cout << m_xyz[jdx].z - m_xyz[idx].z << endl;
                  Point3f v(m_xyz[jdx].x - m_xyz[idx].x,
                            m_xyz[jdx].y - m_xyz[idx].y,
                            m_xyz[jdx].z - m_xyz[idx].z);
                  float vNorm = sqrt(v.dot(v));
                  v.x /= vNorm;
                  v.y /= vNorm;
                  u.push_back(v);
               }
            }
            // Calculate the angle between the two vectors
            float thetaDiff = abs(acos(u[0].dot(u[1])) - fPi/2.0);
            if( thetaDiff*180.0/fPi < 10.0) m_rightAngle = true;
            ++idx;
         } 
      }

      // Set side lengths
      void setSideLength()
      {
         // Combos: 0,1 - 1,2 - 2,0
         Point3f u(m_xyz[0].x - m_xyz[1].x,
                   m_xyz[0].y - m_xyz[1].y,
                   m_xyz[0].z - m_xyz[1].z);
         
         Point3f v(m_xyz[2].x - m_xyz[1].x,
                   m_xyz[2].y - m_xyz[1].y,
                   m_xyz[2].z - m_xyz[1].z);

         Point3f w(m_xyz[2].x - m_xyz[0].x,
                   m_xyz[2].y - m_xyz[0].y,
                   m_xyz[2].z - m_xyz[0].z);
 
         m_sideLength.push_back(sqrt(u.dot(u)));
         m_sideLength.push_back(sqrt(v.dot(v)));
         m_sideLength.push_back(sqrt(w.dot(w)));

      }

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
