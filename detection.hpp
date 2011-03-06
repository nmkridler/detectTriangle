#include <iostream>
#include <vector>
#include <cmath>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace cv;
using namespace std;

class Detection 
{
   public:
      // Constructor
      Detection(){}

      Detection( vector<Point> &contact): m_contact(contact) {} 

      // Getters
      // Get distance to vertices
      void getDistance( Point3f &distance ){ distance = m_distance;}

      // Center of mass
      void getCentMass( Point &centMass ){ centMass = m_centMass; }

      // Get Color
      void getColor( Scalar &color ){ color = m_color; }
      
      // Get Area
      void getArea( float &area){ area = m_area; }

      // Get Vertices
      void getVertices( vector<Point> &contact ){ contact = m_contact; }

      // Setters
      void setDistance( Point3f &distance ){ m_distance = distance;}

      void setColor( Scalar &color ){m_color = color;}

      void setArea( float &area ){ m_area = area; }

      void setCentMass( Point &centMass ){ m_centMass = centMass;}
      // Destructor
      ~Detection(){}
   private:
      vector<Point> m_contact;       // Vertices of object
      Point3f       m_distance;      // Distance to each vertex in meters
      Scalar        m_color;         // Color of the object
      Point3f       m_normalVector;  // Normal to the object
      Point         m_centMass;      // Center of mass 
      float         m_area;          // Area of the object
      

};
