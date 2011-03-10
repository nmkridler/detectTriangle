
#include "detection.hpp"
Detection::Detection( vector<Point> &contact): m_contact(contact),
                                               m_valid(false),
                                               m_rightAngle(false){}

// Get distance to vertices
void Detection::getDistance( vector<float> &distance ) const
{ 
   distance = m_distance;
}

// Center of mass
void Detection::getCentMass( Point &centMass ) const
{
   centMass = m_centMass; 
}

// Get Color
void Detection::getColor( Scalar &color ) const 
{ 
   color = m_color; 
}
      
// Get Area
void Detection::getArea( float &area) const
{ 
   area = m_area; 
}

// Get Vertices
void Detection::getVertices( vector<Point> &contact ) const
{ 
   contact = m_contact; 
}

// Get Valid
bool Detection::validDetection() const 
{
   return m_valid;
}

// Get hit count
unsigned int Detection::getMissCount() const 
{
   return m_missCount;
}

// Increment hit count
void Detection::addMiss(){ m_missCount++;}

// Setters
void Detection::setDistance( vector<float> &distance )
{
   m_distance = distance;
   pixelToMetric();
}

// Set Color
void Detection::setColor( Scalar &color ){m_color = color;}

// Set Area
void Detection::setArea( float &area ){ m_area = area; }

// Set center of mass
void Detection::setCentMass( Point &centMass ){ m_centMass = centMass;}

// Set valid
void Detection::setValid( bool valid ){ m_valid = valid; }

// Set hit count to zero
void Detection::resetMissCount(){ m_missCount = 0;}

// Convert Pixel(X,Y),depth to xyz
void Detection::pixelToMetric()
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
void Detection::setRightAngle()
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
void Detection::setSideLength()
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

  
