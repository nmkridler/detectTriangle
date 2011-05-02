
#include "detection.h"
Detection::Detection( vector<Point> &contact): m_contact(contact),
                                               m_area(0),
                                               m_valid(false),
                                               m_rightAngle(false),
                                               m_colorScore(0),
                                               m_perimeter(0),
                                               m_hitCount(0)
{
   // Set the center of mass
   Stats::centerOfMass(m_contact,m_centMass);
}

// Get distance to vertices
void Detection::getDistance( vector<float> &distance ) const
{ 
   distance = m_distance;
}

// Get distance to vertices
void Detection::getLengths( vector<float> &length ) const
{ 
   length = m_sideLength;
}

// Center of mass
void Detection::getCentMass( Point &centMass ) const
{
   centMass = m_centMass; 
}

// Get Vertices
void Detection::getVertices( vector<Point> &contact ) const
{ 
   contact = m_contact; 
}

bool Detection::validDetection() const 
{
   return m_valid;
}

// Get hit count
unsigned int Detection::getMissCount() const 
{
   return m_missCount;
}

unsigned int Detection::getHitCount() const 
{
   return m_hitCount;
}
// Increment hit count
void Detection::addMiss(){ m_missCount++;}

void Detection::addHit()
{
   // Everytime we get a hit, redo the area
   m_hitCount++;
   pixelToMetric();
   setSideLength();
   setRightAngle();
   
}

// Setters
void Detection::setDistance( vector<float> &distance )
{
   m_distance = distance;
   pixelToMetric();
}

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
      Point3f xyzPt(((float)m_contact[idx].x - (float)cx_d)*xFact, 
                    ((float)m_contact[idx].y - (float)cy_d)*yFact,
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
           Point3f v(m_xyz[jdx].x - m_xyz[idx].x,
                     m_xyz[jdx].y - m_xyz[idx].y,
                     m_xyz[jdx].z - m_xyz[idx].z);
           float vNorm = sqrt(v.dot(v));
           v.x /= vNorm;
           v.y /= vNorm;
           v.z /= vNorm;
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

   // Reset the perimeter
   float localPerimeter = 0.0;
   for( unsigned int idx = 0; idx < m_sideLength.size(); ++idx)
      localPerimeter += m_sideLength[idx];
   m_perimeter = localPerimeter;

   // Lengths
#if 0
   cout << "Lengths: ";
   cout << sqrt(u.dot(u)) << " ";
   cout << sqrt(v.dot(v)) << " ";
   cout << sqrt(w.dot(w)) << endl;
#endif

   // We only need two sides to determine the area
   float theta = acos(u.dot(v));
   m_area = m_sideLength[0]*m_sideLength[1]*abs(sin(theta));
   float score = getScore();
//   cout << m_area << "," << m_perimeter << "," << score << endl;
}

// Set the color score
void Detection::setColorScore( float &colorScore )
{
   m_colorScore = colorScore;
}

float Detection::getScore()
{

   float areaScore = abs(m_area - TARGET_AREA_METERS)/TARGET_AREA_METERS;
   float permScore = abs(m_perimeter - TARGET_PERIM)/TARGET_PERIM;
//   if( areaScore < 1.0 && permScore < 1.0)
//   {
//      m_valid = true;
//   }
//   else m_valid = false;
   return areaScore+permScore;
   
}

  
