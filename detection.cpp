
#include "detection.h"
Detection::Detection( vector<Point> &contact): m_contact(contact),
                                               m_valid(false),
                                               m_rightAngle(false),
                                               m_areaSum(0),
                                               m_hitCount(0){}

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
// Mean
void Detection::getMean( Scalar &meanVal ) const
{
   meanVal = m_mean;
}

// Standard deviation
void Detection::getStdDev( Scalar &stdVal ) const
{
   stdVal = m_stddev;
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
float Detection::getArea() 
{
   float area = 0.0;
   area = m_areaSum;
   if( m_hitCount > 0){
      area /= ((float)m_hitCount);
   }
   
   setArea(area);
   return area;
   
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

unsigned int Detection::getHitCount() const 
{
   return m_hitCount;
}
// Increment hit count
void Detection::addMiss(){ m_missCount++;}

void Detection::addHit(){ m_hitCount++;}

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

   // We only need two sides to determine the area
   float theta = acos(u.dot(v));
   m_areaSum += m_sideLength[0]*m_sideLength[1]*abs(sin(theta));
}

// Set the mean
void Detection::setMean( Scalar &meanVal )
{
   m_mean = meanVal;
}

// Set the standard deviation
void Detection::setStdDev( Scalar &stdVal )
{
   m_stddev = stdVal;
}

float Detection::getScore()
{
   float colorScore = 0;
   colorScore += (float)m_stddev[0];
   colorScore += (float)m_stddev[1];
   colorScore += (float)m_stddev[2];
   colorScore += abs((float)(m_mean[0] - TARGET_COLOR[0]));
   colorScore += abs((float)(m_mean[1] - TARGET_COLOR[1]));
   colorScore += abs((float)(m_mean[2] - TARGET_COLOR[2]));

   m_area = getArea();
   float areaScore = abs(m_area - TARGET_AREA_METERS)/TARGET_AREA_METERS;
   //cout << "area: " << m_area << endl;
   return areaScore*colorScore;
   
}

  
