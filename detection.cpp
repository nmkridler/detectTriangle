
#include "detection.h"

// Constructor for the detection
Detection::Detection( cv::Point const & cMass, 
                      float     const & score ): m_centMass(cMass),
                                                 m_score(score),
                                                 m_missCount(0)
{
}
