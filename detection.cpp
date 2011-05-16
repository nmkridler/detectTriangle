
#include "detection.h"
Detection::Detection( Point &cMass, float &score ): m_centMass(cMass),
                                                    m_score(score),
                                                    m_missCount(0)
{
}
