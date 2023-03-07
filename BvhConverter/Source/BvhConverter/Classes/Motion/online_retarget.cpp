#include "online_retarget.h"
#include "constraint.h"

using namespace ml;

OnlineRetarget::OnlineRetarget(void)
{
}

OnlineRetarget::~OnlineRetarget(void)
{
}

void OnlineRetarget::SetJoint( int left_foot, int right_foot )
{
	m_joint[0] = left_foot;
	m_joint[1] = right_foot;
}

void OnlineRetarget::Clear()
{
	for (int i = 0; i < 2; ++i) {
		m_count[i] = 0;
	}
}


void OnlineRetarget::AdjustFeet( Posture *p, const Constraint &c )
{
	for(int i=0; i<2; ++i) {
		if( c.IsConstrained(m_joint[i]) ) {
			if(m_count[i] == m_range) {
				p->IkLimb(m_joint[i], m_lastPos[i]);
			}
			else {
				m_lastPos[i] = p->GetGlobalTranslation(m_joint[i]);
			}
			m_count[i] = m_range;
		}
		else if ( m_count[i] > 0) {
			--m_count[i];
			double t = ((double)m_count[i] / (double)m_range);

			cml::vector3 target = p->GetGlobalTranslation(m_joint[i]);

			target = target * (1.-t) + m_lastPos[i] * t;
			p->IkLimb(m_joint[i], target);
			target = m_lastPos[i];
		}
	}
}