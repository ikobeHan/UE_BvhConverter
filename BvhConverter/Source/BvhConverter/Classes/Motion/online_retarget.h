#pragma once

#include "motion.h"
#include "constraint.h"

namespace ml {

class OnlineRetarget
{
public:
	OnlineRetarget(void);
	~OnlineRetarget(void);

	int range() { return m_range; }
	void range(int range) { m_range = range; }

	void Clear();
	void AdjustFeet(Posture *p,  const Constraint &c);
	void SetJoint(int left_foot, int right_foot);

protected:
	int m_range;

	int m_count[2];

	int m_joint[2];

	cml::vector3 m_lastPos[2];
};

}