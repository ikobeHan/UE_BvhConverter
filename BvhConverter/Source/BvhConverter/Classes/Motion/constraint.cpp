#include "constraint.h"

using namespace ml;
using namespace cml;

bool Constraint::IsConstrained( int joint ) const
{
  for (size_t i = 0; i < m_entity.size(); ++i) {
    if (joint == m_entity[i].joint) return true;
  }
  return false;
}

const ConstraintEntity* 
Constraint::GetConstraintEntity(int joint) const
{
	for (size_t i = 0; i < m_entity.size(); ++i) {
		if (joint == m_entity[i].joint) return &m_entity[i];
	}

	return nullptr;
}


void Constraint::Push( int joint, ConstraintMask m, const cml::transf &t )
{
  ConstraintEntity e;

  e.joint = joint;
  e.mask = m;
  e.value = t;
  m_entity.push_back(e);
} 

void Constraint::Push( int joint, const cml::vector3 &v )
{
  ConstraintEntity e;

  e.joint = joint;
  e.mask = C_POSITION;
  e.value = make_transf(identity_mat(), v);
  m_entity.push_back(e);
}

void Constraint::Push( int joint, const cml::matrix3 &m )
{
  ConstraintEntity e;

  e.joint = joint;
  e.mask = C_ORIENTATION;
  e.value = make_transf(m, vector3(0, 0, 0));
  m_entity.push_back(e);
}

void Constraint::Push( int joint, const cml::transf &t )
{
  ConstraintEntity e;

  e.joint = joint;
  e.mask = C_TRANSF;
  e.value = t;
  m_entity.push_back(e);
}

void ml::Constraint::ApplyTransf( const cml::transf& t )
{
  for (size_t i = 0; i < m_entity.size(); ++i) {
    m_entity[i].ApplyTransf(t);
  }
}