#pragma once 
#include "ccml.h"

namespace ml {

	enum ConstraintMask {
		C_UNCONSTRAINED = 0x00,
		C_POSITION = 0x01,
		C_ORIENTATION = 0x02,
		C_TRANSF = 0x03,
	};

	struct ConstraintEntity {
		ConstraintMask mask;
		int joint;
		cml::transf value;

		ConstraintEntity() {
			mask = C_UNCONSTRAINED;
			joint = -1;
		}

		void ApplyTransf(const cml::transf &t) {
			value *= t;
		}


		int	GetDOC() 
		{ 
			if (mask== C_POSITION) return 3;
			else if (mask==C_ORIENTATION) return 3;
			else if (mask==C_TRANSF) return 6;
			//else if (mask==PM_C_COG) return 3;
			//else if (mask==PM_C_BALANCE) return 2;
			else return 0;
		}
	};

	class Constraint {
	public:
		void Clear() { m_entity.clear(); }
		size_t num_entity() const { return m_entity.size(); }

		void ApplyTransf(const cml::transf& t);

		bool IsConstrained(int joint) const;
		const ConstraintEntity* GetConstraintEntity(int joint) const;

		void Push( int joint, ConstraintMask m, const cml::transf &t);
		void Push( int joint, const cml::vector3 &v);
		void Push( int joint, const cml::matrix3 &m);
		void Push( int joint, const cml::transf &t);
		const std::vector<ConstraintEntity> & entities() const { return m_entity; }

		int	GetDOC() 
		{
			int doc=0;

			for( auto &d : m_entity )
				doc += d.GetDOC();

			return doc;
		}

	protected:
		std::vector<ConstraintEntity> m_entity;
	};
}