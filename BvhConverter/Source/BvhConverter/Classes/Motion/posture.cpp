/*
 *  posture.cpp
 *
 *  Created by normit on 09. 09. 09.
 *  Edited by Hanminglu since 2022. 4. 20
 */
#include "posture.h"
#include "func.h"
#include <vector>
using namespace cml;

namespace ml
{
	static double ACOS(double x)
	{
		if (x > 1.0) return 0;
		else if (x < -1.0) return pi();
		else return std::acos(x);
	}

	Posture::Posture() : leftFootTouched(false), rightFootTouched(false) {
		m_body = nullptr;
	}

	Posture::Posture(int num_joint) : leftFootTouched(false), rightFootTouched(false) {
		m_rotates.resize(num_joint);
		m_body = nullptr;
	}

	//void  Posture::init_variable() {
	//  type_ = -1;
	//  object = 0;
	//  object_height_correction = 0.0;
	//  rigid = false;
	//  status.is_highlight = false;
	//  status.color = cml::vector3(0., 0., 0.);
	//  is_interact = false;
	//}

	void  Posture::SwapBody(const Body *tobody)
	{
		std::vector<cml::matrix3> new_rotates;
		for (int i = 0; i < (int)tobody->num_joint(); ++i) {
			int index = m_body->joint_index(tobody->joint_name(i));
			if (index < 0 || index >= (int)m_body->num_joint()) {
				new_rotates.push_back(cml::identity_3x3());
			}
			else {
				new_rotates.push_back(m_rotates[index]);
			}
		}
		m_rotates = new_rotates;
		m_body = tobody;
	}

	void Posture::SetIdentityPose()
	{
		m_trans.set(0, 0, 0);
		for (auto &d : m_rotates)
		{
			d.identity();
		}
	}

	void Posture::body(const Body *body)
	{
		m_body = body;
		if (body) m_rotates.resize(body->num_joint());
	}

	void Posture::ApplyTransf(const cml::transf& t)
	{
		cml::transf new_t = t * GetGlobalTransf(0);
		m_rotates[0] = mat3(new_t);
		m_trans = cml::trans(new_t);
	}


	cml::transf Posture::GetGlobalTransf(int i) const
	{
		transf f;
		f.identity();

		while (i) {
			f = make_transf(m_rotates[i], m_body->offset(i)) * f;
			i = m_body->parent(i);
		}
		f = make_transf(m_rotates[0], trans()+m_body->offset(0)) * f;
		return f;
	}

	cml::matrix3 Posture::GetGlobalRoation(int i) const
	{
		return cml::mat3(GetGlobalTransf(i));
	}

	cml::quaterniond Posture::GetGlobalRotationQ(int i) const
	{
		quaterniond q;
		cml::quaternion_rotation_matrix(q, GetGlobalTransf(i));
		return q;
	}

	cml::vector3 Posture::GetGlobalTranslation(int i) const
	{
		return cml::trans(GetGlobalTransf(i));
	}

	cml::transf Posture::GetGlobalTransf(JointTag t) const
	{
		if (!m_body->HasTag(t))
		{
			std::cerr << "There's no tag, " << t << "- Posture::GetGlobalTransf(JointTag t)" << std::endl;
			exit(0);
		}
		return GetGlobalTransf(m_body->joint_index(t));
	}

	cml::matrix3 Posture::GetGlobalRoation(JointTag t) const
	{
		if (!m_body->HasTag(t))
		{
			std::cerr << "There's no tag, " << t << "- Posture::GetGlobalTranslation(JointTag t)" << std::endl;
			exit(0);
		}


		return GetGlobalRoation(m_body->joint_index(t));
	}

	cml::quaterniond Posture::GetGlobalRotationQ(JointTag t) const
	{
		if (!m_body->HasTag(t))
		{
			std::cerr << "There's no tag, " << t << "- Posture::GetGlobalTranslation(JointTag t)" << std::endl;
			exit(0);
		}
		return GetGlobalRotationQ(m_body->joint_index(t));

	}

	cml::vector3 Posture::GetGlobalTranslation(JointTag t) const
	{
		if (!m_body->HasTag(t))
		{
			std::cerr << "There's no tag, " << t << "- Posture::GetGlobalTranslation(JointTag t)" << std::endl;
			exit(0);
		}
		return cml::trans(GetGlobalTransf(m_body->joint_index(t)));
	}

	void Posture::PrintRotation()
	{
		UE_LOG(LogTemp, Warning, TEXT("----------------------------------------------"));
		//std::vector<cml::matrix3> m_rotates;
		for (cml::matrix3 m : m_rotates) {
			
			for (int i = 0;i<3;i++)
			{
				for (int j=0;j<3;j++)
				{
					double v = m.basis_element(i,j);
					UE_LOG(LogTemp, Warning, TEXT("%f,"),v);
				}
				UE_LOG(LogTemp, Warning, TEXT("    "));
			}
			UE_LOG(LogTemp, Warning, TEXT("============ \n"));
		}
		UE_LOG(LogTemp, Warning, TEXT("----------------------------------------------"));
	}


	//cml::transf Posture::RecalculateLocalBvhTransform(JointTag Tag, FMatrix Global) const
	//{
	//	if (!m_body->HasTag(Tag))
	//	{
	//		std::cerr << "There's no tag, " << Tag << "- Posture::GetGlobalTransf(JointTag t)" << std::endl;
	//		return cml::transf();
	//	}
	//	//int index = m_body->joint_index(Tag);
	//	//int ParentIndex = m_body->parent(index);
	//	//if (ParentIndex == -1)
	//	//{

	//	//}
	//}



	void Posture::IkLimb(JointTag j_tag, const cml::vector3& pos, bool reverse)
	{
		if (j_tag == L_WRIST || j_tag == L_PALM) IkLimb(j_tag, L_ELBOW, L_SHOULDER, pos, reverse);
		if (j_tag == R_WRIST || j_tag == R_PALM) IkLimb(j_tag, R_ELBOW, R_SHOULDER, pos, reverse);
		if (j_tag == L_ANKLE || j_tag == L_FOOT) IkLimb(j_tag, L_KNEE, L_HIP, pos, reverse);
		if (j_tag == R_ANKLE || j_tag == R_FOOT) IkLimb(j_tag, R_KNEE, R_HIP, pos, reverse);
		else
			IkLimb(body()->joint_index(j_tag), pos, reverse);
	}

	void Posture::IkLimb(int joint, const cml::vector3& pos, bool reverse)
	{
		int lower = body()->parent(joint);
		int upper = body()->parent(lower);

		IkLimb(joint, lower, upper, pos, reverse);
	}

	void Posture::IkLimb(JointTag end_j_tag, JointTag lower_j_tag, JointTag upper_j_tag, const cml::vector3& pos, bool reverse)
	{
		IkLimb(
			body()->joint_index(end_j_tag),
			body()->joint_index(lower_j_tag),
			body()->joint_index(upper_j_tag),
			pos,
			reverse);
	}

	void Posture::IkLimb(int end_joint, int lower_joint, int upper_joint, const cml::vector3& pos, bool reverse)
	{
		int joint = end_joint;
		int lower = lower_joint;
		int upper = upper_joint;

		vector3 B = GetGlobalTranslation (joint);
		vector3 C = GetGlobalTranslation (lower);
		vector3 A = GetGlobalTranslation (upper);

		//error 贸府
		if ((B - pos).length() < 0.000001)
			return;

		vector3 L = B - A;
		vector3 N = B - C;
		vector3 M = C - A;

		double l = L.length();
		double n = N.length();
		double m = M.length();

		double a = ACOS((l*l + n * n - m * m) / (2 * l*n));
		double b = ACOS((l*l + m * m - n * n) / (2 * l*m));

		vector3 B_new = pos;
		vector3 L_new = B_new - A;

		double l_ = L_new.length();

		double a_ = ACOS((l_*l_ + n * n - m * m) / (2 * l_*n));
		double b_ = ACOS((l_*l_ + m * m - n * n) / (2 * l_*m));

		//迫捞 檬扁蔼锭巩俊 馆措肺 勃捞绰 版快啊 积扁搁 捞甫 馆措肺 荐沥
		if (reverse == true)
		{
			a_ *= -1.0;
			b_ *= -1.0;
		}

		vector3 rotV = normalize(cross(M, L));

		double rotb = b - b_;
		double rota = a_ - a - rotb;

		//乞搁惑俊辑 雀傈秦辑 嘎眠扁
		RotateGlobalRotation(upper, exp_mat3(rotV * rotb));
		RotateGlobalRotation(lower, exp_mat3(rotV * rota));

		////乞搁观俊辑 雀傈秦辑 嘎眠扁
		vector3 rotV2 = normalize(cross(L, L_new));

		double l_new = L_new.length();
		double l_diff = (L_new - L).length();

		double rot2 = ACOS((l_new * l_new + l * l - l_diff * l_diff) / (2.0 * l_new * l));
		RotateGlobalRotation(upper, exp_mat3(rotV2 * rot2));
	}

	void Posture::SetGlobalRotation(int i, const cml::matrix3& rot)
	{
		if ( i>0 )
		{
			matrix3 parent_gloal = GetGlobalRoation(m_body->parent(i));
			m_rotates[i] = transpose(parent_gloal) * rot;
		}
		else
		{
			m_rotates[i] = rot;
		}
	}

	void Posture::SetGlobalRotation(JointTag t, const cml::matrix3& rot)
	{
		SetGlobalRotation(m_body->joint_index(t), rot);
	}

	void Posture::SetGlobalTrans(const cml::vector3d & t)
	{
		trans(t - m_body->offset(0));
	}

	void Posture::RotateGlobalRotation(int i, const cml::matrix3 &rot)
	{
		SetGlobalRotation(i, rot * GetGlobalRoation(i));
	}

	void Posture::RotateGlobalRotation(JointTag t, const cml::matrix3 &rot)
	{
		RotateGlobalRotation(m_body->joint_index(t), rot);
	}

	void  smoothing(Posture& p_modified, const Posture& p_fixed, const Posture& orig, int time, int range, bool forward)
	{
		std::pair<cml::vector3d, std::vector<cml::vector3d>> tmp_d;
		if (forward) {
			add_difference(p_modified, tmp_d = difference(p_fixed, orig), scalarTransitionFunc(static_cast<double>(time), static_cast<double>(range)));
		}
		else {
			add_difference(p_modified, tmp_d = difference(p_fixed, orig), 1.0 - scalarTransitionFunc(static_cast<double>(time), static_cast<double>(range)));
		}
	}

	cml::vector3  shoulder_orientation(const Posture &pos)
	{
		cml::vector3 right_shoulder(pos.GetGlobalTranslation(12));
		cml::vector3 left_shoulder(pos.GetGlobalTranslation(8));
		cml::vector3 head(pos.GetGlobalTranslation(7));
		return plane_vec3(cross(left_shoulder - right_shoulder, head - right_shoulder)).normalize();
	}





}
