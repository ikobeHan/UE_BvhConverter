/*
 *  动画姿势，like Pose in UE
 *  posture.h
 *
 *  Edited by Myung Geol Choi since 2016. 10. 22
 *  Edited by Hanminglu since 2022. 4. 20
 *
 */
#pragma once
#include "ccml.h"
#include "constraint.h"
#include "body.h"

namespace ml 
{
	class Posture
	{
	public:
		Posture();
		Posture(int num_joint);

		// void init_variable();

		const Body* body() const { return m_body; }
		const cml::vector3& trans() const { return m_trans; }
		const cml::matrix3& rotate(int i) const { return m_rotates[i]; }
		const cml::matrix3& rotate(JointTag j_tag) const { return m_rotates[m_body->joint_index(j_tag)]; }

		void body(const Body* body);
		void trans(const cml::vector3& trans) { m_trans = trans; }
		void rotate(int i, const cml::matrix3& rot) { m_rotates[i] = rot; }
		void rotate(JointTag j_tag, const cml::matrix3& rot) { m_rotates[m_body->joint_index(j_tag)] = rot; }

		size_t num_joint() const { return m_rotates.size(); }

		void ApplyTransf(const cml::transf& t);

		void IkFullBodyAnalytic(Constraint const& c);
		void IkFullBody(Constraint const& c);
		void IkLimb(int joint, const cml::vector3& pos, bool reverse = false);
		void IkLimb(int end_joint, int lower_joint, int upper_joint, const cml::vector3& pos, bool reverse = false);
		void IkLimb(JointTag end_j_tag, JointTag lower_j_tag, JointTag upper_j_tag, const cml::vector3& pos, bool reverse = false);
		void IkLimb(JointTag j_tag, const cml::vector3& pos, bool reverse = false);
		void AddDisplacement(cml::vectord const& disp);

		void SwapBody(const Body* tobody);

		void SetIdentityPose();

		void RotateGlobalRotation(int i, const cml::matrix3& rot);
		void RotateGlobalRotation(JointTag t, const cml::matrix3& rot);
		void SetGlobalRotation(int i, const cml::matrix3& rot);
		void SetGlobalRotation(JointTag t, const cml::matrix3& rot);
		void SetGlobalTrans(const cml::vector3d& t);

		cml::transf GetGlobalTransf(int i) const;
		cml::matrix3 GetGlobalRoation(int i) const;
		cml::quaterniond GetGlobalRotationQ(int i) const;
		cml::vector3 GetGlobalTranslation(int i) const;
		cml::transf GetGlobalTransf(JointTag t) const;
		cml::quaterniond GetGlobalRotationQ(JointTag i) const;
		cml::matrix3 GetGlobalRoation(JointTag i) const;
		cml::vector3 GetGlobalTranslation(JointTag t) const;

	public:
		//反向计算bvh关节的LocalSpaceTransform
		//cml::transf RecalculateLocalBvhTransform(JointTag Tag,FMatrix Global) const;

		void PrintRotation();


		const Body* m_body;
		cml::vector3 m_trans;
		//存储每个joint的rotation
		std::vector<cml::matrix3> m_rotates;
		double time;

		/*int type_;
		int object;

		double object_height_correction;
		bool rigid;

		struct Status {
			bool is_highlight;
			cml::vector3 color;
		} status;

		void set_status_in_viewer(bool is_highlight, cml::vector3 color) { status.is_highlight = is_highlight; status.color = color;}*/

		bool leftFootTouched;
		bool rightFootTouched;
		bool is_interact;

	};

	void smoothing(Posture& p_modified, const Posture& p_fixed, const Posture& orig, int time, int range, bool forward);
	cml::vector3 shoulder_orientation(const Posture& pos);



};
