/*
 *  body.h
 *  Bvh Skeleton
 *
 */
#pragma once
#include <map>
#include <set>
#include "ccml.h"

namespace ml{

	enum Channel { XPOS = 1, YPOS, ZPOS, ZROT, XROT, YROT };

	enum JointTag {
		UNKNOWN = -1,
		PELVIS = 0, SPINE, SPINE1, SPINE2, CHEST, NECK, HEAD, HEAD_END,
		L_HIP, L_KNEE, L_ANKLE, L_FOOT, L_TOE, L_TOE_END,
		R_HIP, R_KNEE, R_ANKLE, R_FOOT, R_TOE, R_TOE_END,
		L_CLAVICLE, L_SHOULDER, L_ELBOW, L_WRIST, L_PALM, L_PALM_END,
		R_CLAVICLE, R_SHOULDER, R_ELBOW, R_WRIST, R_PALM, R_PALM_END
	};

	static const int joint_tag_num = 32;

	static const std::string joint_tag_strings[joint_tag_num] =
	{
		"PELVIS", "SPINE", "SPINE1", "SPINE2", "CHEST", "NECK", "HEAD", "HEAD_END",
		"L_HIP", "L_KNEE", "L_ANKLE", "L_FOOT", "L_TOE", "L_TOE_END",
		"R_HIP", "R_KNEE", "R_ANKLE", "R_FOOT", "R_TOE", "R_TOE_END",
		"L_CLAVICLE", "L_SHOULDER", "L_ELBOW", "L_WRIST", "L_PALM", "L_PALM_END",
		"R_CLAVICLE", "R_SHOULDER", "R_ELBOW", "R_WRIST", "R_PALM", "R_PALM_END"
	};

	

	struct Joint
	{
		Joint()
		{
			ik_rigidity_coeff = 1.0;
			ik_damping_coeff = 0.1;
			ik_joint_limit_coeff = 10;
		}

		cml::vector3 offset;
		int parent;
		double ik_rigidity_coeff;
		double ik_damping_coeff;
		double ik_joint_limit_coeff;
	};

	class Body
	{
	public:
		Body();
		Body(const std::vector<Joint>& joints, const std::map<std::string, int>& joint_map);
		~Body();

		size_t num_joint() const { return m_joints.size(); }

		int joint_index(const std::string& name) const;
		std::string joint_name(int joint) const;
		bool IsAncestor(int ancestor_joint, int d_joint) const;

		const std::map<std::string, int>& joint_map() const { return m_jointMap; }
		
		int parent(int joint) const { return m_joints[joint].parent; }
		const cml::vector3& offset(int joint) const { return m_joints[joint].offset; }
		cml::vector3& offset(int joint) { return m_joints[joint].offset; }

		void GetLeafNodes(std::vector<int>& vec) const;
		cml::vector3d GetGlobalTranslation(int joint) const;
		cml::transf GetGlobalTransf(int joint) const;

		// Joint Tags
		void SetPredefinedTagMap(std::string map_name="woody");
		int joint_index(JointTag tag) const;
		JointTag joint_tag(int joint) const;
		void SetJointTag(std::string j_name, JointTag t);
		const std::map<JointTag, int>& joint_tag_map() const { return m_joint_tag_map; }
		bool HasTag(JointTag tag) const;

		const Joint& joint(int j) const { return m_joints[j]; }
		      Joint& joint(int j) { return m_joints[j]; }
		const Joint& joint(JointTag j_tag) const { return m_joints[ m_joint_tag_map.find(j_tag)->second ]; }
		      Joint& joint(JointTag j_tag) { return m_joints[ m_joint_tag_map.find(j_tag)->second ]; }

		int GetDOF() const;
		int GetDOF(int joint) const;

		void Clear() { m_joints.clear(); m_jointMap.clear(); m_joint_tag_map.clear(); }

		std::vector<Joint>& editable_joints() { return m_joints; }
		std::map<std::string, int>& editable_joint_name_map() { return m_jointMap; }
		std::map<JointTag, int>& editable_joint_tag_map() { return m_joint_tag_map; }

		//获取输入时候的BVH名字
		std::string GetJointName_Bvh(int joint) 
		{
			for (const auto& i : m_jointMap)
			{
				if (i.second == joint) 
				return i.first;
			}
			return "UNKNOWN";
		};
		const std::string GetJointName_Bvh(int joint) const
		{
			for (const auto& i : m_jointMap)
			{
				if (i.second == joint)
					return i.first;
			}
			return "UNKNOWN";
		};

		std::vector<int> GetJointChildren(int pjoint);

		const std::vector<int> GetJointChildren(int pjoint) const;

	public:
		//暂存一个T-Pose
		class Posture* T_pose;

		std::vector<Joint> m_joints;
		std::map<std::string, int> m_jointMap;
		std::map<JointTag, int> m_joint_tag_map;
	};


	const std::set<JointTag>& GetEssentialJointTagSet();

	/**
	This function checks the Body (body) has all the essential joint tags.
	The essential joints are required for IK.
	*/
	bool VarifyEssentialJointTags(const Body *body);
};


