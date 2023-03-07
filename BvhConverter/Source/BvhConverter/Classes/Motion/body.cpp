/*
*  body.cpp
*  unagi
*
*  Created by normit on 09. 09. 09.
*  Copyright 2009 __MyCompanyName__. All rights reserved.
*
*/


#include "body.h"
#include <map>
#include <set>

using namespace ml;


std::set<JointTag> g_essential_joint_tags;

static void InitEssentialJointTagSet()
{
	g_essential_joint_tags.insert(PELVIS);

	// Torso
	g_essential_joint_tags.insert(SPINE);
	g_essential_joint_tags.insert(CHEST);
	g_essential_joint_tags.insert(HEAD);

	// Left Arm
	g_essential_joint_tags.insert(L_SHOULDER);
	g_essential_joint_tags.insert(L_ELBOW);
	g_essential_joint_tags.insert(L_WRIST);

	// Right Arm
	g_essential_joint_tags.insert(R_SHOULDER);
	g_essential_joint_tags.insert(R_ELBOW);
	g_essential_joint_tags.insert(R_WRIST);

	// Left Leg
	g_essential_joint_tags.insert(L_HIP);
	g_essential_joint_tags.insert(L_KNEE);
	g_essential_joint_tags.insert(L_ANKLE);

	// Right Leg
	g_essential_joint_tags.insert(R_HIP);
	g_essential_joint_tags.insert(R_KNEE);
	g_essential_joint_tags.insert(R_ANKLE);
}

const std::set<JointTag>& ml::GetEssentialJointTagSet()
{
	// Assign the values at the first time only.
	if ( g_essential_joint_tags.empty() )
	{
		InitEssentialJointTagSet();
	}

	return g_essential_joint_tags;
}


bool ml::VarifyEssentialJointTags(const Body *body)
{
	// Assign the values at the first time only.
	if ( g_essential_joint_tags.empty() )
	{
		InitEssentialJointTagSet();
	}

	// Check
	bool check = true;
	for ( auto &d : g_essential_joint_tags )
	{
		if ( !body->HasTag(d) )
		{
			std::cerr << "A body does not have the essential joint tag, " << joint_tag_strings[d] << "." << std::endl;
			check = false;
		}
	}

	return check;
}

Body::Body()
{

}

Body::Body(const std::vector<Joint>& joints, const std::map<std::string, int>& joint_map)
{
	m_joints.assign(joints.begin(), joints.end());
	m_jointMap.insert(joint_map.begin(), joint_map.end());
}

Body::~Body()
{

}

int Body::joint_index( const std::string& name ) const
{
	std::map<std::string,int>::const_iterator it = m_jointMap.find(name);
	if(it != m_jointMap.end())
		return it->second;
	else 
		return -1;
}

int Body::joint_index( JointTag tag ) const
{
	auto i = m_joint_tag_map.find(tag);;
	if(i != m_joint_tag_map.end()) return i->second;
	else return -1;
}

void ml::Body::GetLeafNodes( std::vector<int>& vec ) const
{
	vec.clear();
	std::vector<int> checked(m_joints.size(),0);
	for(size_t i = 1; i < m_joints.size(); ++i) {
		checked[m_joints[i].parent] = 1;
	}
	for(size_t i = 0; i < m_joints.size(); ++i) {
		if(!checked[i]) vec.push_back((int)i);
	}
}

cml::vector3d ml::Body::GetGlobalTranslation(int joint) const
{
	cml::vector3d f(0., 0., 0.);

	int i= joint;
	while(1) {
		f = offset(i) + f;
		if ( i==0 ) break;
		i = parent(i);
	}
	
	return f;
}

cml::transf ml::Body::GetGlobalTransf(int joint) const
{
	cml::transf t;
	t.identity();
	cml::matrix_translation(t, GetGlobalTranslation(joint));

	return t;
}

std::string ml::Body::joint_name( int joint ) const
{
	for(auto it = joint_map().begin(); it != joint_map().end(); ++it) {
		if(it->second == joint) return it->first;
	}
	return "Unknown";
}


// JointTag auto-fill
void ml::Body::SetPredefinedTagMap(std::string map_name)
{
	std::string lower_map_name;
	std::transform(map_name.begin(), map_name.end(), lower_map_name.begin(), std::tolower);
	
	if ( map_name.compare("woody") ==  0 )
	{
		m_joint_tag_map[ml::JointTag::PELVIS] = 0;

		m_joint_tag_map[ml::JointTag::SPINE]      = m_jointMap["Spine"];
		m_joint_tag_map[ml::JointTag::SPINE1]     = -1;
		m_joint_tag_map[ml::JointTag::SPINE2]     = -1;
		m_joint_tag_map[ml::JointTag::CHEST]      = m_jointMap["Spine1"];
		m_joint_tag_map[ml::JointTag::NECK]       = -1;
		m_joint_tag_map[ml::JointTag::HEAD]       = m_jointMap["HEad"];
		m_joint_tag_map[ml::JointTag::HEAD_END]   = m_jointMap["HEadEnd"];
												  
		m_joint_tag_map[ml::JointTag::L_HIP]      = m_jointMap["LeftUpLeg"];
		m_joint_tag_map[ml::JointTag::L_KNEE]     = m_jointMap["LeftLeg"];
		m_joint_tag_map[ml::JointTag::L_ANKLE]    = m_jointMap["LeftFoot"];
		m_joint_tag_map[ml::JointTag::L_FOOT]     = m_jointMap["LeftToes"];
		m_joint_tag_map[ml::JointTag::L_TOE]      = -1;
		m_joint_tag_map[ml::JointTag::L_TOE_END]  = m_jointMap["LeftToesEnd"];
												  
		m_joint_tag_map[ml::JointTag::R_HIP]      = m_jointMap["RightUpLeg"];
		m_joint_tag_map[ml::JointTag::R_KNEE]     = m_jointMap["RightLeg"];
		m_joint_tag_map[ml::JointTag::R_ANKLE]    = m_jointMap["RightFoot"];
		m_joint_tag_map[ml::JointTag::R_FOOT]     = m_jointMap["RightToes"];
		m_joint_tag_map[ml::JointTag::R_TOE]      = -1;
		m_joint_tag_map[ml::JointTag::R_TOE_END]  = m_jointMap["RightToesEnd"];

		m_joint_tag_map[ml::JointTag::L_CLAVICLE] = m_jointMap["LeftShoulder1"];
		m_joint_tag_map[ml::JointTag::L_SHOULDER] = m_jointMap["LeftArm"];
		m_joint_tag_map[ml::JointTag::L_ELBOW]    = m_jointMap["LeftForeArm"];
		m_joint_tag_map[ml::JointTag::L_WRIST]    = m_jointMap["LeftHand"];
		m_joint_tag_map[ml::JointTag::L_PALM]     = -1;
		m_joint_tag_map[ml::JointTag::L_PALM_END] = m_jointMap["LeftHandEnd"];

		m_joint_tag_map[ml::JointTag::R_CLAVICLE] = m_jointMap["RightShoulder"];
		m_joint_tag_map[ml::JointTag::R_SHOULDER] = m_jointMap["RightArm"];
		m_joint_tag_map[ml::JointTag::R_ELBOW]    = m_jointMap["RightForeArm"];
		m_joint_tag_map[ml::JointTag::R_WRIST]    = m_jointMap["RightHand"];
		m_joint_tag_map[ml::JointTag::R_PALM]     = -1;
		m_joint_tag_map[ml::JointTag::R_PALM_END] = m_jointMap["RightHandEnd"];
	}

	else if ( map_name.compare("cmu_bvh") ==  0 )
	{
		m_joint_tag_map[ml::JointTag::PELVIS] = 0;

		m_joint_tag_map[ml::JointTag::SPINE]      = m_jointMap["Spine"];
		m_joint_tag_map[ml::JointTag::SPINE1]     = -1;
		m_joint_tag_map[ml::JointTag::SPINE2]     = -1;
		m_joint_tag_map[ml::JointTag::CHEST]      = m_jointMap["Spine1"];
		m_joint_tag_map[ml::JointTag::NECK]       = m_jointMap["Neck"];
		m_joint_tag_map[ml::JointTag::HEAD]       = m_jointMap["Head"];
		m_joint_tag_map[ml::JointTag::HEAD_END]   = m_jointMap["HeadEnd"];

		m_joint_tag_map[ml::JointTag::L_HIP]      = m_jointMap["LeftUpLeg"];
		m_joint_tag_map[ml::JointTag::L_KNEE]     = m_jointMap["LeftLeg"];
		m_joint_tag_map[ml::JointTag::L_ANKLE]    = m_jointMap["LeftFoot"];
		m_joint_tag_map[ml::JointTag::L_FOOT]     = m_jointMap["LeftToeBase"];
		m_joint_tag_map[ml::JointTag::L_TOE]      = -1;
		m_joint_tag_map[ml::JointTag::L_TOE_END]  = m_jointMap["LeftToeBaseEnd"];

		m_joint_tag_map[ml::JointTag::R_HIP]      = m_jointMap["RightUpLeg"];
		m_joint_tag_map[ml::JointTag::R_KNEE]     = m_jointMap["RightLeg"];
		m_joint_tag_map[ml::JointTag::R_ANKLE]    = m_jointMap["RightFoot"];
		m_joint_tag_map[ml::JointTag::R_FOOT]     = m_jointMap["RightToeBase"];
		m_joint_tag_map[ml::JointTag::R_TOE]      = -1;
		m_joint_tag_map[ml::JointTag::R_TOE_END]  = m_jointMap["RightToeBaseEnd"];

		m_joint_tag_map[ml::JointTag::L_CLAVICLE] = m_jointMap["LeftShoulder"];
		m_joint_tag_map[ml::JointTag::L_SHOULDER] = m_jointMap["LeftArm"];
		m_joint_tag_map[ml::JointTag::L_ELBOW]    = m_jointMap["LeftForeArm"];
		m_joint_tag_map[ml::JointTag::L_WRIST]    = m_jointMap["LeftHand"];
		m_joint_tag_map[ml::JointTag::L_PALM]     = -1;
		m_joint_tag_map[ml::JointTag::L_PALM_END] = m_jointMap["LeftHandIndex1End"];

		m_joint_tag_map[ml::JointTag::R_CLAVICLE] = m_jointMap["RightShoulder"];
		m_joint_tag_map[ml::JointTag::R_SHOULDER] = m_jointMap["RightArm"];
		m_joint_tag_map[ml::JointTag::R_ELBOW]    = m_jointMap["RightForeArm"];
		m_joint_tag_map[ml::JointTag::R_WRIST]    = m_jointMap["RightHand"];
		m_joint_tag_map[ml::JointTag::R_PALM]     = -1;
		m_joint_tag_map[ml::JointTag::R_PALM_END] = m_jointMap["RightHandIndex1End"];
	}
}


void ml::Body::SetJointTag(std::string j_name, JointTag t)
{
	if ( m_jointMap.count(j_name) > 0 )
	{
		m_joint_tag_map[t] = m_jointMap[j_name];
	}
}


JointTag ml::Body::joint_tag( int joint ) const
{
	for (const auto &i : m_joint_tag_map)
	{
		if ( i.second == joint ) return i.first;
	}

	return UNKNOWN;
}

bool ml::Body::HasTag( ml::JointTag tag ) const
{
	auto t = m_joint_tag_map.find(tag);
	
	if ( t == m_joint_tag_map.end() ) return false;
	if ( t->second == -1 ) return false;

	return true;

}

bool ml::Body::IsAncestor(int ancestor_joint, int d_joint) const
{
	while (  d_joint >= 0 )
	{
		if ( ancestor_joint==d_joint ) return true;
		d_joint = parent(d_joint);
	}

	return false;
}




int ml::Body::GetDOF() const
{
	// In the current implementation, 
	// we assume that all the joints are Ball-and-Socket joints.
	// only the root (PELVIS) has 3D-translation dof. 
	return num_joint() * 3 + 3;
}

int ml::Body::GetDOF(int joint) const
{
	// In the current implementation, 
	// we assume that all the joints are Ball-and-Socket joints.
	// only the root (PELVIS) has 3D-translation dof. 

	if ( joint == 0 ) return 6;

	return 3;
}

std::vector<int> ml::Body::GetJointChildren(int pjoint)
{
	std::vector<int> pV;
	for (int i = 0; i < m_joints.size(); ++i)
	{
		if (parent(i) == pjoint)
		{
			pV.push_back(i);
		}
	}
	return pV;
}

const std::vector<int> ml::Body::GetJointChildren(int pjoint) const
{
	std::vector<int> pV;
	for (int i = 0; i < m_joints.size(); ++i)
	{
		if (parent(i) == pjoint)
		{
			pV.push_back(i);
		}
	}
	return pV;
}

