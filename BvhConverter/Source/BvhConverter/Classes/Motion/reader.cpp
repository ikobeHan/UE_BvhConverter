/*
 *  reader.cpp
 *  unagi
 *
 *  Created by normit on 09. 09. 10.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 *  Edited by Myung Geol Choi sice 2016. 08. 12
 *  Edited by Hanminglu  sice 2022. 04. 12
 */

#include "reader.h"
#include "motion.h"
#include <stack>
#include <fstream>
#include <string>
#include <sstream>
#include "body.h"

using std::string;
using std::stringstream;

using namespace cml;
namespace ml
{
	static std::string& Trim(std::string &str)
	{
		//some bvh file joint has num index,like "#6" 
		size_t ext_index = str.find("#");
		if (std::string::npos != ext_index)
		{
			str = str.substr(0, ext_index);
		}
		// Trim Both leading and trailing spaces  
		size_t startpos = str.find_first_not_of(" \r\n\t");
		size_t endpos = str.find_last_not_of(" \r\n\t");

		if (std::string::npos == startpos) startpos = 0;
		if (std::string::npos == endpos) endpos = str.length() - 1;
		str = str.substr(startpos, endpos - startpos + 1);

		return str;
	}

	void BVHReader::AddChannel(int i, Channel ch)
	{
		m_channels[i].push_back(ch);
	}

	int BVHReader::NewNode(const std::string &name, int parent)
	{
		int new_index = (int)m_joints.size();

		Joint joint;
		joint.parent = parent;

		m_joints.push_back(joint);
		if (name != "DUMMY") m_jointMap[name] = new_index;

		m_channels.push_back(std::vector<Channel>());

		return new_index;
	}

	bool LoadBVH(const char *file, Motion &out_motion, bool root_offset, bool human_load, double scale, int sample)
	{
		BVHReader r;
		r.LoadBVH(file, &out_motion, root_offset, human_load, scale, sample);

		return true;
	}



	bool LoadBVH_UE4(const char *file, Motion &out_motion, bool root_offset, bool human_load, double scale, int sample)
	{
		//load step 2

		BVHReader r;
		r.LoadBVH_UE4(file, &out_motion, root_offset, human_load, scale, sample);

		return true;
	}

	bool LoadBVH_UE4(FString T_motion_file, FString file, Motion &out_motion, bool root_offset, bool human_load, double scale, int sample)
	{
		//load step 1
		bool succeed = true;

		ml::Motion t_motion;
		

		succeed = LoadBVH_UE4(TCHAR_TO_ANSI(*T_motion_file), t_motion, root_offset);
		if ( !succeed ) return false;

		const ml::Posture &t_pose = t_motion.posture(0);

		

		succeed = LoadBVH_UE4(TCHAR_TO_ANSI(*file), out_motion, root_offset);
		if ( !succeed ) 
			return false;

		//暂存Tpose
		out_motion.editable_body()->T_pose = new Posture(t_motion.posture(0));

		int joint_num = out_motion.body()->num_joint();

		for ( int j=0; j<joint_num; j++ )
		{
			//if ( j==0 )
			//{
			//	out_motion.editable_body()->joint(j).offset = cml::vector3d(0, 0, 0);//t_pose.GetGlobalTranslation(0);
			//}
			//else
			{
				//int p_j = t_pose.body()->parent(j);
				out_motion.editable_body()->joint(j).offset = t_pose.rotate(j) * out_motion.editable_body()->joint(j).offset;
			}
		}


		ml::Motion src_motion;
		succeed = LoadBVH_UE4(TCHAR_TO_ANSI(*file), src_motion, root_offset);
		if ( !succeed ) return false;


		for ( int f=0; f<out_motion.size(); f++ )
		{
			for ( int j=0; j<joint_num; j++ )
			{
				//最终motion数据存的都是全局的rotation
				out_motion.posture(f).SetGlobalRotation(j, src_motion.posture(f).GetGlobalRoation(j) * cml::inverse(t_pose.GetGlobalRoation(j)));
				//out_motion.posture(f).rotate(j, out_motion.posture(f).GetGlobalRoation(j));
			}
		}

		return succeed;
	}


	void BVHReader::LoadBVH_UE4(const char *file, Motion *motion, bool root_offset, bool human_load, double scale, int sample)
	{
		//load step 3
		
		//cml::matrix3 convert_unreal_m(1, 0, 0, 0, 0, 1, 0, 1, 0);
		cml::matrix3 convert_unreal_m(
			1, 0, 0, 
			0, 0, 1, 
			0, 1, 0); 

		cml::matrix3 convert_unreal_m_inv = convert_unreal_m;
		convert_unreal_m_inv.inverse();


		std::ifstream in;
		std::string bufstr;

		m_channels.clear();
		m_joints.clear();
		m_jointMap.clear();
		in.open(file);

		std::stack<int> nodes;
		nodes.push(-1);
		int cur_node = -1;
		int num_channels = 0;

		std::string last_joint_name;

		while (in) { //遍历读取关键单词
			in >> bufstr; //每次读取一段字符串，空格为结束。

			if (bufstr == "ROOT") {
				std::getline(in, bufstr);
				Trim(bufstr);

				cur_node = NewNode(bufstr, -1);
				continue;
			}
			if (bufstr == "OFFSET") {
				double x, y, z;
				in >> x >> y >> z;
				x *= scale;
				y *= scale;
				z *= scale;
				m_joints[cur_node].offset = convert_unreal_m * cml::vector3(x, y, z); //??右手系转左手系？？
				
				continue;
			}
			if (bufstr == "CHANNELS") {
				int n;
				in >> n;
				num_channels += n;
				for (int i = 0; i < n; ++i) {
					in >> bufstr;
					if (bufstr == "Xrotation") AddChannel(cur_node, XROT);
					if (bufstr == "Yrotation") AddChannel(cur_node, YROT);
					if (bufstr == "Zrotation") AddChannel(cur_node, ZROT);

					if (bufstr == "Xposition") AddChannel(cur_node, XPOS);
					if (bufstr == "Yposition") AddChannel(cur_node, YPOS);
					if (bufstr == "Zposition") AddChannel(cur_node, ZPOS);
				}
				continue;
			}
			if (bufstr == "JOINT") {
				std::getline(in, bufstr);
				Trim(bufstr);
				last_joint_name = bufstr;
				cur_node = NewNode(bufstr, cur_node);
				continue;
			}
			if (bufstr == "END" || bufstr == "End") {//hanminglu: 跳过end site
				std::getline(in, bufstr);
				Trim(bufstr);
				std::getline(in, bufstr);
				std::getline(in, bufstr);
				std::getline(in, bufstr);
				//cur_node = NewNode(last_joint_name + "End", cur_node);
				continue;
			}
			if (bufstr == "{") {
				nodes.push(cur_node);
				continue;
			}
			if (bufstr == "}") {
				nodes.pop();
				cur_node = nodes.top();
				continue;
			}
			if (bufstr == "MOTION") break;
		}

		if ( !root_offset && !m_joints.empty() )
		{
			m_joints[0].offset.set(0, 0, 0);
		}

		if (human_load) {
			motion->body(new Body(m_joints, m_jointMap)); //用bvh解析的数据，构建一个ml::body_Skeleton
		}

		motion->m_channels = m_channels;
		motion->num_channels = num_channels;

		int fcount;
		double ftime;
		in >> bufstr;
		in >> fcount;
		in >> bufstr;
		in >> bufstr;
		in >> ftime;

		motion->size(fcount / sample);
		motion->fps(std::round(1. / ftime));

		Posture p(m_joints.size());
		p.body(motion->body());

		double time = 0.;

		for (int i = 0; i < fcount; i++) {
			int joint_index = 0;
			size_t channel_index = 0;
			double value;

			matrix3 rotate = identity_mat();
			vector3 trans(0, 0, 0);
			for (int j = 0; j < num_channels; ++j) {
				in >> value;

				while (channel_index >= (int)m_channels[joint_index].size()) {
					++joint_index;
					channel_index = 0;
				}
				switch (m_channels[joint_index][channel_index]) {
				case XPOS:
					trans[0] += value * scale;
					break;
				case YPOS:
					trans[1] += value * scale;
					break;
				case ZPOS:
					trans[2] += value * scale;
					break;
				case XROT:
					rotate *= rotx_mat(deg2rad(value));
					break;
				case YROT:
					rotate *= roty_mat(deg2rad(value));
					break;
				case ZROT:
					rotate *= rotz_mat(deg2rad(value));
					break;
				}
				++channel_index;
				if (channel_index >= m_channels[joint_index].size()) {
					/// ignore translate term if the joint is not root
					if (joint_index == 0) {
						p.trans(convert_unreal_m * trans);
						trans.zero();
					}
					p.rotate(joint_index,  convert_unreal_m * rotate * convert_unreal_m_inv);
					rotate.identity();
				}
			}
			p.time = time;
			time += ftime;
			motion->posture(i / sample, p);
		}
	}

	void BVHReader::LoadBVH(const char* file, Motion* motion, bool root_offset, bool human_load, double scale, int sample)
	{
		std::ifstream in;
		std::string bufstr;

		m_channels.clear();
		m_joints.clear();
		m_jointMap.clear();
		in.open(file);

		std::stack<int> nodes;
		nodes.push(-1);
		int cur_node = -1;
		int num_channels = 0;

		std::string last_joint_name;

		while (in) {
			in >> bufstr;

			if (bufstr == "ROOT") {
				std::getline(in, bufstr);
				Trim(bufstr);

				cur_node = NewNode(bufstr, -1);
				continue;
			}
			if (bufstr == "OFFSET") {
				double x, y, z;
				in >> x >> y >> z;
				x *= scale;
				y *= scale;
				z *= scale;
				m_joints[cur_node].offset = cml::vector3(x, y, z);
				continue;
			}
			if (bufstr == "CHANNELS") {
				int n;
				in >> n;
				num_channels += n;
				for (int i = 0; i < n; ++i) {
					in >> bufstr;
					if (bufstr == "Xrotation") AddChannel(cur_node, XROT);
					if (bufstr == "Yrotation") AddChannel(cur_node, YROT);
					if (bufstr == "Zrotation") AddChannel(cur_node, ZROT);

					if (bufstr == "Xposition") AddChannel(cur_node, XPOS);
					if (bufstr == "Yposition") AddChannel(cur_node, YPOS);
					if (bufstr == "Zposition") AddChannel(cur_node, ZPOS);
				}
				continue;
			}
			if (bufstr == "JOINT") {
				std::getline(in, bufstr);
				Trim(bufstr);
				last_joint_name = bufstr;
				cur_node = NewNode(bufstr, cur_node);
				continue;
			}
			if (bufstr == "END" || bufstr == "End") {
				std::getline(in, bufstr);
				Trim(bufstr);
				cur_node = NewNode(last_joint_name + "End", cur_node);
				continue;
			}
			if (bufstr == "{") {
				nodes.push(cur_node);
				continue;
			}
			if (bufstr == "}") {
				nodes.pop();
				cur_node = nodes.top();
				continue;
			}
			if (bufstr == "MOTION") break;
		}

		if ( !root_offset && !m_joints.empty() )
		{
			m_joints[0].offset.set(0, 0, 0);
		}

		if (human_load) {
			motion->body(new Body(m_joints, m_jointMap));
		}

		int fcount;
		double ftime;
		in >> bufstr;
		in >> fcount;
		in >> bufstr;
		in >> bufstr;
		in >> ftime;

		motion->size(fcount / sample);
		motion->fps(std::round(1. / ftime));

		Posture p(m_joints.size());
		p.body(motion->body());

		double time = 0.;

		for (int i = 0; i < fcount; i++) {
			int joint_index = 0;
			size_t channel_index = 0;
			double value;

			matrix3 rotate = identity_mat();
			vector3 trans(0, 0, 0);
			for (int j = 0; j < num_channels; ++j) {
				in >> value;

				while (channel_index >= (int)m_channels[joint_index].size()) {
					++joint_index;
					channel_index = 0;
				}
				switch (m_channels[joint_index][channel_index]) {
				case XPOS:
					trans[0] += value * scale;
					break;
				case YPOS:
					trans[1] += value * scale;
					break;
				case ZPOS:
					trans[2] += value * scale;
					break;
				case XROT:
					rotate *= rotx_mat(deg2rad(value));
					break;
				case YROT:
					rotate *= roty_mat(deg2rad(value));
					break;
				case ZROT:
					rotate *= rotz_mat(deg2rad(value));
					break;
				}
				++channel_index;
				if (channel_index >= m_channels[joint_index].size()) {
					/// ignore translate term if the joint is not root
					if (joint_index == 0) {
						p.trans(trans);
						trans.zero();
					}
					p.rotate(joint_index, rotate);
					rotate.identity();
				}
			}
			p.time = time;
			time += 1.0;
			motion->posture(i / sample, p);
		}
	}

	bool BVHReader::LoadBVHOneMotion_UE4(FString file, TArray<float>& OutData)
	{
		std::ifstream in;
		std::string bufstr;
		in.open(TCHAR_TO_ANSI(*file));
		in >> bufstr; //Motion
		while (in) {
			float temp;
			in >> temp;
			OutData.Add(temp);
		}
		return true;
	}


	int AMCReader::NewNode(const std::string& name)
	{
		int new_index = (int)m_joints.size();

		AMCJoint joint;
		joint.parent = -1;
		joint.name = name;

		m_joints.push_back(joint);
		m_jointMap[name] = new_index;
		m_channels.push_back(std::vector<Channel>());

		return new_index;
	}

	void AMCReader::LoadAMC(const char *amc_file, const char *asf_file, Motion *motion, bool human_load, double scale, int sample)
	{
		std::ifstream in;
		std::string bufstr;
		int cur_node = -1;
		int num_leaf = 0;

		m_channels.clear();
		m_joints.clear();
		m_jointMap.clear();

		in.open(asf_file);
		if (in.fail()) return;

		while (in)
		{
			in >> bufstr;
			if (bufstr == ":bonedata") break;
		}


		getline(in, bufstr);

		cur_node = NewNode("root");

		m_joints[0].offset = vector3(0, 0, 0);
		m_joints[0].dir = vector3(0, 0, 0);
		m_joints[0].m.identity();
		m_joints[0].parent = -1;
		m_joints[0].is_parent = false;

		m_channels[0].push_back(XPOS);
		m_channels[0].push_back(YPOS);
		m_channels[0].push_back(ZPOS);
		m_channels[0].push_back(XROT);
		m_channels[0].push_back(YROT);
		m_channels[0].push_back(ZROT);

		double tx =0, ty=0, tz=0, rx=0, ry=0, rz=0, s=0;
		while (in)
		{
			in >> bufstr;
			if (bufstr == ":hierarchy") break;
			while (1)
			{
				in >> bufstr;

				if (bufstr == "id") {
					in >> bufstr;
					continue;
				}
				if (bufstr == "name") {
					getline(in, bufstr);
					Trim(bufstr);
					cur_node = NewNode(bufstr);
					continue;
				}
				if (bufstr == "direction") {
					in >> tx >> ty >> tz;
					continue;
				}
				if (bufstr == "length") {
					in >> s;
					continue;
				}
				if (bufstr == "axis") {
					in >> rx >> ry >> rz >> bufstr;
					rx = M_PI * rx / 180;
					ry = M_PI * ry / 180;
					rz = M_PI * rz / 180;
					continue;
				}
				if (bufstr == "dof") {
					getline(in, bufstr);
					stringstream sst(bufstr);
					string token;
					while (sst) {
						sst >> token;
						if (token == "rx") m_channels[cur_node].push_back(XROT);
						if (token == "ry") m_channels[cur_node].push_back(YROT);
						if (token == "rz") m_channels[cur_node].push_back(ZROT);
					}
				}
				if (bufstr == "end") {

					break;
				}
			}
			m_joints[cur_node].dir = vector3(tx, ty, tz) * s * scale;
			m_joints[cur_node].m = rotz_mat(rz) * roty_mat(ry) * rotx_mat(rx);
			m_joints[cur_node].is_parent = false;
		}
		// read 'begin'
		in >> bufstr;
		while (in) {
			string parent;
			string token;
			in >> parent;
			if (parent == "end") break;
			getline(in, bufstr);
			stringstream sst(bufstr);
			while (sst) {
				sst >> token;
				m_joints[m_jointMap[token]].parent = m_jointMap[parent];
				m_joints[m_jointMap[parent]].is_parent = true;
			}
		}

		in.close();

		std::vector<Joint> joints;
		std::map<std::string, int> joint_map;
		std::map<int, int> index_map;
		std::map<int, int> inverse_index_map;

		joints.push_back(Joint());
		joints[0].offset = vector3(0, 0, 0);
		joints[0].parent = -1;
		joint_map["root"] = 0;

		index_map[0] = 0;
		inverse_index_map[0] = 0;

		for (int i = 1; i < (int)m_joints.size(); ++i) {
			// check a joint is dummy
			if (m_channels[i].size()) {
				int cur_index = joints.size();

				index_map[cur_index] = i;
				inverse_index_map[i] = cur_index;
				joint_map[m_joints[i].name] = cur_index;

				joints.push_back(Joint());
				vector3 offset(0, 0, 0); //m_joints[i].offset;
				int new_parent = m_joints[i].parent;
				while (!m_channels[new_parent].size()) {
					offset += m_joints[new_parent].dir;
					new_parent = m_joints[new_parent].parent;
				}
				offset += m_joints[new_parent].dir;
				joints[cur_index].offset = offset;
				joints[cur_index].parent = inverse_index_map[new_parent];
			}
		}

		// add leaf dummy nodes
		for (int i = 1; i < (int)m_joints.size(); ++i) {
			if (!m_joints[i].is_parent) {
				int cur_index = (int)joints.size();
				index_map[cur_index] = -2;
				joint_map[m_joints[i].name + "End"] = cur_index;

				joints.push_back(Joint());
				joints[cur_index].offset = m_joints[i].dir;
				joints[cur_index].parent = inverse_index_map[i];

				num_leaf++;
			}
		}


		if (human_load) {
			motion->body(new Body(joints, joint_map));
		}

		motion->size(0);

		Posture p(joints.size());
		p.body(motion->body());

		in.open(amc_file);
		if (in.fail()) return;

		while (in) {
			in >> bufstr;
			if (bufstr == ":DEGREES") break;
		}

		double time = 0.0;
		while (in) {
			int fno;
			string token;

			in >> fno;
			if (!in) break;

			for (int i = 0; i < (int)joints.size() - num_leaf; ++i) {
				in >> token;
				if (token == "root") {
					in >> tx >> ty >> tz >> rx >> ry >> rz;
					tx *= scale;
					ty *= scale;
					tz *= scale;
					p.trans(vector3(tx, ty, tz));
					rx = M_PI * rx / 180;
					ry = M_PI * ry / 180;
					rz = M_PI * rz / 180;

					cml::matrix3 axis_m = m_joints[0].m;
					p.rotate(0,
						axis_m
						*  rotz_mat(rz) * roty_mat(ry) * rotx_mat(rx) *
						cml::inverse(axis_m)
					);
				}
				else {
					int joint_id = joint_map[token];
					cml::matrix3 axis_m = m_joints[index_map[joint_id]].m;

					in >> rx >> ry >> rz;
					rx = M_PI * rx / 180;
					ry = M_PI * ry / 180;
					rz = M_PI * rz / 180;
					p.rotate(joint_id,
						axis_m
						* rotz_mat(rz) * roty_mat(ry) * rotx_mat(rx) *
						cml::inverse(axis_m)
					);
				}
			}
			for (int i = 0; i < num_leaf; ++i) {
				p.rotate((int)joints.size() - 1 - i, identity_3x3());
			}

			p.time = time;
			motion->AddPosture(p);
			time += 1.0;
		}
	}

}