/*
 *  reader.h
 *  unagi
 *
 *  Created by normit on 09. 09. 10.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */
#pragma once
#include "ccml.h"
#include "body.h"
#include <map>
#include <vector>
#include "CoreMinimal.h"


namespace ml {
	class Motion;
	class BVHReader
	{
	public:
		/**
		@param root_offset If root_offset is false, then it will not load the offset value of the root node in the hierarchy section
		and	set the offset to (0, 0, 0). In most BVH files, the root translation data in the motion section are global potisions, 
		so the root offset value in the hierarcy section must be ignored when evaluating global positions.
		*/
		void LoadBVH(const char *file, Motion *motion, bool root_offset=false, bool human_load=true, double scale=1.0, int sample = 1);
		void LoadBVH_UE4(const char *file, Motion *motion, bool root_offset=false, bool human_load=true, double scale=1.0, int sample = 1);

		//加个辅助函数，只读Motion
		bool LoadBVHOneMotion_UE4(FString file, TArray<float>& OutData);

	protected:
		//enum Channel { XPOS = 1, YPOS, ZPOS, ZROT, XROT, YROT };

		void AddChannel(int i, Channel ch);

		int NewNode(const std::string&name, int parent);

		std::vector<std::vector<Channel> > m_channels;
		std::vector<Joint> m_joints;
		std::map<std::string, int> m_jointMap;

	};

	bool LoadBVH(const char *file, Motion &out_motion, bool root_offset=false, bool human_load=true, double scale=1.0, int sample=1);
	bool LoadBVH_UE4(const char *file, Motion &out_motion, bool root_offset=false, bool human_load=true, double scale=1.0, int sample=1);
	//bool LoadBVH_UE4(FString file, Motion &out_motion, bool root_offset=false, bool human_load=true, double scale=1.0, int sample=1);
	//Load and pasre
	bool LoadBVH_UE4(FString T_motion_file, FString file, Motion &out_motion, bool root_offset=false, bool human_load=true, double scale=1.0, int sample=1);

	class AMCReader
	{
	public:
		void LoadAMC(const char *amc_file, const char *asf_file, Motion *motion, bool human_load=true, double scale=1.0, int sample=1);
	protected:
		//enum Channel { XPOS = 1, YPOS, ZPOS, ZROT, XROT, YROT };
		struct AMCJoint
		{
			std::string name;
			cml::vector3 dir;
			cml::vector3 offset;
			cml::matrix3 m;
			bool is_parent;
			int parent;
		};
		void AddChannel(int i, Channel ch);
		int NewNode(const std::string& name);

		std::vector<std::vector<Channel> > m_channels;
		std::vector<AMCJoint> m_joints;
		std::map<std::string, int> m_jointMap;
	};

};