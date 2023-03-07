/*
 *  motion.h
 *  unagi
 *
 *  Created by normit on 09. 09. 09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 *  Edited by Myung Geol Choi since 2016. 08. 12
 *  Edited by Haminglu since 2022. 8. 12
 */
#pragma once
#include <iterator>
#include "ccml.h"
#include "body.h"
#include "posture.h"
#include <vector>

namespace ml 
{


class Motion 
{
public:
	friend Motion& stitch(Motion m1, Motion m2, int warp_width);

	Motion(Body *body = NULL);

	template <class In>
	Motion(const Motion &other, In i, In j) {
		copy_common(other);
		std::copy(i, j, std::back_inserter(m_postures));
	}

	////////////////////////////////////////////////////////////////////////////////////
	// Assign
	template <class In>
	void copy(const Motion &other, In i, In j) {
		m_postures.clear();
		copy_common(other);
		std::copy(i, j, std::back_inserter(m_postures));
	}
	void copy_common(const Motion &other);
	void get_motion(const Motion &other, int first, int last);

	virtual ~Motion();

	


	////////////////////////////////////////////////////////////////////////////////////
	// Body
	const Body *body() const { return m_body; }
	Body *editable_body() const { return m_body; }
	void body(Body *body) { m_body = body; }



	////////////////////////////////////////////////////////////////////////////////////
	// Access Postures
	Posture&		posture(int i) { return m_postures[i]; }
	const Posture&	posture(int i) const { return m_postures[i]; }
	void			posture(int i, const Posture& p);
	Posture&		last_posture() { return m_postures[size() - 1]; }
	const Posture&	last_posture() const { return m_postures[size() - 1]; }
	Posture&		first_posture() { return m_postures[0]; }
	const Posture&	first_posture() const { return m_postures[0]; }
	Posture&		posture_at_time(double time) { return *std::min_element(begin(), end(), [time](const ml::Posture &p1, const ml::Posture &p2) {return fabs(p1.time - time) < fabs(p2.time - time); }); }
	const Posture&	posture_at_time(double time)const { return *std::min_element(begin(), end(), [time](const ml::Posture &p1, const ml::Posture &p2) {return fabs(p1.time - time) < fabs(p2.time - time); }); }
	Posture			GetPostureAtTime_ms(float ms) const;


	////////////////////////////////////////////////////////////////////////////////////
	// std-style Iterator for Postures
	typedef std::vector<Posture>::iterator Iterator;
	typedef std::vector<Posture>::const_iterator Const_iterator;

	Iterator		begin() { return m_postures.begin(); }
	Const_iterator	begin() const { return m_postures.begin(); }
	Iterator		end() { return m_postures.end(); }
	Const_iterator	end() const { return m_postures.end(); }
	Iterator		iterator_at(double time) { return std::min_element(begin(), end(), [time](const ml::Posture &p1, const ml::Posture &p2) {return fabs(p1.time - time) < fabs(p2.time - time); }); }
	Const_iterator	iterator_at(double time) const { return std::min_element(begin(), end(), [time](const ml::Posture &p1, const ml::Posture &p2) {return fabs(p1.time - time) < fabs(p2.time - time); }); }
	inline Posture& operator[](int i) { return m_postures[i]; }
	const Posture& operator[](int i) const { return m_postures[i]; }
	void size(int size);
	size_t size() const { return m_postures.size(); }


	//////////////////////////////////////////////////////////////////////////////////////////
	// Edit
	void AddPosture(const Posture& p);
	void Stitch(const Motion & const_add_m, bool forward = true);
	void Stitch_UE4(const Motion & const_add_m, bool forward = true);
	void SwapBody(Body *toBody);
	void Sample(double rate);

	/**
	* Add for runtime hml
	*/
	Posture CreatePosture_UE(TArray<float> ChannelInfoArr);

	//////////////////////////////////////////////////////////////////////////////////////////
	// Clone
	Motion* CropMotion(int from, int size, Motion *ret_motion = 0) const;
	Motion* Clone(Motion *ret_motion = 0) const;

	


	//////////////////////////////////////////////////////////////////////////////////////////////
	// I/O
	void LoadAMC(const char *amc_file, const char *asf_file, bool human_load = true, double scale = 1.0);
	void LoadAMC_with_contactInfo(const char *amc_file, const char *asf_file, bool human_load = true, double scale = 1.0, double ground_truth = 0.0);
	void LoadBVH(const char *file, bool root_offset, bool human_load = true, double scale = 1.0, int sample = 1);


	//////////////////////////////////////////////////////////////////////////////////////////////
	// IK
	void IkLimbSmoothly(const size_t frame, const int fduration, const int bduration, const size_t joint, const cml::vector3& pos);
	void SetFootConstraint(int ltoe_j, int lfoot_j, int rtoe_j, int rfoot_j, double toe_speed, double toe_height, double ankle_speed, double ankle_height);


	//////////////////////////////////////////////////////////////////////////////////////////////
	// Transformation
	void translate(const cml::vector3d &v);
	void translateSmoothly(const size_t posture_frame, const int forward_duration, const int backward_duration, const cml::vector3& pin_pos);
	void rotate(double theta);
	void translate_time(double time);
	void translate_time_to_zero();
	void ApplyTransf(const cml::transf &t, double time = 0.0);
	void transform_between_posture(ml::Posture to_posture, ml::Posture from_posture);



	//////////////////////////////////////////////////////////////////////////////////////////////
	// Properties
	void fps(double f) { fps_ = f; }
	double fps() const { return fps_; }

	void name(std::string n) { name_ = n; }
	std::string name() const { return name_; }

public:
	//辅助参数，记录下BVH生成此motion时的channel
	std::vector<std::vector<Channel> > m_channels;
	int num_channels = 0;

protected:
	std::vector<Posture> m_postures;
	Body *m_body;
	Motion *connected_fore;
	Motion *connected_back;
	bool checked;
	double fps_;

	std::string name_;
};


class MotionFrame
{
public:
	MotionFrame() { motion_=nullptr; frame_id_=0; }
	MotionFrame(Motion *m, int f) { motion_ = m; frame_id_ = f; }

	void motion(Motion *m) { motion_ = m; }
	void frame_id(int f) { frame_id_ = f; }
	Posture* posture() const { return &(motion_->posture(frame_id_)); };
	Motion* motion() const { return motion_; }
	int frame_id() const { return frame_id_; }

	bool operator==(const MotionFrame& rhs) 
	{ 
		return (motion_==rhs.motion_)&&(frame_id_==rhs.frame_id()); 
	}

	bool operator!=(const MotionFrame& rhs) 
	{ 
		return (motion_!=rhs.motion_)||(frame_id_!=rhs.frame_id()); 
	}


private:
	int frame_id_;
	Motion *motion_;
};

////////////////////////////////////////////////////
// Algorithms
void CalculLinearKineticEnergy(const Motion *m, std::vector<double> &out);
void CalculLinearKineticEnergy(const Motion *m, int start_frame, int end_frame, std::vector<double> &out);
void SelectKeyFramesByKineticEnergy(const Motion *m , std::vector<int> &out_key_frames);

};

