#include "func.h"

#include <vector>
#include <fstream>
#include "common.h"

namespace ml {

	double DiffPosture(const Posture& p1, const Posture& p2)
	{
		double ret_error = 0.;
		for (size_t i = 0; i < p1.body()->num_joint(); ++i) {
			ret_error += (p1.GetGlobalTranslation(i) - p2.GetGlobalTranslation(i)).length_squared();
		}

		return ret_error;
	}

	double DiffPoseGlobalPos(const Posture& p1, const Posture& p2)
	{
		double ret_error = 0.;
		for (size_t i = 0; i < p1.body()->num_joint(); ++i) {
			ret_error += (p1.GetGlobalTranslation(i) - p2.GetGlobalTranslation(i)).length_squared();
		}

		return ret_error;
	}

	double DiffPoseLocalPos(const Motion& m1, int f1, const Motion& m2, int f2)
	{
		return DiffPoseLocalPos(m1.posture(f1), m2.posture(f2));
	}


	double DiffPoseLocalPos(const Posture& p1, const Posture& p2)
	{
		Posture p1_0 = p1;
		p1_0.trans(cml::vector3(0, 0, 0));
		p1_0.m_rotates[0].identity();

		Posture p2_0 = p2;
		p2_0.trans(cml::vector3(0, 0, 0));
		p2_0.m_rotates[0].identity();

		return DiffPoseGlobalPos(p1_0, p2_0);
	}

	// d = p2 - p1
	std::pair<cml::vector3d, std::vector<cml::vector3d>> difference(const Posture &p2, const Posture &p1)
	{
		cml::vector3d diff_trans = p2.trans() - p1.trans();
		std::vector<cml::vector3d> diff_oris;
		for (int k = 0; k < (int)p2.body()->num_joint(); ++k)
		{
			diff_oris.push_back(cml::log_mat3(cml::inverse(p1.rotate(k)) * p2.rotate(k)));
		}
		return std::make_pair(diff_trans, diff_oris);
	}

	std::pair<cml::vector3d, cml::vector3d> difference_root(const Posture &p2, const Posture &p1)
	{
		cml::vector3d diff_trans = p2.trans() - p1.trans();
		int k = 0;
		return std::make_pair(diff_trans, cml::log_mat3(cml::inverse(p1.rotate(k)) * p2.rotate(k)));
	}

	// p + diff
	void add_difference(Posture &p, std::pair<cml::vector3d, std::vector<cml::vector3d>> &diff, double ratio)
	{
		p.trans(p.trans() + diff.first * ratio);
		for (int k = 0; k < (int)p.body()->num_joint(); ++k)
		{
			p.rotate(k, p.rotate(k) * cml::exp_mat3(diff.second[k] * ratio));
		}
	}

	void add_difference_root_direction_height(Posture &p, std::pair<cml::vector3d, cml::vector3d> &diff, double ratio)
	{
		//p.trans(p.trans() + diff.first * ratio);
		p.m_trans[1] = p.m_trans[1] + (diff.first)[1] * ratio;
		for (int k = 0; k < 1; ++k)
		{
			p.rotate(k, p.rotate(k) * cml::exp_mat3(diff.second * ratio));
		}
	}


	Motion& stitch(Motion m1, Motion m2, int warp_width /*= 20 */)
	{
		//translate and rotate ( diff_rot * p2_rot = p1_rot )
		Posture p1 = m1[m1.size() - 1];
		Posture p2 = m2[0];
		cml::matrix3 diff_rot = cml::PlaneProject_mat3(p1.rotate(0) * cml::inverse(p2.rotate(0)));
		for (int i = 0; i < (int)m2.size(); ++i)
		{
			Posture &p = m2[i];
			double height = p.trans()[1];
			cml::vector3d diff_v = p.trans() - p2.trans();
			cml::vector3d new_pos = p1.trans() + diff_rot * diff_v;
			new_pos[1] = height;

			p.trans(new_pos);
			p.rotate(0, diff_rot * p.rotate(0));
		}

		//warp
		p1 = m1[m1.size() - 1];
		p2 = m2[0];
		Posture mid = p1;
		std::pair<cml::vector3d, std::vector<cml::vector3d>> diff_tmp;
		add_difference(mid, diff_tmp = difference(p2, p1), 0.5);
		std::pair<cml::vector3d, std::vector<cml::vector3d>> diff_mid_p1 = difference(mid, p1);
		std::pair<cml::vector3d, std::vector<cml::vector3d>> diff_mid_p2 = difference(mid, p2);

		int half_width = warp_width / 2;
		for (int i = (int)m1.size() - 1 - half_width; i <= (int)m1.size() - 1; ++i)
		{
			double x = (i - ((int)(m1.size()) - 1)) / double(half_width + 1);
			double ratio = cml::blend_weight(x);
			Posture &p = m1[i];
			add_difference(p, diff_mid_p1, ratio);
		}

		for (int i = 0; i <= half_width; ++i)
		{
			double x = i / double(half_width + 1);
			double ratio = cml::blend_weight(x);
			Posture &p = m2[i];
			add_difference(p, diff_mid_p2, ratio);
		}

		//stitch
		Motion &m = *(new Motion(m1));
		m.m_postures.insert(m.m_postures.end(), m2.m_postures.begin() + 1, m2.m_postures.end());
		return m;
	}

	double scalarTransitionFunc(const double t, const double range)
	{
		return 0.5*std::cos(cml::pi()*t / range) + 0.5;
	}


	void warp(ml::Motion * before_m, Motion * after_m) {
		Posture before_p = before_m->last_posture();
		Posture after_p = after_m->first_posture();

		int warp_width = 20;
		int before_width = ((int)before_m->size() < warp_width) ? before_m->size() : warp_width;
		int after_width = ((int)after_m->size() < warp_width) ? after_m->size() : warp_width;

		Posture mid = before_p;
		std::pair<cml::vector3d, std::vector<cml::vector3d>> tmp_diff;
		add_difference(mid, tmp_diff = difference(after_p, before_p), (double)before_width / (double)(before_width + after_width));

		for (int i = (int)before_m->size() - 1 - (before_width - 1); i <= (int)before_m->size() - 1; ++i) {
			double x = (i - ((int)(before_m->size()) - 1)) / double(before_width);
			double ratio = cml::blend_weight(x);
			Posture &p = before_m->posture(i);
			std::pair<cml::vector3d, std::vector<cml::vector3d>> tmp_diff2;
			add_difference(p, tmp_diff2 = difference(mid, before_p), ratio);
		}
		for (int i = 0; i <= (after_width - 1); ++i) {
			double x = i / double(after_width);
			double ratio = cml::blend_weight(x);
			Posture &p = after_m->posture(i);
			std::pair<cml::vector3d, std::vector<cml::vector3d>> tmp_diff2;
			add_difference(p, tmp_diff2 = difference(mid, after_p), ratio);
		}
	}

	void warp_root_direction_height(ml::Motion * before_m, Motion * after_m, int before_warp_width, int after_warp_width) {
		Posture before_p = before_m->last_posture();
		Posture after_p = after_m->first_posture();

		int before_width = ((int)before_m->size() < before_warp_width) ? before_m->size() : before_warp_width;
		int after_width = ((int)after_m->size() < after_warp_width) ? after_m->size() : after_warp_width;

		Posture mid = before_p;
		std::pair<cml::vector3d, cml::vector3d> tmp_diff;
		add_difference_root_direction_height(mid, tmp_diff = difference_root(after_p, before_p), (double)before_width / (double)(before_width + after_width));

		for (int i = (int)before_m->size() - 1 - (before_width - 1); i <= (int)before_m->size() - 1; ++i) {
			double x = (i - ((int)(before_m->size()) - 1)) / double(before_width);
			double ratio = cml::blend_weight(x);
			Posture &p = before_m->posture(i);
			std::pair<cml::vector3d, cml::vector3d> tmp_diff2;
			add_difference_root_direction_height(p, tmp_diff2 = difference_root(mid, before_p), ratio);
		}
		for (int i = 0; i <= (after_width - 1); ++i) {
			double x = i / double(after_width);
			double ratio = cml::blend_weight(x);
			Posture &p = after_m->posture(i);
			std::pair<cml::vector3d, cml::vector3d> tmp_diff2;
			add_difference_root_direction_height(p, tmp_diff2 = difference_root(mid, after_p), ratio);
		}
	}



	//////////////////////////////////////////////////////////

	void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]) {
		res[0] = FMath::Atan2(r11, r12);
		res[1] = FMath::Acos(r21);
		res[2] = FMath::Atan2(r31, r32);
	}

	void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]) {
		res[0] = FMath::Atan2(r31, r32);
		res[1] = FMath::Acos(r21);
		res[2] = FMath::Atan2(r11, r12);
	}

	void quaternion2Euler(const FQuat& q, double res[], RotSeq rotSeq)
	{
		switch (rotSeq) {
		case zyx:
			threeaxisrot(2 * (q.X * q.Y + q.W * q.Z),
				q.W * q.W + q.X * q.X - q.Y * q.Y - q.Z * q.Z,
				-2 * (q.X * q.Z - q.W * q.Y),
				2 * (q.Y * q.Z + q.W * q.X),
				q.W * q.W - q.X * q.X - q.Y * q.Y + q.Z * q.Z,
				res);
			break;

		case zyz:
			twoaxisrot(2 * (q.Y * q.Z - q.W * q.X),
				2 * (q.X * q.Z + q.W * q.Y),
				q.W * q.W - q.X * q.X - q.Y * q.Y + q.Z * q.Z,
				2 * (q.Y * q.Z + q.W * q.X),
				-2 * (q.X * q.Z - q.W * q.Y),
				res);
			break;

		case zxy:
			threeaxisrot(-2 * (q.X * q.Y - q.W * q.Z),
				q.W * q.W - q.X * q.X + q.Y * q.Y - q.Z * q.Z,
				2 * (q.Y * q.Z + q.W * q.X),
				-2 * (q.X * q.Z - q.W * q.Y),
				q.W * q.W - q.X * q.X - q.Y * q.Y + q.Z * q.Z,
				res);
			break;

		case zxz:
			twoaxisrot(2 * (q.X * q.Z + q.W * q.Y),
				-2 * (q.Y * q.Z - q.W * q.X),
				q.W * q.W - q.X * q.X - q.Y * q.Y + q.Z * q.Z,
				2 * (q.X * q.Z - q.W * q.Y),
				2 * (q.Y * q.Z + q.W * q.X),
				res);
			break;

		case yxz:
			threeaxisrot(2 * (q.X * q.Z + q.W * q.Y),
				q.W * q.W - q.X * q.X - q.Y * q.Y + q.Z * q.Z,
				-2 * (q.Y * q.Z - q.W * q.X),
				2 * (q.X * q.Y + q.W * q.Z),
				q.W * q.W - q.X * q.X + q.Y * q.Y - q.Z * q.Z,
				res);
			break;

		case yxy:
			twoaxisrot(2 * (q.X * q.Y - q.W * q.Z),
				2 * (q.Y * q.Z + q.W * q.X),
				q.W * q.W - q.X * q.X + q.Y * q.Y - q.Z * q.Z,
				2 * (q.X * q.Y + q.W * q.Z),
				-2 * (q.Y * q.Z - q.W * q.X),
				res);
			break;

		case yzx:
			threeaxisrot(-2 * (q.X * q.Z - q.W * q.Y),
				q.W * q.W + q.X * q.X - q.Y * q.Y - q.Z * q.Z,
				2 * (q.X * q.Y + q.W * q.Z),
				-2 * (q.Y * q.Z - q.W * q.X),
				q.W * q.W - q.X * q.X + q.Y * q.Y - q.Z * q.Z,
				res);
			break;

		case yzy:
			twoaxisrot(2 * (q.Y * q.Z + q.W * q.X),
				-2 * (q.X * q.Y - q.W * q.Z),
				q.W * q.W - q.X * q.X + q.Y * q.Y - q.Z * q.Z,
				2 * (q.Y * q.Z - q.W * q.X),
				2 * (q.X * q.Y + q.W * q.Z),
				res);
			break;

		case xyz:
			threeaxisrot(-2 * (q.Y * q.Z - q.W * q.X),
				q.W * q.W - q.X * q.X - q.Y * q.Y + q.Z * q.Z,
				2 * (q.X * q.Z + q.W * q.Y),
				-2 * (q.X * q.Y - q.W * q.Z),
				q.W * q.W + q.X * q.X - q.Y * q.Y - q.Z * q.Z,
				res);
			break;

		case xyx:
			twoaxisrot(2 * (q.X * q.Y + q.W * q.Z),
				-2 * (q.X * q.Z - q.W * q.Y),
				q.W * q.W + q.X * q.X - q.Y * q.Y - q.Z * q.Z,
				2 * (q.X * q.Y - q.W * q.Z),
				2 * (q.X * q.Z + q.W * q.Y),
				res);
			break;

		case xzy:
			threeaxisrot(2 * (q.Y * q.Z + q.W * q.X),
				q.W * q.W - q.X * q.X + q.Y * q.Y - q.Z * q.Z,
				-2 * (q.X * q.Y - q.W * q.Z),
				2 * (q.X * q.Z + q.W * q.Y),
				q.W * q.W + q.X * q.X - q.Y * q.Y - q.Z * q.Z,
				res);
			break;

		case xzx:
			twoaxisrot(2 * (q.X * q.Z - q.W * q.Y),
				2 * (q.X * q.Y + q.W * q.Z),
				q.W * q.W + q.X * q.X - q.Y * q.Y - q.Z * q.Z,
				2 * (q.X * q.Z + q.W * q.Y),
				-2 * (q.X * q.Y - q.W * q.Z),
				res);
			break;
		default:
			std::cout << "Unknown rotation sequence" << std::endl;
			break;
		}
	}
}