#pragma once

#include <cml/cml.h>


namespace cml
{
	typedef quaternion<double, fixed<>, scalar_first> quater;
	typedef vector2d vector2;
	typedef vector3d vector3;
	typedef vector4d vector4;
	typedef matrix44d transf;
	typedef matrix33d matrix3;
	
	typedef matrix44d SE3;
	typedef matrix33d SO3;

	double pi();

	double deg2rad(double deg);
	double rad2deg(double rad);

	// double ACOS(double x);
	double blend_weight(double x);

	vector2 vec2(const vector3& v);
	vector3 vec3(const vector2& v);
	vector3 vec3(const vector4& v);
	vector4 pos4(const vector3& v);
	vector4 vec4(const vector3& v);
	vector3 plane_vec3(const vector3& v);

	vector3 log_mat3(const matrix3& mat);
	matrix3 exp_mat3(const vector3& v);

	int GetCcw(const vector2& p1, const vector2& p2, const vector2& p3);

	double cross_vec2(const vector2& v1, const vector2 &v2);

	vector3 trans(const transf& t);

	vector3 between_vector(const vector3 &a, const vector3 &b);
	double angle_distance(const matrix3 &mat1, const matrix3 &mat2);

	matrix3 identity_mat();
	matrix3 rotx_mat(double theta);
	matrix3 roty_mat(double theta);
	matrix3 rotz_mat(double theta);
	matrix3 interpolate(const matrix3& m1, const matrix3& m2, double ratio);
	matrix3 PlaneProject_mat3(const matrix3& m);
	matrix3 mat3(const transf& f);

	transf identity_transf();
	transf make_transf(const matrix3& mat, const vector3& v);
	transf trans_transf(double x, double y, double z);
	transf trans_transf(const vector3& v);
	transf rotx_transf(double theta);
	transf roty_transf(double theta);
	transf rotz_transf(double theta);
	transf PlaneProject_transf(const transf& f);

	double SpeedFunc(double t);

	void ConvexHull(std::vector<vector2>& points);
	void ConvexHull2(std::vector<vector2>& points);
	void ConvexHull3(std::vector<vector2>& points);
	void ConvexHullwithIndex(const std::vector<vector2>& points, std::vector<int> &index);
	bool InterSect(const cml::vector2 &a0, const cml::vector2 &b0, const cml::vector2& a1, const cml::vector2& b1, cml::vector2 *out);
}

int random(int lowest, int highest);
double random_float(double lowest, double hightest, double unit);
void get_diff2s(std::vector<cml::vector2d> &diff2s);
double gaussian_function(const double x, const double m, const double sigma, const double amp=1.0);